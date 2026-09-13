import './styles.css';
import compiled from '../data/compiled.json';
import { ARM, buildEpisodeArtifact, clamp, distance, forwardKinematics, guidedStep, MAX_EPISODE_TRANSITIONS, projectToReachableWorkspace, solveInverseKinematics } from './core.js';

const $ = (selector) => document.querySelector(selector);
const fmt = (value, digits = 2) => Number(value).toFixed(digits);

const state = {
  goal: [...compiled.workflows[0].goal],
  q: [-0.45, 0.2, 0.3, -0.2, -0.1, 0.15],
  lastAction: Array(6).fill(0),
  running: false,
  recording: false,
  transitions: [],
  step: 0,
  startedAt: performance.now(),
  currentWorkflow: compiled.workflows[0],
  guidancePlan: null,
  replaying: false,
  voice: { dataUrl: null, mimeType: null, transcript: '', audioUrl: null, recorder: null, recognition: null, stream: null, bytes: 0, captureTimeout: null },
};
const sliders = $('#sliders');
const joints = $('#joints');
const gripper = $('#gripper');
const MAX_VOICE_BYTES = 5 * 1024 * 1024;
const MAX_IMPORT_BYTES = 8 * 1024 * 1024;

function setVoiceStatus(message, isError = false) {
  $('#voice-status').textContent = message;
  $('#voice-status').style.color = isError ? '#b04a24' : '#45861a';
}

function setVoiceAudio(dataUrl, mimeType = null) {
  if (state.voice.audioUrl) URL.revokeObjectURL(state.voice.audioUrl);
  state.voice.dataUrl = dataUrl;
  state.voice.mimeType = mimeType || (dataUrl ? dataUrl.slice(5, dataUrl.indexOf(';')) : null);
  state.voice.audioUrl = dataUrl;
  $('#voice-play').disabled = !dataUrl;
}

function blobToDataUrl(blob) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(reader.result);
    reader.onerror = () => reject(reader.error);
    reader.readAsDataURL(blob);
  });
}

function stopVoiceRecognition() {
  if (state.voice.recognition) {
    state.voice.recognition.stop();
    state.voice.recognition = null;
  }
}

function beginTranscription() {
  const Recognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!Recognition) return false;
  const recognition = new Recognition();
  recognition.continuous = true;
  recognition.interimResults = true;
  recognition.lang = navigator.language || 'en-US';
  recognition.onresult = (event) => {
    let transcript = '';
    for (let index = 0; index < event.results.length; index += 1) transcript += event.results[index][0].transcript;
    state.voice.transcript = transcript.trim();
    $('#transcript').value = state.voice.transcript;
  };
  recognition.onerror = () => setVoiceStatus('Audio saved; browser transcription unavailable', true);
  try {
    recognition.start();
    state.voice.recognition = recognition;
    return true;
  } catch {
    return false;
  }
}

async function finishVoiceCapture(recorder) {
  const blob = new Blob(recorder.chunks, { type: recorder.mimeType || 'audio/webm' });
  clearTimeout(state.voice.captureTimeout);
  state.voice.captureTimeout = null;
  recorder.stream.getTracks().forEach((track) => track.stop());
  state.voice.recorder = null;
  state.voice.stream = null;
  stopVoiceRecognition();
  $('#voice-record').textContent = '● Capture voice';
  if (!blob.size) { setVoiceStatus('No audio was captured', true); return; }
  setVoiceStatus('Encoding audio for episode…');
  try {
    setVoiceAudio(await blobToDataUrl(blob), blob.type);
    setVoiceStatus('Audio captured · replayable and embedded on export');
  } catch {
    setVoiceStatus('Audio captured but could not be serialized', true);
  }
}

async function toggleVoiceCapture() {
  if (state.voice.recorder) {
    state.voice.recorder.stop();
    return;
  }
  if (!navigator.mediaDevices?.getUserMedia || !window.MediaRecorder) {
    setVoiceStatus('Audio capture is unavailable in this browser', true);
    return;
  }
  try {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    const recorder = new MediaRecorder(stream);
    recorder.stream = stream;
    recorder.chunks = [];
    recorder.ondataavailable = (event) => {
      if (!event.data.size) return;
      if (state.voice.bytes + event.data.size > MAX_VOICE_BYTES) {
        setVoiceStatus('Voice capture reached the 5 MB episode limit', true);
        recorder.stop();
        return;
      }
      state.voice.bytes += event.data.size;
      recorder.chunks.push(event.data);
    };
    recorder.onstop = () => finishVoiceCapture(recorder);
    recorder.start(250);
    state.voice.recorder = recorder;
    state.voice.stream = stream;
    state.voice.bytes = 0;
    state.voice.captureTimeout = setTimeout(() => {
      setVoiceStatus('Voice capture reached the 60 second episode limit', true);
      recorder.stop();
    }, 60_000);
    const transcribing = beginTranscription();
    setVoiceStatus(transcribing ? 'Recording and transcribing…' : 'Recording audio · transcription unavailable');
    $('#voice-record').textContent = '■ Stop voice';
  } catch {
    setVoiceStatus('Microphone permission was not granted', true);
  }
}

function replayEpisode() {
  if (!state.transitions.length || state.replaying) return;
  state.replaying = true;
  $('#replay').textContent = 'Replay in progress…';
  let index = 0;
  const run = () => {
    const transition = state.transitions[index];
    state.q = transition.observation.slice(0, 6);
    state.lastAction = transition.action_after_safety_clamp.slice(0, 6);
    state.step = transition.index;
    syncSliders(); updateArm();
    index += 1;
    if (index < state.transitions.length) setTimeout(run, 1000 / compiled.environment.control_hz);
    else { state.replaying = false; $('#replay').textContent = '↻ Replay episode'; }
  };
  if (state.voice.dataUrl) new Audio(state.voice.dataUrl).play().catch(() => {});
  run();
}

function importEpisode(file) {
  if (file.size > MAX_IMPORT_BYTES) {
    setVoiceStatus('Episode is larger than the 8 MB browser safety limit', true);
    return;
  }
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const artifact = JSON.parse(reader.result);
      if (!Array.isArray(artifact.transitions) || !artifact.transitions.every((entry) => Array.isArray(entry.observation) && entry.observation.length >= 6)) throw new Error('invalid episode');
      state.transitions = artifact.transitions;
      state.currentWorkflow = compiled.workflows.find((workflow) => workflow.id === artifact.task?.id) || state.currentWorkflow;
      if (artifact.task?.goal) setGoal(...artifact.task.goal);
      state.voice.transcript = artifact.voice?.transcript || '';
      $('#transcript').value = state.voice.transcript;
      setVoiceAudio(artifact.voice?.audio_data_url || null, artifact.voice?.mime_type);
      $('#recording-count').textContent = state.transitions.length;
      $('#download').disabled = false;
      $('#replay').disabled = !state.transitions.length;
      setVoiceStatus(state.voice.dataUrl ? 'Episode imported · audio ready to replay' : 'Episode imported · no audio attached');
    } catch {
      setVoiceStatus('Could not import that episode file', true);
    }
  };
  reader.readAsText(file);
}

function updateArm() {
  const { points, angle } = forwardKinematics();
  const line = points.map(([px, py], index) => `${index ? 'L' : 'M'}${px} ${py}`).join(' ');
  $('#arm-link').setAttribute('d', line);
  $('#arm-shadow').setAttribute('d', line);
  joints.replaceChildren(...points.map(([x, y], i) => {
    const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
    circle.setAttribute('cx', x); circle.setAttribute('cy', y); circle.setAttribute('r', i === 0 ? 14 : i === points.length - 1 ? 9 : 11);
    circle.setAttribute('class', i === points.length - 1 ? 'joint small' : 'joint');
    return circle;
  }));
  const [x, y] = points.at(-1);
  const rad = 0.35;
  gripper.innerHTML = `<path class="grip" d="M${x} ${y} l${Math.cos(angle + rad) * 19} ${Math.sin(angle + rad) * 19} M${x} ${y} l${Math.cos(angle - rad) * 19} ${Math.sin(angle - rad) * 19}"/>`;
  updateTelemetry(points.at(-1));
}

function updateTelemetry(ee) {
  const goalDistance = distance(ee, state.goal);
  const reward = -goalDistance / 100 - 0.001 * state.q.reduce((sum, q) => sum + q * q, 0);
  $('#distance').textContent = `${fmt(goalDistance / 10, 1)} cm`;
  $('#reward').textContent = fmt(reward, 3);
  $('#step').textContent = `${state.step} / ${compiled.environment.max_steps}`;
  $('#reward-bar').style.width = `${clamp(100 - goalDistance / 2.7, 0, 100)}%`;
  $('#reward-bar-value').textContent = `${Math.round(clamp(100 - goalDistance / 2.7, 0, 100))}%`;
  $('#safety-state').textContent = state.q.some((v) => Math.abs(v) >= ARM.jointLimit) ? 'At a joint limit' : 'Within limits';
  $('#safety-state').style.color = state.q.some((v) => Math.abs(v) >= ARM.jointLimit) ? '#b04a24' : '#45861a';
}

function syncSliders() {
  [...sliders.querySelectorAll('input')].forEach((input, index) => { input.value = state.q[index]; input.nextElementSibling.value = fmt(state.q[index]); });
}

function setGoal(x, y) {
  state.goal = projectToReachableWorkspace([x, y], compiled.environment.safety, ARM);
  state.guidancePlan = solveInverseKinematics(state.goal, ARM);
  $('#goal-ring').setAttribute('cx', state.goal[0]); $('#goal-ring').setAttribute('cy', state.goal[1]);
  $('#goal-cross').setAttribute('d', `M${state.goal[0] - 20} ${state.goal[1]}h40M${state.goal[0]} ${state.goal[1] - 20}v40`);
  $('#object').setAttribute('transform', `translate(${state.goal[0]} ${state.goal[1]})`);
  updateArm();
}

function addTransition() {
  if (!state.recording) return;
  if (state.transitions.length >= MAX_EPISODE_TRANSITIONS) {
    state.recording = false;
    $('#record').textContent = '● Record';
    return;
  }
  const { points } = forwardKinematics();
  const [x, y] = points.at(-1);
  state.transitions.push({
    index: state.step, timestamp_ms: Math.round(performance.now() - state.startedAt),
    observation: [...state.q, ...Array(6).fill(0), x, y, 0, 1, 0, 0, 0, state.goal[0], state.goal[1], 0],
    action: [...state.lastAction, 0], action_after_safety_clamp: [...state.lastAction, 0],
    task: state.currentWorkflow.id, reward: -distance([x, y], state.goal) / 100,
  });
  $('#recording-count').textContent = state.transitions.length;
  $('#download').disabled = false;
  $('#replay').disabled = false;
  if (state.transitions.length === MAX_EPISODE_TRANSITIONS) {
    state.recording = false;
    $('#record').textContent = '● Record';
  }
}

function renderWorkflows() {
  $('#workflow-cards').innerHTML = compiled.workflows.map((workflow) => `<article class="workflow-card"><span class="card-number">${workflow.number}</span><h3>${workflow.name}</h3><p>${workflow.instruction}</p><button data-workflow="${workflow.id}">Load in lab →</button></article>`).join('');
  $('#scenario').innerHTML = compiled.workflows.map((workflow) => `<option value="${workflow.id}">${workflow.number} — ${workflow.name}</option>`).join('');
}
function renderModels() {
  $('#model-cards').innerHTML = compiled.models.map((model) => `<article class="model-card ${model.name === 'SmolVLA' ? 'recommended' : ''}"><span class="badge">${model.kind}</span><h3>${model.name}</h3><p>${model.use}</p><ul>${model.notes.map((note) => `<li>— ${note}</li>`).join('')}</ul><a href="${model.url}" target="_blank" rel="noreferrer">Primary source ↗</a></article>`).join('');
}
function renderDatasets() {
  $('#dataset-table').innerHTML = compiled.datasets.map((dataset) => `<article class="dataset-row"><div><h3>${dataset.name}</h3><p>${dataset.purpose}</p></div><span>${dataset.type}<br>${dataset.license}</span><a href="${dataset.url}" target="_blank" rel="noreferrer">View source ↗</a></article>`).join('');
}
function loadWorkflow(id) {
  state.currentWorkflow = compiled.workflows.find((workflow) => workflow.id === id) || compiled.workflows[0];
  $('#scenario').value = state.currentWorkflow.id;
  setGoal(...state.currentWorkflow.goal);
  document.querySelector('#demo').scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function demoPolicy() {
  if (state.running) return;
  state.running = true; $('#run-policy').textContent = 'Policy running…';
  let frames = 0;
  const run = () => {
    const guided = guidedStep(state.q, state.goal, ARM, state.guidancePlan);
    state.q = guided.q; state.lastAction = guided.action;
    state.step += 1; syncSliders(); updateArm(); addTransition(); frames += 1;
    if (frames < compiled.environment.max_steps && guided.distance > 8 && state.step < compiled.environment.max_steps) requestAnimationFrame(run);
    else { state.running = false; $('#run-policy').textContent = 'Run demo policy'; }
  };
  requestAnimationFrame(run);
}

function installListeners() {
  $('#scene').addEventListener('pointerdown', (event) => {
    const rect = event.currentTarget.getBoundingClientRect(); const x = (event.clientX - rect.left) / rect.width * 760; const y = (event.clientY - rect.top) / rect.height * 490; setGoal(x, y);
  });
  $('#scenario').addEventListener('change', (event) => loadWorkflow(event.target.value));
  $('#workflow-cards').addEventListener('click', (event) => { const id = event.target.dataset.workflow; if (id) loadWorkflow(id); });
  $('#run-policy').addEventListener('click', demoPolicy);
  $('#reset').addEventListener('click', () => { state.q = [-0.45, 0.2, 0.3, -0.2, -0.1, 0.15]; state.lastAction = Array(6).fill(0); state.step = 0; state.running = false; syncSliders(); updateArm(); });
  $('#record').addEventListener('click', () => {
    if (state.recording) { state.recording = false; $('#record').textContent = '● Record'; return; }
    state.transitions = []; state.step = 0; state.recording = true;
    $('#recording-count').textContent = '0'; $('#download').disabled = true; $('#replay').disabled = true;
    $('#record').textContent = '■ Stop recording';
  });
  $('#voice-record').addEventListener('click', toggleVoiceCapture);
  $('#voice-play').addEventListener('click', () => { if (state.voice.dataUrl) new Audio(state.voice.dataUrl).play().catch(() => setVoiceStatus('Audio playback was blocked by the browser', true)); });
  $('#transcript').addEventListener('input', (event) => { state.voice.transcript = event.target.value; });
  $('#replay').addEventListener('click', replayEpisode);
  $('#import').addEventListener('click', () => $('#import-file').click());
  $('#import-file').addEventListener('change', (event) => { if (event.target.files[0]) importEpisode(event.target.files[0]); event.target.value = ''; });
  $('#download').addEventListener('click', () => {
    const artifact = buildEpisodeArtifact({ environment: compiled.environment.id, task: state.currentWorkflow, transitions: state.transitions, voice: state.voice });
    const url = URL.createObjectURL(new Blob([JSON.stringify(artifact, null, 2)], {type:'application/json'})); const link = document.createElement('a'); link.href = url; link.download = `armlab-${state.currentWorkflow.id}-${Date.now()}.json`; link.click(); URL.revokeObjectURL(url);
  });
  document.querySelectorAll('.tab').forEach((tab) => tab.addEventListener('click', () => { document.querySelectorAll('.tab,.tab-content').forEach((el) => el.classList.remove('active')); tab.classList.add('active'); $(`#${tab.dataset.tab}`).classList.add('active'); }));
}

function makeSliders() {
  sliders.innerHTML = state.q.map((q, i) => `<label class="slider">J${i + 1}<input type="range" min="-${ARM.jointLimit}" max="${ARM.jointLimit}" step="0.01" value="${q}" aria-label="Joint ${i + 1} target"/><output>${fmt(q)}</output></label>`).join('');
  [...sliders.querySelectorAll('input')].forEach((input, index) => input.addEventListener('input', () => { const previous = state.q[index]; state.q[index] = Number(input.value); state.lastAction = Array(6).fill(0); state.lastAction[index] = clamp(state.q[index] - previous, -ARM.maxActionDelta, ARM.maxActionDelta); state.step += 1; input.nextElementSibling.value = fmt(state.q[index]); updateArm(); addTransition(); }));
}

renderWorkflows(); renderModels(); renderDatasets(); makeSliders(); installListeners(); setGoal(...state.goal);
setInterval(() => { $('#sim-time').textContent = `T + ${fmt((performance.now() - state.startedAt) / 1000, 1).padStart(4, '0')} s`; }, 100);
