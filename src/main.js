import './styles.css';
import compiled from '../data/compiled.json';
import registry from '../data/sources.json';
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
  familyFilter: 'all',
  modelFilter: 'all',
  voice: { dataUrl: null, mimeType: null, transcript: '', audioUrl: null, recorder: null, recognition: null, stream: null, bytes: 0, captureTimeout: null },
};
const sliders = $('#sliders');
const joints = $('#joints');
const gripper = $('#gripper');
const MAX_VOICE_BYTES = 5 * 1024 * 1024;
const MAX_IMPORT_BYTES = 8 * 1024 * 1024;

/** SVG elements are not HTMLElement, so `.hidden` never reaches the DOM on them. */
const setHidden = (element, hidden) => { if (hidden) element.setAttribute('hidden', ''); else element.removeAttribute('hidden'); };

/** The three.js stage is optional: it is only fetched when an operator asks for it. */
const viewport = { mode: '2d', instance: null, pending: null, failed: false };

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
  const { points, angle } = forwardKinematics(state.q);
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
  pushViewportState();
  updateTelemetry(points.at(-1));
}

function pushViewportState() {
  viewport.instance?.update({ q: [...state.q], goal: [...state.goal] });
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
  const { points } = forwardKinematics(state.q);
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

/* ---------- viewports ---------- */

async function mountViewport3D() {
  if (viewport.instance) return viewport.instance;
  if (viewport.pending) return viewport.pending;
  viewport.pending = (async () => {
    const { createViewport3D } = await import('./viewport3d.js');
    const instance = createViewport3D($('#stage-3d'), {
      workspace: compiled.environment.safety,
      onGoalPick: (point) => setGoal(...point),
    });
    viewport.instance = instance;
    pushViewportState();
    return instance;
  })();
  try {
    return await viewport.pending;
  } finally {
    viewport.pending = null;
  }
}

async function setViewportMode(mode) {
  if (viewport.mode === mode) return;
  const wants3D = mode === '3d';
  $('#view-2d').classList.toggle('active', !wants3D);
  $('#view-3d').classList.toggle('active', wants3D);
  $('#view-2d').setAttribute('aria-pressed', String(!wants3D));
  $('#view-3d').setAttribute('aria-pressed', String(wants3D));

  if (!wants3D) {
    viewport.mode = '2d';
    viewport.instance?.stop();
    setHidden($('#stage-3d'), true);
    setHidden($('#scene'), false);
    $('#viewport-hint').textContent = 'Click anywhere in the scene to move the goal.';
    return;
  }

  $('#viewport-hint').textContent = 'Loading the 3-D viewport…';
  try {
    const instance = await mountViewport3D();
    viewport.mode = '3d';
    setHidden($('#scene'), true);
    setHidden($('#stage-3d'), false);
    instance.start();
    pushViewportState();
    $('#viewport-hint').textContent = 'Drag to orbit · scroll to zoom · click the floor to move the goal.';
  } catch (error) {
    viewport.failed = true;
    $('#view-2d').classList.add('active');
    $('#view-3d').classList.remove('active');
    $('#view-3d').disabled = true;
    $('#viewport-hint').textContent = 'The 3-D viewport could not start in this browser. The 2-D scene is unaffected.';
    console.error('3-D viewport unavailable', error);
  }
}

/* ---------- content sections ---------- */

const familyOf = (id) => compiled.families.find((family) => family.id === id);
const difficultyDots = (level) => `<span class="difficulty" title="Difficulty ${level} of 5" aria-label="Difficulty ${level} of 5">${'●'.repeat(level)}${'○'.repeat(5 - level)}</span>`;

function renderFamilyFilters() {
  const options = [{ id: 'all', name: 'All examples' }, ...compiled.families];
  $('#family-filters').innerHTML = options.map((family) => `<button class="chip ${family.id === state.familyFilter ? 'active' : ''}" data-family="${family.id}">${family.name}</button>`).join('');
}

function visibleWorkflows() {
  return state.familyFilter === 'all' ? compiled.workflows : compiled.workflows.filter((workflow) => workflow.family === state.familyFilter);
}

function renderTaskList() {
  $('#task-list').innerHTML = visibleWorkflows().map((workflow) => `
    <button class="task-button ${workflow.id === state.currentWorkflow.id ? 'active' : ''}" data-task="${workflow.id}" role="tab" aria-selected="${workflow.id === state.currentWorkflow.id}">
      <span class="task-index">${workflow.number}</span>
      <span class="task-name"><b>${workflow.name}</b><small>${familyOf(workflow.family)?.name || ''}</small></span>
      ${difficultyDots(workflow.difficulty)}
    </button>`).join('');
}

function renderTaskDetail() {
  const task = state.currentWorkflow;
  const family = familyOf(task.family);
  $('#task-detail').innerHTML = `
    <div class="task-detail-head">
      <div><p class="eyebrow">${family?.level || 'TASK'} · ${task.metric}</p><h3>${task.number} — ${task.name}</h3></div>
      ${difficultyDots(task.difficulty)}
    </div>
    <p class="task-summary">${task.summary}</p>
    <p class="task-instruction"><span>INSTRUCTION</span>“${task.instruction}”</p>
    <dl class="task-spec">
      <dt>Success signal</dt><dd>${task.success}</dd>
      <dt>Starting sensors</dt><dd>${task.sensors}</dd>
      <dt>Curriculum</dt><dd><ol class="curriculum">${task.curriculum.map((stage) => `<li>${stage}</li>`).join('')}</ol></dd>
      <dt>Known failure modes</dt><dd><ul class="failures">${task.failure_modes.map((mode) => `<li>${mode}</li>`).join('')}</ul></dd>
      <dt>Safety boundary</dt><dd class="guardrail">${task.guardrail}</dd>
    </dl>
    <div class="task-detail-foot">
      <span class="task-meta">Suggested baseline <b>${task.baseline}</b> · horizon ${task.horizon_steps} steps · goal ${task.goal[0]}, ${task.goal[1]} px</span>
      <button class="button compact primary" data-load="${task.id}">Load in lab →</button>
    </div>`;
}

function renderModels() {
  const routes = [
    { id: 'all', name: 'All routes' },
    { id: 'starter', name: 'Learn first' },
    { id: 'open', name: 'Open VLA' },
    { id: 'frontier', name: 'Frontier' },
  ];
  $('#model-filters').innerHTML = routes.map((route) => `<button class="chip ${route.id === state.modelFilter ? 'active' : ''}" data-route="${route.id}">${route.name}</button>`).join('');
  const visible = state.modelFilter === 'all' ? compiled.models : compiled.models.filter((model) => model.route === state.modelFilter);
  $('#model-cards').innerHTML = visible.map((model) => `<article class="model-card ${model.name === 'SmolVLA' ? 'recommended' : ''}"><span class="badge">${model.kind}</span><h3>${model.name}</h3><p>${model.use}</p><ul>${model.notes.map((note) => `<li>— ${note}</li>`).join('')}</ul><div class="tags">${model.tags.map((tag) => `<span class="tag">${tag}</span>`).join('')}</div><a href="${model.url}" target="_blank" rel="noreferrer">Primary source ↗</a></article>`).join('');
}

function renderDatasets() {
  $('#dataset-table').innerHTML = compiled.datasets.map((dataset) => `<article class="dataset-row"><div><h3>${dataset.name}</h3><p>${dataset.purpose}</p></div><span>${dataset.type}<br>${dataset.license}</span><a href="${dataset.url}" target="_blank" rel="noreferrer">View source ↗</a></article>`).join('');
}

function renderStudyPath() {
  $('#study-list').innerHTML = registry.study_path.map((entry, index) => `<article class="study-card"><span class="study-stage">${String(index + 1).padStart(2, '0')} · ${entry.stage}</span><a href="${entry.url}" target="_blank" rel="noreferrer">${entry.title} ↗</a><p>${entry.blurb}</p></article>`).join('');
}

function loadWorkflow(id, { scroll = false } = {}) {
  state.currentWorkflow = compiled.workflows.find((workflow) => workflow.id === id) || compiled.workflows[0];
  $('#scenario').value = state.currentWorkflow.id;
  $('#scenario-instruction').textContent = `“${state.currentWorkflow.instruction}” — ${state.currentWorkflow.metric}`;
  renderTaskList();
  renderTaskDetail();
  setGoal(...state.currentWorkflow.goal);
  if (scroll) $('#demo').scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function renderScenarioSelect() {
  $('#scenario').innerHTML = compiled.families.map((family) => {
    const items = compiled.workflows.filter((workflow) => workflow.family === family.id);
    return `<optgroup label="${family.name}">${items.map((workflow) => `<option value="${workflow.id}">${workflow.number} — ${workflow.name}</option>`).join('')}</optgroup>`;
  }).join('');
}

function demoPolicy() {
  if (state.running) return;
  state.running = true; $('#run-policy').textContent = 'Policy running…';
  let frames = 0;
  const horizon = Math.min(state.currentWorkflow.horizon_steps || compiled.environment.max_steps, compiled.environment.max_steps);
  const run = () => {
    const guided = guidedStep(state.q, state.goal, ARM, state.guidancePlan);
    state.q = guided.q; state.lastAction = guided.action;
    state.step += 1; syncSliders(); updateArm(); addTransition(); frames += 1;
    if (frames < horizon && guided.distance > 8 && state.step < compiled.environment.max_steps) requestAnimationFrame(run);
    else { state.running = false; $('#run-policy').textContent = 'Run demo policy'; }
  };
  requestAnimationFrame(run);
}

function installListeners() {
  $('#scene').addEventListener('pointerdown', (event) => {
    const rect = event.currentTarget.getBoundingClientRect(); const x = (event.clientX - rect.left) / rect.width * 760; const y = (event.clientY - rect.top) / rect.height * 490; setGoal(x, y);
  });
  $('#view-2d').addEventListener('click', () => setViewportMode('2d'));
  $('#view-3d').addEventListener('click', () => setViewportMode('3d'));
  $('#scenario').addEventListener('change', (event) => loadWorkflow(event.target.value));
  $('#family-filters').addEventListener('click', (event) => {
    const family = event.target.dataset.family;
    if (!family) return;
    state.familyFilter = family;
    renderFamilyFilters();
    renderTaskList();
  });
  $('#task-list').addEventListener('click', (event) => {
    const id = event.target.closest('[data-task]')?.dataset.task;
    if (id) loadWorkflow(id);
  });
  $('#task-detail').addEventListener('click', (event) => {
    const id = event.target.dataset.load;
    if (id) loadWorkflow(id, { scroll: true });
  });
  $('#model-filters').addEventListener('click', (event) => {
    const route = event.target.dataset.route;
    if (!route) return;
    state.modelFilter = route;
    renderModels();
  });
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

renderScenarioSelect(); renderFamilyFilters(); renderModels(); renderDatasets(); renderStudyPath(); makeSliders(); installListeners();
loadWorkflow(compiled.workflows[0].id);
setInterval(() => { $('#sim-time').textContent = `T + ${fmt((performance.now() - state.startedAt) / 1000, 1).padStart(4, '0')} s`; }, 100);
