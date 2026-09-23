import './styles.css';
import compiled from '../data/compiled.json';
import registry from '../data/sources.json';
import { ARM, ARM_B, buildEpisodeArtifact, clamp, computePolicyProgress, distance, evaluateCellSafety, forwardKinematics, GOAL_Z, HALT, haltState, HOME_POSE, liveDragStep, MAX_EPISODE_TRANSITIONS, nearestArm, planSafeCellMotion, planTowelFoldMotion, projectToReachableWorkspace, solveInverseKinematics } from './core.js';
import { ClothSimulator } from './cloth.js';

const $ = (selector) => document.querySelector(selector);
const fmt = (value, digits = 2) => Number(value).toFixed(digits);

const JOINT_LABELS = ['Yaw', 'Pitch', 'Pitch', 'Yaw', 'Pitch', 'Roll'];
const jointLimitOf = (index) => (index === 0 ? ARM.yawLimit : ARM.jointLimit);
const ARMS = [ARM, ARM_B];

const makeArmState = (arm) => ({
  arm,
  q: [...HOME_POSE],
  lastAction: Array(6).fill(0),
  goal: [...compiled.workflows[0].goal, compiled.workflows[0].goal_height],
  plan: null,
});

const state = {
  arms: ARMS.map(makeArmState),
  armCount: 1,
  activeArm: 0,
  recording: false,
  transitions: [],
  step: 0,
  startedAt: performance.now(),
  currentWorkflow: compiled.workflows[0],
  replaying: false,
  familyFilter: 'all',
  modelFilter: 'all',
  policy: { status: HALT.IDLE, steps: 0, speed: 1, budget: compiled.workflows[0].horizon_steps, loop: false, frame: null, accumulator: 0, restart: null, path: null, pathIndex: 0 },
  safetyNotice: null,
  voice: { dataUrl: null, mimeType: null, transcript: '', audioUrl: null, recorder: null, recognition: null, stream: null, bytes: 0, captureTimeout: null },
  frameHistory: [],
  // Must match the 3D viewport's cloth grid (src/viewport3d.js makeCloth) —
  // frame history stores whichever simulator's snapshot was available, and
  // restore() does a raw Float32Array.set() into this instance, so a size
  // mismatch throws when scrubbing the timeline.
  cloth2d: new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2 }),
  timeline: { currentStep: 0, totalSteps: 0, scrubbing: false },
};

const activeArms = () => state.arms.slice(0, state.armCount);
const controlledArm = () => state.arms[state.activeArm];

const sliders = $('#sliders');
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

const SVG_NS = 'http://www.w3.org/2000/svg';
const svgNode = (name, attributes) => {
  const node = document.createElementNS(SVG_NS, name);
  for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
  return node;
};

/**
 * Build one SVG group per arm and per goal.
 *
 * The cell is either one arm or two, so the scene graph is rebuilt only when
 * the loaded task changes that count rather than on every frame.
 */
function renderSceneGraph() {
  $('#arms').replaceChildren(...activeArms().map(({ arm }) => {
    const group = svgNode('g', { class: 'arm', 'data-arm': arm.id });
    group.append(
      svgNode('path', { class: 'arm-shadow' }),
      svgNode('path', { class: 'arm-link' }),
      svgNode('g', { class: 'joints' }),
      svgNode('g', { class: 'gripper' }),
    );
    return group;
  }));
  $('#goals').replaceChildren(...activeArms().map(({ arm }) => {
    const group = svgNode('g', { class: 'goal', 'data-arm': arm.id });
    group.append(
      svgNode('circle', { class: 'goal-ring', r: 23 }),
      svgNode('path', { class: 'goal-cross', d: '' }),
      svgNode('rect', { class: 'cube-shadow', x: -12, y: -12, width: 24, height: 24, rx: 4 }),
      svgNode('rect', { class: 'cube', x: -12, y: -12, width: 24, height: 24, rx: 4 }),
      svgNode('text', { class: 'goal-z', x: 0, y: -34, 'text-anchor': 'middle' }),
    );
    return group;
  }));
  $('#arm-switch').innerHTML = activeArms().map(({ arm }, index) => `<button class="arm-chip ${index === state.activeArm ? 'active' : ''}" data-arm-index="${index}">Arm ${arm.id}</button>`).join('');
  setHidden($('#arm-switch'), state.armCount < 2);
}

function replayEpisode() {
  if (!state.transitions.length || state.replaying) return;
  state.replaying = true;
  $('#replay').textContent = 'Replay in progress…';
  let index = 0;
  const run = () => {
    const transition = state.transitions[index];
    const replayQs = activeArms().map((armState, armIndex) => {
      const block = transition.observation.slice(armIndex * 22, armIndex * 22 + 22);
      return block.length < 22 ? armState.q : block.slice(0, 6);
    });
    const exceedsJointStep = replayQs.some((q, armIndex) => q.some((value, joint) => (
      Math.abs(value - state.arms[armIndex].q[joint]) > state.arms[armIndex].arm.maxActionDelta + 1e-9
    )));
    if (exceedsJointStep) {
      state.replaying = false;
      state.safetyNotice = 'rate';
      $('#replay').textContent = '↻ Replay episode';
      updateTelemetry();
      return;
    }
    const replaySafety = evaluateCellSafety(replayQs.map((q, index) => ({ q, arm: state.arms[index].arm })), compiled.environment.safety);
    if (!replaySafety.safe) {
      state.replaying = false;
      state.safetyNotice = replaySafety.reason;
      $('#replay').textContent = '↻ Replay episode';
      updateTelemetry();
      return;
    }
    activeArms().forEach((armState, armIndex) => {
      const block = transition.observation.slice(armIndex * 22, armIndex * 22 + 22);
      if (block.length < 22) return;
      armState.q = replayQs[armIndex];
      armState.goal = block.slice(19, 22);
      armState.lastAction = transition.action_after_safety_clamp.slice(armIndex * 7, armIndex * 7 + 6);
    });
    state.step = transition.index;
    syncSliders(); updateArms(); updateTimelineSlider();
    index += 1;
    if (index < state.transitions.length) setTimeout(run, 1000 / compiled.environment.control_hz / state.policy.speed);
    else { state.replaying = false; $('#replay').textContent = '↻ Replay episode'; updateTimelineSlider(); }
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
      if (!Array.isArray(artifact.transitions) || !artifact.transitions.every((entry) => Array.isArray(entry.observation) && entry.observation.length >= 22)) throw new Error('invalid episode');
      state.transitions = artifact.transitions;
      if (artifact.task?.id) loadWorkflow(artifact.task.id);
      state.voice.transcript = artifact.voice?.transcript || '';
      $('#transcript').value = state.voice.transcript;
      setVoiceAudio(artifact.voice?.audio_data_url || null, artifact.voice?.mime_type);
      $('#recording-count').textContent = state.transitions.length;
      $('#download').disabled = false;
      $('#replay').disabled = !state.transitions.length;
      setVoiceStatus(state.voice.dataUrl ? 'Episode imported · audio ready to replay' : 'Episode imported · no audio attached');
      updateTimelineSlider();
    } catch {
      setVoiceStatus('Could not import that episode file', true);
    }
  };
  reader.readAsText(file);
}

/** The top-down scene cannot show height directly, so it is drawn as a cast shadow. */
const shadowOffset = (height) => height * 0.11;
const castShadow = ([x, y, z]) => [x + shadowOffset(z) * 0.4, y + shadowOffset(z)];

function updateArms() {
  const armGroups = [...$('#arms').children];
  activeArms().forEach((armState, index) => {
    const group = armGroups[index];
    if (!group) return;
    const { points, forward } = forwardKinematics(armState.q, armState.arm);
    const path = (project) => points.map((point, pointIndex) => {
      const [px, py] = project(point);
      return `${pointIndex ? 'L' : 'M'}${px} ${py}`;
    }).join(' ');
    group.querySelector('.arm-link').setAttribute('d', path((point) => point));
    group.querySelector('.arm-shadow').setAttribute('d', path(castShadow));
    group.querySelector('.joints').replaceChildren(...points.map(([x, y, z], pointIndex) => svgNode('circle', {
      cx: x, cy: y,
      // Height reads as scale in a top-down view: a raised joint is nearer the camera.
      r: (pointIndex === 0 ? 14 : pointIndex === points.length - 1 ? 9 : 11) * (1 + z / 900),
      class: pointIndex === points.length - 1 ? 'joint small' : 'joint',
    })));
    const [x, y] = points.at(-1);
    // The tool heading seen from above is the forward vector's ground projection.
    const heading = Math.atan2(forward[1], forward[0]);
    const spread = 0.35;
    group.querySelector('.gripper').innerHTML = `<path class="grip" d="M${x} ${y} l${Math.cos(heading + spread) * 19} ${Math.sin(heading + spread) * 19} M${x} ${y} l${Math.cos(heading - spread) * 19} ${Math.sin(heading - spread) * 19}"/>`;
  });
  updateGoals();
  updateCloth2D();
  pushViewportState();
  updateTelemetry();
}

function updateCloth2D() {
  const clothGroup = $('#cloth-2d');
  if (!clothGroup) return;
  const isFold = state.currentWorkflow.id === 'fold';
  setHidden(clothGroup, !isFold);
  if (!isFold) return;

  const tips = activeArms().map((armState) => {
    const { points } = forwardKinematics(armState.q, armState.arm);
    const tip = points.at(-1);
    return {
      x: (tip[0] - 325) / 100,
      y: (tip[1] - 240) / 100,
      z: (tip[2] || 0) / 100,
    };
  });

  state.cloth2d.step({
    targetA: tips[0],
    targetB: tips[1],
  });

  const { basePoints, creasePoints } = state.cloth2d.get2DPolygons([325, 240], 100);
  const pathString = (pts) => pts.map((p, i) => `${i ? 'L' : 'M'}${fmt(p[0], 1)} ${fmt(p[1], 1)}`).join(' ') + ' Z';
  const creaseString = (pts) => pts.map((p, i) => `${i ? 'L' : 'M'}${fmt(p[0], 1)} ${fmt(p[1], 1)}`).join(' ');

  clothGroup.innerHTML = `
    <path class="cloth-2d-base" d="${pathString(basePoints)}" />
    <path class="cloth-2d-crease" d="${creaseString(creasePoints)}" />
    <circle class="cloth-2d-pin" cx="${fmt(basePoints[0][0], 1)}" cy="${fmt(basePoints[0][1], 1)}" r="4" />
  `;
}

function updateTimelineSlider() {
  const slider = $('#loading-sliderbar');
  const status = $('#loading-sliderbar-status');
  const progress = $('#loading-sliderbar-progress');
  const mode = $('#loading-sliderbar-mode');
  if (!slider) return;

  const total = Math.max(
    state.policy.path?.length || 0,
    state.frameHistory.length ? state.frameHistory.length - 1 : 0,
    state.transitions.length ? state.transitions.length - 1 : 0,
    1,
  );
  const current = clamp(
    state.timeline.scrubbing ? state.timeline.currentStep : (state.policy.steps || state.step || 0),
    0,
    total,
  );

  slider.min = '0';
  slider.max = String(total);
  slider.value = String(current);

  const pct = Math.round((current / total) * 100);
  if (progress) progress.style.width = `${pct}%`;
  if (status) status.textContent = `Step ${current} / ${total} (${pct}%)`;
  if (mode) {
    if (state.policy.status === HALT.RUNNING) mode.textContent = 'RUNNING';
    else if (state.replaying) mode.textContent = 'REPLAY';
    else if (state.timeline.scrubbing) mode.textContent = 'SCRUB';
    else mode.textContent = 'READY';
  }
  updatePolicyProgressBar();
}

function updatePolicyProgressBar() {
  const wrap = $('#policy-progress-wrap');
  const fill = $('#policy-progress-fill');
  const label = $('#policy-progress-label');
  if (!fill) return;

  const total = state.policy.path?.length || policyBudget();
  const current = state.policy.steps || 0;
  const pct = computePolicyProgress(current, total);

  fill.style.width = `${pct}%`;
  if (label) label.textContent = `${pct}%`;
  if (wrap) {
    wrap.setAttribute('aria-valuenow', String(pct));
    wrap.classList.toggle('running', state.policy.status === HALT.RUNNING);
    wrap.classList.toggle('reached', state.policy.status === HALT.REACHED);
  }
}

function loadSimulationStep(stepIndex) {
  state.timeline.scrubbing = true;
  state.timeline.currentStep = stepIndex;

  if (state.policy.status === HALT.RUNNING) {
    haltPolicy(HALT.OPERATOR);
  }

  const historyEntry = state.frameHistory[stepIndex];
  if (historyEntry) {
    activeArms().forEach((armState, armIndex) => {
      if (historyEntry.arms?.[armIndex]) {
        armState.q = [...historyEntry.arms[armIndex].q];
        armState.goal = [...historyEntry.arms[armIndex].goal];
        armState.lastAction = [...historyEntry.arms[armIndex].lastAction];
      }
    });
    state.step = historyEntry.step ?? stepIndex;
    state.policy.steps = stepIndex;
    if (historyEntry.clothSnapshot) {
      viewport.instance?.restoreCloth?.(historyEntry.clothSnapshot);
      state.cloth2d?.restore?.(historyEntry.clothSnapshot);
    }
  } else if (state.policy.path?.[stepIndex]) {
    const frame = state.policy.path[stepIndex];
    activeArms().forEach((armState, index) => {
      if (frame[index]) {
        armState.q = [...frame[index]];
      }
    });
    state.step = stepIndex;
    state.policy.steps = stepIndex;
  } else if (state.transitions[stepIndex]) {
    const transition = state.transitions[stepIndex];
    activeArms().forEach((armState, armIndex) => {
      const block = transition.observation.slice(armIndex * 22, armIndex * 22 + 22);
      if (block.length >= 22) {
        armState.q = block.slice(0, 6);
        armState.goal = block.slice(19, 22);
        armState.lastAction = transition.action_after_safety_clamp.slice(armIndex * 7, armIndex * 7 + 6);
      }
    });
    state.step = transition.index;
    state.policy.steps = stepIndex;
  }

  syncSliders();
  updateArms();
  updateTimelineSlider();
}

function updateGoals() {
  const goalGroups = [...$('#goals').children];
  activeArms().forEach((armState, index) => {
    const group = goalGroups[index];
    if (!group) return;
    const [x, y, z] = armState.goal;
    const [shadowX, shadowY] = castShadow(armState.goal);
    group.querySelector('.goal-ring').setAttribute('cx', x);
    group.querySelector('.goal-ring').setAttribute('cy', y);
    group.querySelector('.goal-cross').setAttribute('d', `M${x - 20} ${y}h40M${x} ${y - 20}v40`);
    group.querySelector('.cube-shadow').setAttribute('transform', `translate(${shadowX} ${shadowY})`);
    group.querySelector('.cube').setAttribute('transform', `translate(${x} ${y}) scale(${1 + z / 600})`);
    const label = group.querySelector('.goal-z');
    label.setAttribute('transform', `translate(${x} ${y})`);
    label.textContent = `${armState.arm.id} · z ${fmt(z / 10, 1)} cm`;
  });
}

function pushViewportState() {
  viewport.instance?.update({
    arms: activeArms().map((armState) => ({
      id: armState.arm.id,
      q: [...armState.q],
      goal: [...armState.goal],
    })),
    taskId: state.currentWorkflow.id,
    policyProgress: state.policy.path?.length ? state.policy.pathIndex / state.policy.path.length : 0,
  });
}

const tipOf = (armState) => forwardKinematics(armState.q, armState.arm).points.at(-1);
const armError = (armState) => distance(tipOf(armState), armState.goal);

function updateTelemetry() {
  const errors = activeArms().map(armError);
  const worst = Math.max(...errors);
  const reward = -worst / 100 - 0.001 * activeArms().reduce((sum, armState) => sum + armState.q.reduce((inner, q) => inner + q * q, 0), 0);
  $('#distance').textContent = state.armCount > 1
    ? `${errors.map((error, index) => `${state.arms[index].arm.id} ${fmt(error / 10, 1)}`).join(' / ')} cm`
    : `${fmt(worst / 10, 1)} cm`;
  $('#tool-height').textContent = activeArms().map((armState) => `${fmt(tipOf(armState)[2] / 10, 1)}`).join(' / ') + ' cm';
  $('#reward').textContent = fmt(reward, 3);
  $('#step').textContent = `${state.step} / ${compiled.environment.max_steps}`;
  const score = clamp(100 - worst / 2.7, 0, 100);
  $('#reward-bar').style.width = `${score}%`;
  $('#reward-bar-value').textContent = `${Math.round(score)}%`;
  const atLimit = activeArms().some((armState) => armState.q.some((value, index) => Math.abs(value) >= jointLimitOf(index) - 1e-6));
  const actualSafety = evaluateCellSafety(activeArms().map(({ q, arm }) => ({ q, arm })), compiled.environment.safety);
  const reason = state.safetyNotice || actualSafety.reason;
  const safetyCopy = { floor: 'Blocked at floor', workspace: 'Blocked at floor edge', collision: 'Blocked arm collision', rate: 'Blocked joint-step jump' };
  $('#safety-state').textContent = reason ? safetyCopy[reason] : atLimit ? 'At a joint limit' : 'Within floor + collision limits';
  $('#safety-state').style.color = reason || atLimit ? '#b04a24' : '#45861a';
}

function syncSliders() {
  const armState = controlledArm();
  [...sliders.querySelectorAll('input[data-joint]')].forEach((input, index) => {
    input.value = armState.q[index];
    input.nextElementSibling.value = fmt(armState.q[index]);
  });
  const goalZ = sliders.querySelector('#goal-z');
  if (goalZ) { goalZ.value = armState.goal[2]; goalZ.nextElementSibling.value = `${fmt(armState.goal[2] / 10, 1)} cm`; }
}

function setArmGoal(armState, point, { replan = true } = {}) {
  armState.goal = projectToReachableWorkspace(point, compiled.environment.safety, armState.arm);
  if (replan) replanArms();
}

/** Select a jointly safe pair of final IK poses, trying both planning orders. */
function replanArms() {
  const arms = activeArms();
  const orders = arms.length === 2 ? [[0, 1], [1, 0]] : [[0]];
  let best = null;
  for (const order of orders) {
    const plans = [];
    let error = 0;
    for (const index of order) {
      // `plans` is deliberately sparse while an order is being evaluated.
      // Keep each solved pose paired with its original arm index: in the
      // B-then-A order, treating B's pose as A (or as B's own obstacle) made
      // the alternate bimanual plan nondeterministic.
      const otherPoses = plans.flatMap((plan, otherIndex) => (
        plan && otherIndex !== index ? [{ q: plan.q, arm: arms[otherIndex].arm }] : []
      ));
      plans[index] = solveInverseKinematics(arms[index].goal, arms[index].arm, compiled.environment.safety, otherPoses);
      error += plans[index].distance;
    }
    const safe = plans.every(Boolean) && evaluateCellSafety(plans.map((plan, index) => ({ q: plan.q, arm: arms[index].arm })), compiled.environment.safety).safe;
    if (safe && (!best || error < best.error)) best = { plans, error };
  }
  if (!best) {
    state.safetyNotice = 'collision';
    return false;
  }
  arms.forEach((armState, index) => { armState.plan = best.plans[index]; });
  state.safetyNotice = null;
  return true;
}

/**
 * Height changes coming from a drag in the 3-D viewport.
 *
 * A full IK search on every pointer move would stall the drag, so the goal
 * follows the pointer unplanned and the solver runs once on release.
 */
function setGoalHeight(armId, height, { committed = true } = {}) {
  const armState = state.arms.find((candidate) => candidate.arm.id === armId);
  if (!armState) return;
  if (height !== null) setArmGoal(armState, [armState.goal[0], armState.goal[1], height], { replan: false });
  if (committed) replanArms();
  state.activeArm = state.arms.indexOf(armState);
  [...$('#arm-switch').children].forEach((chip, index) => chip.classList.toggle('active', index === state.activeArm));
  updateArms();
  syncSliders();
}

/** Route a scene click to the arm whose base column is nearest, keeping its goal height. */
function setGoalFromScene(point) {
  const arm = nearestArm(point, activeArms().map((armState) => armState.arm));
  const armState = state.arms.find((candidate) => candidate.arm.id === arm.id);
  setArmGoal(armState, [point[0], point[1], armState.goal[2]]);
  updateArms();
  syncSliders();
}

function addTransition() {
  if (!state.recording) return;
  if (state.transitions.length >= MAX_EPISODE_TRANSITIONS) {
    stopRecording();
    return;
  }
  const observation = [];
  const action = [];
  for (const armState of activeArms()) {
    const [x, y, z] = tipOf(armState);
    observation.push(...armState.q, ...Array(6).fill(0), x, y, z, 1, 0, 0, 0, ...armState.goal);
    action.push(...armState.lastAction, 0);
  }
  state.transitions.push({
    index: state.step,
    timestamp_ms: Math.round(performance.now() - state.startedAt),
    observation,
    action: [...action],
    action_after_safety_clamp: [...action],
    task: state.currentWorkflow.id,
    arms: state.armCount,
    reward: -Math.max(...activeArms().map(armError)) / 100,
  });
  $('#recording-count').textContent = state.transitions.length;
  $('#download').disabled = false;
  $('#replay').disabled = false;
  if (state.transitions.length === MAX_EPISODE_TRANSITIONS) stopRecording();
}

function stopRecording() {
  state.recording = false;
  $('#record').textContent = '● Record';
}

/* ---------- viewports ---------- */

async function mountViewport3D() {
  if (viewport.instance) return viewport.instance;
  if (viewport.pending) return viewport.pending;
  viewport.pending = (async () => {
    const { createViewport3D } = await import('./viewport3d.js');
    const instance = createViewport3D($('#stage-3d'), {
      workspace: compiled.environment.safety,
      onGoalPick: (point) => setGoalFromScene(point),
      onGoalHeight: setGoalHeight,
      onClothFrame: ({ frames, captured, settled, foldMetrics }) => {
        if (state.currentWorkflow.id !== 'fold') return;
        const foldDist = foldMetrics ? ` · fold: ${fmt(foldMetrics.frontDistance * 10, 1)} cm` : '';
        const foldStatus = foldMetrics?.folded ? ' · folded' : '';
        $('#cloth-status').textContent = `Cloth frames ${frames} / 120 · ${captured.map((value, index) => `${index ? 'B' : 'A'} ${value ? 'held' : 'free'}`).join(' · ')}${settled ? ' · settled' : ''}${foldDist}${foldStatus}`;
      },
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
    setHidden($('#viewport-loader'), true);
    $('#viewport-hint').textContent = 'Click anywhere in the scene to move the goal.';
    return;
  }

  const loader = $('#viewport-loader');
  const loaderFill = $('#viewport-loader-fill');
  const loaderPercent = $('#viewport-loader-percent');
  const loaderStep = $('#viewport-loader-step');

  const setLoaderProgress = (percent, text) => {
    if (loader) setHidden(loader, false);
    if (loaderFill) loaderFill.style.width = `${percent}%`;
    if (loaderPercent) loaderPercent.textContent = `${percent}%`;
    if (loaderStep) loaderStep.textContent = text;
  };

  setLoaderProgress(15, 'Loading Three.js graphics engine…');
  $('#viewport-hint').textContent = 'Loading the 3-D viewport…';

  try {
    setLoaderProgress(40, 'Building spatial kinematics & workcell…');
    await new Promise((resolve) => setTimeout(resolve, 60));
    setLoaderProgress(70, 'Initializing Position-Based Dynamics cloth physics…');
    const instance = await mountViewport3D();
    setLoaderProgress(95, 'Compiling WebGL shaders & lighting…');
    await new Promise((resolve) => setTimeout(resolve, 80));
    setLoaderProgress(100, 'Ready');

    viewport.mode = '3d';
    setHidden($('#scene'), true);
    setHidden($('#stage-3d'), false);
    setTimeout(() => {
      if (loader) setHidden(loader, true);
    }, 200);

    instance.start();
    pushViewportState();
    $('#viewport-hint').textContent = 'Drag to orbit · scroll to zoom · click the floor to move a goal · drag a cube up or down to change its height.';
  } catch (error) {
    if (loader) setHidden(loader, true);
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
  haltPolicy(HALT.IDLE, { silent: true });
  state.currentWorkflow = compiled.workflows.find((workflow) => workflow.id === id) || compiled.workflows[0];
  const workflow = state.currentWorkflow;
  state.armCount = workflow.arms || 1;
  state.activeArm = 0;
  state.policy.path = null;
  state.policy.pathIndex = 0;
  state.policy.accumulator = 0;
  state.policy.budget = workflow.id === 'fold' ? 360 : workflow.horizon_steps;
  state.policy.loop = workflow.id === 'fold';
  $('#policy-loop').checked = state.policy.loop;
  $('#policy-budget').value = state.policy.budget;
  $('#policy-budget-value').textContent = state.policy.budget;
  $('#policy-profile').textContent = workflow.id === 'fold' ? 'Towel-fold specialist' : 'Geometric policy';
  setHidden($('#cloth-status'), workflow.id !== 'fold');
  $('#cloth-status').textContent = 'Cloth frames 0 / 120';
  state.cloth2d?.reset();
  viewport.instance?.resetCloth?.();
  state.frameHistory = [];
  state.timeline.currentStep = 0;
  state.timeline.scrubbing = false;
  updateTimelineSlider();
  setArmGoal(state.arms[0], [...workflow.goal, workflow.goal_height], { replan: false });
  setArmGoal(state.arms[1], [...(workflow.goal_b || workflow.goal), workflow.goal_height], { replan: false });
  replanArms();
  $('#scenario').value = workflow.id;
  $('#scenario-instruction').textContent = `“${workflow.instruction}” — ${workflow.metric}`;
  $('#arm-count').textContent = state.armCount > 1 ? 'Bimanual · 2 arms' : 'Single arm';
  renderSceneGraph();
  renderTaskList();
  renderTaskDetail();
  updateArms();
  syncSliders();
  if (scroll) $('#demo').scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function renderScenarioSelect() {
  $('#scenario').innerHTML = compiled.families.map((family) => {
    const items = compiled.workflows.filter((workflow) => workflow.family === family.id);
    return `<optgroup label="${family.name}">${items.map((workflow) => `<option value="${workflow.id}">${workflow.number} — ${workflow.name}</option>`).join('')}</optgroup>`;
  }).join('');
}

const HALT_COPY = {
  [HALT.IDLE]: () => 'Idle · policy not running',
  [HALT.RUNNING]: () => `Running · step ${state.policy.steps}`,
  [HALT.REACHED]: () => `Halted · goal reached in ${state.policy.steps} steps`,
  [HALT.BUDGET]: () => `Halted · step budget exhausted at ${state.policy.steps}`,
  [HALT.OPERATOR]: () => `Halted by operator at step ${state.policy.steps}`,
  [HALT.SAFETY]: () => `Halted · safety boundary at step ${state.policy.steps}`,
};

function renderHaltState() {
  const element = $('#halt-state');
  element.textContent = HALT_COPY[state.policy.status]();
  element.className = `halt-state ${state.policy.status}`;
  $('#run-policy').textContent = state.policy.status === HALT.RUNNING ? '■ Halt policy' : 'Run demo policy';
  updatePolicyProgressBar();
}

/** Advance one prevalidated floor- and collision-safe control frame. */
function policyStep() {
  const next = state.policy.path?.[state.policy.pathIndex];
  if (!next) return HALT.SAFETY;
  // Validate a live frame before state changes. This defense-in-depth check
  // prevents a malformed path from teleporting an arm between safe poses.
  const exceedsJointStep = next.some((q, index) => q.some((value, joint) => (
    Math.abs(value - state.arms[index].q[joint]) > state.arms[index].arm.maxActionDelta + 1e-9
  )));
  if (exceedsJointStep) { state.safetyNotice = 'rate'; return HALT.SAFETY; }
  const nextSafety = evaluateCellSafety(next.map((q, index) => ({ q, arm: state.arms[index].arm })), compiled.environment.safety);
  if (!nextSafety.safe) { state.safetyNotice = nextSafety.reason; return HALT.SAFETY; }
  let error = 0;
  activeArms().forEach((armState, index) => {
    const previous = armState.q;
    armState.q = [...next[index]];
    armState.lastAction = armState.q.map((value, joint) => value - previous[joint]);
    error = Math.max(error, armError(armState));
  });
  state.policy.pathIndex += 1;
  state.step += 1;
  state.policy.steps += 1;
  syncSliders();
  updateArms();
  addTransition();

  const clothSnap = viewport.instance?.getClothSnapshot?.() || state.cloth2d?.snapshot();
  state.frameHistory[state.policy.steps] = {
    step: state.step,
    arms: activeArms().map((a) => ({ q: [...a.q], goal: [...a.goal], lastAction: [...a.lastAction] })),
    clothSnapshot: clothSnap,
  };
  updateTimelineSlider();

  return haltState({ error, steps: state.policy.steps, budget: policyBudget() });
}

const policyBudget = () => clamp(Math.round(state.policy.budget), 1, 400);

/**
 * Advance the run by `speed` control steps per animation frame.
 *
 * A fractional speed spreads one step across several frames, so the same
 * accumulator serves slow-motion inspection and fast-forwarding a long reach.
 */
function policyFrame() {
  state.policy.accumulator += state.policy.speed;
  let status = HALT.RUNNING;
  while (state.policy.accumulator >= 1 && status === HALT.RUNNING) {
    state.policy.accumulator -= 1;
    status = policyStep();
  }
  renderHaltState();
  if (status === HALT.RUNNING) {
    state.policy.frame = requestAnimationFrame(policyFrame);
    return;
  }
  settlePolicy(status);
}

function settlePolicy(status) {
  state.policy.frame = null;
  state.policy.status = status;
  renderHaltState();
  // Only a budget halt may retry. A safety halt is always terminal.
  if (status === HALT.BUDGET && state.policy.loop) {
    state.policy.restart = setTimeout(() => { state.policy.restart = null; resetArms(); startPolicy(); }, 700);
  }
}

function startPolicy() {
  clearTimeout(state.policy.restart);
  state.policy.restart = null;
  if (!replanArms()) { state.policy.status = HALT.SAFETY; renderHaltState(); return; }

  let motion = null;
  if (state.currentWorkflow.id === 'fold' && activeArms().length === 2) {
    motion = planTowelFoldMotion(
      activeArms().map(({ q, arm }) => ({ q, arm })),
      compiled.environment.safety,
    );
  }
  if (!motion) {
    motion = planSafeCellMotion(
      activeArms().map(({ q, arm }) => ({ q, arm })),
      activeArms().map(({ plan }) => plan.q),
      compiled.environment.safety,
    );
  }
  if (!motion) { state.safetyNotice = 'workspace'; state.policy.status = HALT.SAFETY; renderHaltState(); updateTelemetry(); return; }
  state.policy.path = motion.frames;
  state.policy.pathIndex = 0;
  state.policy.status = HALT.RUNNING;
  state.policy.steps = 0;
  state.policy.accumulator = 0;
  state.timeline.scrubbing = false;

  const clothSnap = viewport.instance?.getClothSnapshot?.() || state.cloth2d?.snapshot();
  state.frameHistory = [{
    step: state.step,
    arms: activeArms().map((a) => ({ q: [...a.q], goal: [...a.goal], lastAction: [...a.lastAction] })),
    clothSnapshot: clothSnap,
  }];

  renderHaltState();
  updateTimelineSlider();
  state.policy.frame = requestAnimationFrame(policyFrame);
}

function haltPolicy(status = HALT.OPERATOR, { silent = false } = {}) {
  if (state.policy.frame !== null) cancelAnimationFrame(state.policy.frame);
  clearTimeout(state.policy.restart);
  state.policy.frame = null;
  state.policy.restart = null;
  if (silent) { state.policy.status = HALT.IDLE; state.policy.steps = 0; }
  else state.policy.status = status;
  renderHaltState();
}

function resetArms() {
  for (const armState of state.arms) {
    armState.q = [...HOME_POSE];
    armState.lastAction = Array(6).fill(0);
  }
  state.step = 0;
  state.safetyNotice = null;
  state.policy.path = null;
  state.policy.pathIndex = 0;
  state.policy.accumulator = 0;
  state.policy.steps = 0;
  state.frameHistory = [];
  state.timeline.currentStep = 0;
  state.timeline.scrubbing = false;
  state.cloth2d?.reset();
  viewport.instance?.resetCloth?.();
  // The sim-time readout and any pending voice auto-stop are timers, not
  // policy/episode state, and neither was touched here — the clock kept
  // counting from page load and a running capture kept its own countdown
  // across a reset.
  state.startedAt = performance.now();
  if (state.voice.captureTimeout) {
    clearTimeout(state.voice.captureTimeout);
    state.voice.captureTimeout = null;
  }
  syncSliders();
  updateArms();
  updateTimelineSlider();
}

/**
 * Drag-to-record the arm in the 2-D scene.
 *
 * A plain click keeps the original behavior (set a full goal, let
 * replanArms find a plan). Once the pointer actually moves past a small
 * threshold it becomes a live drag instead: each frame takes one
 * liveDragStep toward the pointer — rate-capped and rejected outright by
 * the same floor/workspace/inter-arm envelope as everything else — and,
 * while `state.recording` is on, records a transition per frame, exactly
 * like nudging a joint slider but by dragging the tool itself.
 */
let sceneDrag = null;

function scenePointFromEvent(event) {
  const rect = event.currentTarget.getBoundingClientRect();
  return [(event.clientX - rect.left) / rect.width * 760, (event.clientY - rect.top) / rect.height * 490];
}

function onSceneDragMove(event) {
  if (!sceneDrag) return;
  if (!sceneDrag.dragging) {
    if (Math.hypot(event.clientX - sceneDrag.pressedAt.x, event.clientY - sceneDrag.pressedAt.y) < 6) return;
    sceneDrag.dragging = true;
  }
  const [x, y] = scenePointFromEvent(event);
  const { armState } = sceneDrag;
  const otherPoses = activeArms().filter((candidate) => candidate !== armState).map(({ q, arm }) => ({ q, arm }));
  const previous = armState.q;
  const step = liveDragStep(previous, [x, y, armState.goal[2]], armState.arm, compiled.environment.safety, otherPoses);
  state.safetyNotice = step.safety.safe ? null : step.safety.reason;
  if (!step.moved) { updateTelemetry(); return; }
  armState.q = step.q;
  armState.lastAction = step.q.map((value, index) => value - previous[index]);
  armState.goal = projectToReachableWorkspace([x, y, armState.goal[2]], compiled.environment.safety, armState.arm);
  state.step += 1;
  syncSliders();
  updateArms();
  addTransition();
}

function endSceneDrag(event) {
  if (!sceneDrag) return;
  const { dragging } = sceneDrag;
  sceneDrag = null;
  if (event.currentTarget.hasPointerCapture?.(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
  if (dragging) { replanArms(); syncSliders(); return; }
  // No movement past the threshold: treat it as the original click-to-set-goal.
  setGoalFromScene(scenePointFromEvent(event));
}

function installListeners() {
  $('#scene').addEventListener('pointerdown', (event) => {
    const point = scenePointFromEvent(event);
    const arm = nearestArm(point, activeArms().map((armState) => armState.arm));
    const armState = state.arms.find((candidate) => candidate.arm.id === arm.id);
    sceneDrag = { armState, pressedAt: { x: event.clientX, y: event.clientY }, dragging: false };
    event.currentTarget.setPointerCapture(event.pointerId);
  });
  $('#scene').addEventListener('pointermove', onSceneDragMove);
  $('#scene').addEventListener('pointerup', endSceneDrag);
  $('#scene').addEventListener('pointercancel', endSceneDrag);
  $('#view-2d').addEventListener('click', () => setViewportMode('2d'));
  $('#view-3d').addEventListener('click', () => setViewportMode('3d'));
  $('#scenario').addEventListener('change', (event) => loadWorkflow(event.target.value));
  $('#arm-switch').addEventListener('click', (event) => {
    const index = event.target.dataset.armIndex;
    if (index === undefined) return;
    state.activeArm = Number(index);
    [...$('#arm-switch').children].forEach((chip, chipIndex) => chip.classList.toggle('active', chipIndex === state.activeArm));
    syncSliders();
  });
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
  $('#run-policy').addEventListener('click', () => {
    if (state.policy.status === HALT.RUNNING) haltPolicy(HALT.OPERATOR);
    else startPolicy();
  });
  $('#policy-speed').addEventListener('input', (event) => {
    state.policy.speed = Number(event.target.value);
    $('#policy-speed-value').textContent = `${fmt(state.policy.speed)}×`;
  });
  $('#policy-budget').addEventListener('input', (event) => {
    state.policy.budget = Number(event.target.value);
    $('#policy-budget-value').textContent = state.policy.budget;
  });
  $('#policy-loop').addEventListener('change', (event) => {
    state.policy.loop = event.target.checked;
    if (!state.policy.loop) { clearTimeout(state.policy.restart); state.policy.restart = null; }
    else if (state.policy.status === HALT.BUDGET) settlePolicy(HALT.BUDGET);
  });
  const timelineSlider = $('#loading-sliderbar');
  if (timelineSlider) {
    timelineSlider.addEventListener('input', (event) => {
      loadSimulationStep(Number(event.target.value));
    });
    timelineSlider.addEventListener('change', (event) => {
      loadSimulationStep(Number(event.target.value));
      state.timeline.scrubbing = false;
      updateTimelineSlider();
    });
  }
  $('#reset').addEventListener('click', () => { haltPolicy(HALT.IDLE, { silent: true }); resetArms(); });
  $('#record').addEventListener('click', () => {
    if (state.recording) { stopRecording(); return; }
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
    const artifact = buildEpisodeArtifact({
      environment: compiled.environment.id,
      task: state.currentWorkflow,
      transitions: state.transitions,
      voice: state.voice,
      arms: state.armCount,
      halt: state.policy.status,
    });
    const url = URL.createObjectURL(new Blob([JSON.stringify(artifact, null, 2)], { type: 'application/json' }));
    const link = document.createElement('a');
    link.href = url; link.download = `armlab-${state.currentWorkflow.id}-${Date.now()}.json`; link.click();
    URL.revokeObjectURL(url);
  });
  document.querySelectorAll('.tab').forEach((tab) => tab.addEventListener('click', () => { document.querySelectorAll('.tab,.tab-content').forEach((el) => el.classList.remove('active')); tab.classList.add('active'); $(`#${tab.dataset.tab}`).classList.add('active'); }));
}

function makeSliders() {
  const jointRows = HOME_POSE.map((value, index) => `<label class="slider">${JOINT_LABELS[index]}<input data-joint="${index}" type="range" min="${-jointLimitOf(index)}" max="${jointLimitOf(index)}" step="0.01" value="${value}" aria-label="${index === 0 ? 'Base yaw' : `Joint ${index + 1}`} target"/><output>${fmt(value)}</output></label>`).join('');
  sliders.innerHTML = `${jointRows}<label class="slider goal-z-slider">Goal Z<input id="goal-z" type="range" min="${GOAL_Z.min}" max="${GOAL_Z.max}" step="1" value="${GOAL_Z.rest}" aria-label="Goal height above the table"/><output>${fmt(GOAL_Z.rest / 10, 1)} cm</output></label>`;

  [...sliders.querySelectorAll('input[data-joint]')].forEach((input, index) => input.addEventListener('input', () => {
    const armState = controlledArm();
    const previous = armState.q[index];
    const candidate = [...armState.q];
    // A range input can jump from one end to the other. Apply the same
    // per-step cap as the policy, so manual operation cannot teleport either.
    candidate[index] = previous + clamp(Number(input.value) - previous, -armState.arm.maxActionDelta, armState.arm.maxActionDelta);
    const candidatePoses = activeArms().map((item) => ({ q: item === armState ? candidate : item.q, arm: item.arm }));
    const safety = evaluateCellSafety(candidatePoses, compiled.environment.safety);
    if (!safety.safe) {
      state.safetyNotice = safety.reason;
      syncSliders();
      updateTelemetry();
      return;
    }
    state.safetyNotice = null;
    armState.q = candidate;
    armState.lastAction = Array(6).fill(0);
    armState.lastAction[index] = clamp(armState.q[index] - previous, -ARM.maxActionDelta, ARM.maxActionDelta);
    state.step += 1;
    syncSliders();
    updateArms();
    addTransition();
  }));

  // Height is the axis the top-down scene cannot offer: raise or lower the goal itself.
  sliders.querySelector('#goal-z').addEventListener('input', (event) => {
    const armState = controlledArm();
    setArmGoal(armState, [armState.goal[0], armState.goal[1], Number(event.target.value)]);
    event.target.nextElementSibling.value = `${fmt(armState.goal[2] / 10, 1)} cm`;
    updateArms();
  });
}

renderScenarioSelect(); renderFamilyFilters(); renderModels(); renderDatasets(); renderStudyPath(); makeSliders(); installListeners();
loadWorkflow(compiled.workflows[0].id);
renderHaltState();
setInterval(() => { $('#sim-time').textContent = `T + ${fmt((performance.now() - state.startedAt) / 1000, 1).padStart(4, '0')} s`; }, 100);
