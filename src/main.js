import './styles.css';
import compiled from '../data/compiled.json';
import registry from '../data/sources.json';
import { planHalfFoldMotion } from './half-fold.js';
import { ARM, ARM_B, buildDatasetManifest, buildEpisodeArtifact, clamp, computePolicyProgress, distance, evaluateCellSafety, formatSolverTicker, forwardKinematics, GOAL_Z, HALT, haltState, HOME_POSE, liveDragStep, MAX_EPISODE_TRANSITIONS, nearestArm, planSafeCellMotion, planTowelFoldMotion, projectToReachableWorkspace, reduceSafeCellMotion, solveInverseKinematics } from './core.js';
import { ClothSimulator, clothCorners } from './cloth.js';
import { loadClothSettings, onClothSettingsChange } from './cloth-settings.js';
import { FOLD_GUIDE, FOLD_STAGES, clothPointToScene, foldGuideStage } from './fold-guide.js';
import { bootstrapPolicy, policyRecipeFor, scoreTaskStages } from './task-policies.js';
import { FINGER, fenceWalls, RigidScene, sceneYaw, snapshotObject, toolBasis } from './rigid.js';
import { goalCenter, isLiveRigidTask, isRigidTask, planRigidTask, rigidOutcome } from './rigid-tasks.js';
import { StackController } from './stack-controller.js';

const $ = (selector) => document.querySelector(selector);
const fmt = (value, digits = 2) => Number(value).toFixed(digits);

const JOINT_LABELS = ['Yaw', 'Pitch', 'Pitch', 'Yaw', 'Pitch', 'Roll'];
const jointLimitOf = (index) => (index === 0 ? ARM.yawLimit : ARM.jointLimit);
const ARMS = [ARM, ARM_B];
// Ceiling for the Step budget slider and the fold task's default budget.
// Raised from the old flat 400 because the fold plan's own settle tail (see
// core.js's planTowelFoldMotion) needs ~477 steps to finish, then to 600 for
// Task 12's half fold (half-fold.js), which runs to ~550; keep this in sync
// with index.html's #policy-budget max attribute.
const FOLD_STEP_BUDGET = 600;

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
  dataset: [],
  step: 0,
  startedAt: performance.now(),
  currentWorkflow: compiled.workflows[0],
  replaying: false,
  familyFilter: 'all',
  modelFilter: 'all',
  policy: { status: HALT.IDLE, steps: 0, speed: 1, budget: compiled.workflows[0].horizon_steps, loop: false, frame: null, accumulator: 0, restart: null, path: null, pathIndex: 0, solveStartedAt: null, planning: false, planningToken: 0, planningPhase: null, stageScore: null },
  safetyNotice: null,
  voice: { dataUrl: null, mimeType: null, transcript: '', audioUrl: null, recorder: null, recognition: null, stream: null, bytes: 0, captureTimeout: null },
  frameHistory: [],
  // Must match the 3D viewport's cloth grid (src/viewport3d.js makeCloth) —
  // frame history stores whichever simulator's snapshot was available, and
  // restore() does a raw Float32Array.set() into this instance, so a size
  // mismatch throws when scrubbing the timeline.
  // Physics values come from the cloth settings page (cloth.html).
  cloth2d: new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2, ...loadClothSettings() }),
  timeline: { currentStep: 0, totalSteps: 0, scrubbing: false },
  // Task 12 (Configurable fold): which edge the half fold lays over the
  // other. Either way both arms carry a corner, so four corners become two.
  foldDirection: 'back-to-front',
  // Tasks 13-15 (Rigid objects): the physics scene and the spec it was built
  // from - a copy of the task's `rigid` block, so a scene click can move the
  // place target without editing the task itself.
  rigid: null,
  rigidSpec: null,
  // Task 16 (Stack under fire): the closed-loop stacker, the physics clock
  // that keeps the table live while the arm is idle, and shots fired.
  stack: null,
  live: { frame: null, last: 0, accumulator: 0, shots: 0 },
};

const activeArms = () => state.arms.slice(0, state.armCount);
/** Tasks whose plan runs to its own end rather than stopping at a goal distance. */
const runsFullPlan = (workflow) => isClothFoldTask(workflow) || isRigidTask(workflow);

/** Rebuild the rigid-object scene from `state.rigidSpec` (or clear it for other tasks). */
function buildRigidScene() {
  const live = isLiveRigidTask(state.currentWorkflow);
  state.rigid = state.rigidSpec ? new RigidScene(state.rigidSpec, { arms: state.armCount, armColliders: live }) : null;
  state.stack = live ? new StackController({ rigid: state.rigidSpec, arm: ARM, safety: compiled.environment.safety }) : null;
  state.live.shots = 0;
  if (live) { startLivePhysics(); updateLiveStatus(); } else stopLivePhysics();
}

/**
 * Task 16's table never stops: balls fly and cubes tumble whether or not
 * the arm is running. While the policy runs, its own frames step the
 * physics; otherwise this clock does, at the 25 Hz control rate, with the
 * arm held where it is.
 */
function startLivePhysics() {
  if (state.live.frame !== null) return;
  state.live.last = performance.now();
  state.live.accumulator = 0;
  const tick = (now) => {
    state.live.frame = requestAnimationFrame(tick);
    // A backgrounded tab resumes without a burst of catch-up steps.
    state.live.accumulator = Math.min(state.live.accumulator + (now - state.live.last), 200);
    state.live.last = now;
    if (state.policy.status === HALT.RUNNING || state.policy.planning) { state.live.accumulator = 0; return; }
    let stepped = false;
    while (state.live.accumulator >= 40) {
      state.live.accumulator -= 40;
      advanceRigidPhysics();
      stepped = true;
    }
    if (stepped) { updateArms(); updateLiveStatus(); }
  };
  state.live.frame = requestAnimationFrame(tick);
}

function stopLivePhysics() {
  if (state.live.frame !== null) cancelAnimationFrame(state.live.frame);
  state.live.frame = null;
}

/**
 * Fire a ball at scene point `target` ([x, y, z] px). Without a `from`, it
 * comes from the far edge of the table, beyond the workspace, high enough
 * to drop in over the pen wall, on a lob that lands on the target: time of flight from a fixed horizontal speed,
 * then the vertical speed that drops it onto the target under gravity.
 */
function shootAt(target, from = [clamp(target[0], 120, 600), 40, 130]) {
  if (!state.rigid) return;
  const g = 9810; // mm/s^2 - scene px are mm
  const horizontal = Math.hypot(target[0] - from[0], target[1] - from[1]);
  const t = Math.max(horizontal / 3200, 0.06);
  const velocity = [
    (target[0] - from[0]) / t,
    (target[1] - from[1]) / t,
    (target[2] - from[2]) / t + 0.5 * g * t,
  ];
  state.rigid.spawnProjectile(from, velocity);
  state.live.shots += 1;
  updateArms();
  updateLiveStatus();
}

function updateLiveStatus() {
  const element = $('#live-status');
  if (!state.stack) return;
  const s = state.stack.status();
  element.textContent = `Tower ${s.tower}/${s.goal} · best ${s.best} · placed ${s.placed} · recoveries ${s.recoveries} · drops ${s.drops} · misses ${s.misses} · shots ${state.live.shots} · returned ${state.rigid.returned} — ${state.policy.status === HALT.RUNNING ? s.phase : 'arm idle: Run demo policy to start stacking'}`;
}

/**
 * Step the rigid scene one control frame with the arms where they now are.
 * Like the cloth, it advances only with an accepted frame - a policy step or
 * a manual move - so both viewports and the timeline show one physical state.
 * `grips` is the plan's gripper command; a manual move passes none and the
 * jaws keep whatever state they are in, still pushing whatever they touch.
 */
function advanceRigidPhysics(grips) {
  state.rigid?.step({ arms: activeArms().map(({ q, arm }) => ({ q, arm })), grips });
}

/**
 * Move a rigid task's place target - the zone, the tray, or the base cube of
 * a stack - to a clicked table point, and start the scene over around it.
 */
function moveRigidGoal([x, y]) {
  const [minX, maxX, minY, maxY] = compiled.environment.safety.goal_workspace;
  const point = [clamp(x, minX + 30, maxX - 30), clamp(y, minY + 30, maxY - 30)];
  const spec = structuredClone(state.rigidSpec);
  const { goal } = spec;
  if (goal.type === 'zone') goal.position = point;
  else if (goal.type === 'tray') spec.fixtures.find((fixture) => fixture.id === goal.fixture).position = point;
  else spec.objects.find((object) => object.id === goal.on).position = point;
  state.rigidSpec = spec;
  haltPolicy(HALT.IDLE, { silent: true });
  resetArms({ keepRigidSpec: true });
}
const controlledArm = () => state.arms[state.activeArm];
const isClothFoldTask = (workflow) => workflow.id === 'fold' || workflow.id === 'fold_custom';

/**
 * Towel centre in scene pixels. Task 7 keeps its validated spot; Task 12's
 * towel sits on the cell midline, halfway between the two bases, because a
 * half fold needs each arm to reach both corners on its side - at Task 7's
 * spot the back-left corner is 67 px from Arm A's column, too close to lift.
 */
const clothOrigin = (workflow = state.currentWorkflow) => (workflow.id === 'fold_custom' ? [380, 240] : [325, 240]);

/** Task 12's corners: the two each arm carries, and the two they are laid on. */
function halfFoldCorners() {
  const corners = clothCorners(state.cloth2d.columns, state.cloth2d.rows);
  const back = [corners.backLeft, corners.backRight];
  const front = [corners.frontLeft, corners.frontRight];
  return state.foldDirection === 'back-to-front' ? { carried: back, partners: front } : { carried: front, partners: back };
}

/** Rest-pose scene position of cloth point `idx`, for the pre-run preview and the planner. */
function clothRestScenePoint(idx) {
  const [ox, oy] = clothOrigin();
  const p = state.cloth2d.restPositions;
  return [ox + p[idx * 3] * 100, oy + p[idx * 3 + 1] * 100];
}

/** Apply the current fold direction to the shared cloth simulator. */
function applyHalfFoldSelection() {
  if (state.currentWorkflow.id !== 'fold_custom') return;
  const { carried, partners } = halfFoldCorners();
  state.cloth2d.setAnchors(carried[0], carried[1], { foldAxis: 'rows', pinOnRelease: false, partners });
}

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
      $('#dataset-add').disabled = false;
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
    if (state.rigid) {
      // The rigid tasks' fingers are colliders; draw them where they are:
      // either side of the tip along the jaw axis, at the physical gap.
      const { lateral } = toolBasis(forward, forwardKinematics(armState.q, armState.arm).up);
      const [lx, ly] = [lateral[0], lateral[2]]; // world (x, z) is scene (x, y)
      const gap = state.rigid.grippers[index]?.halfGap ?? FINGER.openHalfGap;
      const [wx, wy] = [-ly * 6, lx * 6];
      const fingers = [-1, 1].map((side) => {
        const [cx, cy] = [x + lx * side * gap, y + ly * side * gap];
        return `<path class="rigid-finger" d="M${fmt(cx - wx, 1)} ${fmt(cy - wy, 1)}L${fmt(cx + wx, 1)} ${fmt(cy + wy, 1)}"/>`;
      }).join('');
      group.querySelector('.gripper').innerHTML = fingers;
      return;
    }
    // Seen from above, the jaws run back from the tip along the tool's
    // ground-projected forward vector (short when the tool points down) and
    // meet at the tip, where they close on whatever they hold.
    const [fx, fy] = forward;
    const flat = Math.hypot(fx, fy) || 1;
    const [nx, ny] = [-fy / flat, fx / flat];
    const closed = gripping(index);
    const back = 19;
    const baseGap = 7;
    const tipGap = closed ? 1.5 : 7;
    const jaws = [-1, 1].map((side) => {
      const from = [x - fx * back + nx * side * baseGap, y - fy * back + ny * side * baseGap];
      const to = [x + nx * side * tipGap, y + ny * side * tipGap];
      return `M${fmt(from[0], 1)} ${fmt(from[1], 1)} L${fmt(to[0], 1)} ${fmt(to[1], 1)}`;
    }).join(' ');
    group.querySelector('.gripper').innerHTML = `<path class="grip${closed ? ' closed' : ''}" d="${jaws}"/>${closed ? `<circle class="grip-held" cx="${fmt(x, 1)}" cy="${fmt(y, 1)}" r="4"/>` : ''}`;
  });
  updateGoals();
  updateCloth2D();
  updateRigid2D();
  pushViewportState();
  updateTelemetry();
}

function updateCloth2D() {
  const clothGroup = $('#cloth-2d');
  if (!clothGroup) return;
  const isFold = isClothFoldTask(state.currentWorkflow);
  setHidden(clothGroup, !isFold);
  // The fold plan does not chase per-arm goal cubes; the fold guide below
  // shows what each arm is doing instead. The rigid tasks' goal is where
  // the object ends up, drawn by updateRigid2D.
  setHidden($('#goals'), isFold || Boolean(state.rigid));
  if (!isFold) return;

  const { creasePoints, faceUp, faceDown } = state.cloth2d.get2DPolygons(clothOrigin(), 100);
  const cells = (quads) => quads.map((quad) => `${quad.map((pt, i) => `${i ? 'L' : 'M'}${fmt(pt[0], 1)} ${fmt(pt[1], 1)}`).join(' ')} Z`).join(' ');
  const creaseString = (pts) => pts.map((p, i) => `${i ? 'L' : 'M'}${fmt(p[0], 1)} ${fmt(p[1], 1)}`).join(' ');
  const isCustom = state.currentWorkflow.id === 'fold_custom';

  clothGroup.innerHTML = `
    ${isCustom ? '' : `
    <rect class="cloth-2d-target" x="250" y="180" width="75" height="120" rx="3" />
    <text class="cloth-2d-target-label" x="287.5" y="318" text-anchor="middle">FOLDED TARGET</text>`}
    <path class="cloth-2d-base" d="${cells(faceUp)}" />
    ${faceDown.length ? `<path class="cloth-2d-base cloth-2d-flipped" d="${cells(faceDown)}" />` : ''}
    <path class="cloth-2d-crease" d="${creaseString(creasePoints)}" />
    ${isCustom ? configurableFoldGuideSvg() : foldGuideSvg()}
  `;
}

/**
 * Top-down view of the rigid scene: the place target, then every object
 * lowest first with a cast shadow, scaled by height like the arm joints.
 */
function updateRigid2D() {
  const group = $('#rigid-2d');
  setHidden(group, !state.rigid);
  if (!state.rigid) return;
  const spec = state.rigidSpec;
  const snapshot = state.rigid.snapshot();
  const { success, placed } = rigidOutcome(spec, state.rigid);
  const met = placed ? ' met' : '';
  const parts = [];
  if (spec.goal.type === 'zone') {
    const [cx, cy] = spec.goal.position;
    const [w, d] = spec.goal.size;
    parts.push(`<rect class="rigid-zone${met}" x="${cx - w / 2}" y="${cy - d / 2}" width="${w}" height="${d}" rx="3"/>`);
    parts.push(`<text class="rigid-label" x="${cx}" y="${cy + d / 2 + 14}" text-anchor="middle">${success ? 'PLACED' : 'PLACE ZONE'}</text>`);
  }
  for (const wall of fenceWalls(spec.fence)) {
    const [cx, cy] = wall.center;
    const [length, thickness] = wall.size;
    parts.push(`<rect class="rigid-fence" x="${fmt(-length / 2, 1)}" y="${fmt(-thickness / 2, 1)}" width="${fmt(length, 1)}" height="${fmt(thickness, 1)}" transform="translate(${fmt(cx, 1)} ${fmt(cy, 1)}) rotate(${fmt((wall.yaw * 180) / Math.PI, 2)})"/>`);
  }
  if (spec.goal.type === 'tower') {
    const [cx, cy] = spec.goal.position;
    const height = rigidOutcome(spec, state.rigid).height;
    parts.push(`<rect class="rigid-tower${height >= spec.goal.height ? ' met' : ''}" x="${cx - 21}" y="${cy - 21}" width="42" height="42" rx="3"/>`);
    parts.push(`<text class="rigid-label" x="${cx}" y="${cy + 36}" text-anchor="middle">TOWER ${height}/${spec.goal.height}</text>`);
  }
  for (const fixture of spec.fixtures || []) {
    const [cx, cy] = fixture.position;
    const [w, d] = fixture.inner;
    const t = fixture.wall ?? 4;
    const isGoal = spec.goal.fixture === fixture.id;
    parts.push(`<rect class="rigid-tray${isGoal ? met : ''}" x="${cx - w / 2 - t / 2}" y="${cy - d / 2 - t / 2}" width="${w + t}" height="${d + t}" rx="2"/>`);
    if (isGoal) parts.push(`<text class="rigid-label" x="${cx}" y="${cy + d / 2 + 18}" text-anchor="middle">${success ? 'IN TRAY' : 'TRAY'}</text>`);
  }
  const objects = spec.objects.map((object) => ({ object, ...snapshotObject(snapshot, object.id) }))
    .sort((a, b) => a.center[2] - b.center[2]);
  const heldIds = new Set(state.rigid.grippers.map((gripper) => gripper.held?.id).filter(Boolean));
  for (const { object, center, quaternion } of objects) {
    const [x, y, z] = center;
    const scale = 1 + z / 900;
    const half = (object.size / 2) * scale;
    const [sx, sy] = castShadow([x, y, Math.max(0, z - object.size / 2)]);
    const held = heldIds.has(object.id) ? ' held' : '';
    if (object.shape === 'sphere') {
      parts.push(`<circle class="rigid-shadow" cx="${fmt(sx, 1)}" cy="${fmt(sy, 1)}" r="${fmt(object.size / 2, 1)}"/>`);
      parts.push(`<circle class="rigid-object${held}" cx="${fmt(x, 1)}" cy="${fmt(y, 1)}" r="${fmt(half, 1)}" fill="${object.color}"/>`);
      parts.push(`<circle cx="${fmt(x - half * 0.3, 1)}" cy="${fmt(y - half * 0.3, 1)}" r="${fmt(half * 0.3, 1)}" fill="#fff" opacity=".35"/>`);
    } else {
      const yaw = (sceneYaw(quaternion) * 180) / Math.PI;
      const rect = (cx, cy, h, extra) => `<rect ${extra} x="${fmt(-h, 1)}" y="${fmt(-h, 1)}" width="${fmt(2 * h, 1)}" height="${fmt(2 * h, 1)}" rx="2" transform="translate(${fmt(cx, 1)} ${fmt(cy, 1)}) rotate(${fmt(yaw, 1)})"/>`;
      parts.push(rect(sx, sy, object.size / 2, 'class="rigid-shadow"'));
      parts.push(rect(x, y, half, `class="rigid-object${held}" fill="${object.color}"`));
    }
  }
  for (const ball of snapshot.projectiles || []) {
    const [x, y, z] = [ball.position[0] * 1000, ball.position[2] * 1000, ball.position[1] * 1000];
    const [sx, sy] = castShadow([x, y, Math.max(0, z - ball.size / 2)]);
    parts.push(`<circle class="rigid-shadow" cx="${fmt(sx, 1)}" cy="${fmt(sy, 1)}" r="${fmt(ball.size / 2, 1)}"/>`);
    parts.push(`<circle class="rigid-projectile" cx="${fmt(x, 1)}" cy="${fmt(y, 1)}" r="${fmt((ball.size / 2) * (1 + z / 900), 1)}"/>`);
  }
  if (spec.goal.type === 'stack') {
    const base = snapshotObject(snapshot, spec.goal.on).center;
    parts.push(`<text class="rigid-label" x="${fmt(base[0], 1)}" y="${fmt(base[1] + 34, 1)}" text-anchor="middle">${success ? 'STACKED' : 'STACK HERE'}</text>`);
  }
  group.innerHTML = parts.join('');
}

/**
 * Task 12 guide: the fold line across the towel, and an arrow from each
 * corner an arm carries to the corner it is laid on - drawn from the rest
 * pose, so it previews the selected fold before a run and stays as the
 * reference during one. Each carried corner's ring fills while held.
 */
function configurableFoldGuideSvg() {
  const cloth = state.cloth2d;
  const { carried, partners } = halfFoldCorners();
  const [ox, oy] = clothOrigin();
  const halfWidth = (cloth.width / 2) * 100;
  const { cornerGaps, folded } = cloth.getHalfFoldMetrics();
  const released = cloth.wasCaptured[0] && cloth.wasCaptured[1] && !cloth.captured[0] && !cloth.captured[1];
  const arrows = carried.map((idx, arm) => {
    const [fx, fy] = clothRestScenePoint(idx);
    const [tx, ty] = clothRestScenePoint(partners[arm]);
    // Bow each arrow outward, away from the other arm, so both stay readable.
    const bow = (arm === 0 ? -1 : 1) * 28;
    return `
    <path class="fold-arrow" d="M${fmt(fx, 1)} ${fmt(fy, 1)} Q${fmt(fx + bow, 1)} ${fmt((fy + ty) / 2, 1)} ${fmt(tx, 1)} ${fmt(ty, 1)}" marker-end="url(#fold-arrowhead)" />
    <g class="fold-mark ${cloth.captured[arm] || cloth.wasCaptured[arm] ? 'held' : 'pending'}" data-arm="${arm ? 'B' : 'A'}">
      <circle cx="${fmt(fx, 1)}" cy="${fmt(fy, 1)}" r="9" />
    </g>`;
  }).join('');
  const direction = state.foldDirection === 'back-to-front' ? 'Back edge onto front edge' : 'Front edge onto back edge';
  const status = released
    ? (folded ? `Folded · 4 corners → 2 (${cornerGaps.map((gap) => `${fmt(gap * 10, 1)} cm`).join(' / ')} apart)` : `Placed · corners ${cornerGaps.map((gap) => `${fmt(gap * 10, 1)} cm`).join(' / ')} from their partners`)
    : `${direction} · 4 corners → 2`;
  return `
    <defs><marker id="fold-arrowhead" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" class="fold-arrowhead" /></marker></defs>
    <line class="fold-line" x1="${fmt(ox - halfWidth - 6, 1)}" y1="${oy}" x2="${fmt(ox + halfWidth + 6, 1)}" y2="${oy}" />
    ${arrows}
    <text class="fold-stage" x="96" y="112">${status}</text>`;
}

/** Task 7 guide: fold line, each arm's corner, and where B's corner goes next. */
function foldGuideSvg() {
  const cloth = state.cloth2d;
  const stage = foldGuideStage(cloth);
  const { cornerA, cornerB, foldX, place, farY } = FOLD_GUIDE;
  const [ax, ay] = cloth.tablePinA ? [325 + cloth.tablePinA.x * 100, 240 + cloth.tablePinA.y * 100] : clothPointToScene(cloth, cloth.anchorsA[0]);
  const [bx, by] = clothPointToScene(cloth, cloth.anchorsB[0]);
  const ring = (x, y, arm, label, held) => `
    <g class="fold-mark ${held ? 'held' : 'pending'}" data-arm="${arm}">
      <circle cx="${fmt(x, 1)}" cy="${fmt(y, 1)}" r="11" />
      <text x="${fmt(x, 1)}" y="${fmt(y - 17, 1)}" text-anchor="middle">${label}</text>
    </g>`;
  const marks = [];
  if (stage === 'grasp') {
    marks.push(ring(cornerA[0], cornerA[1], 'A', 'A grasp', cloth.captured[0]));
    marks.push(ring(cornerB[0], cornerB[1], 'B', 'B grasp', cloth.captured[1]));
  } else {
    marks.push(stage === 'lift' ? ring(ax, ay, 'A', 'A pin', true) : `<g class="fold-pin"><circle cx="${fmt(ax, 1)}" cy="${fmt(ay, 1)}" r="5" /></g>`);
  }
  if (stage === 'lift' || stage === 'place') {
    // B's corner is lifted over the fold line, so the arrow arcs away from the towel.
    const arc = `M${fmt(bx, 1)} ${fmt(by, 1)} Q${foldX} ${fmt(Math.min(by, place[1]) - 45, 1)} ${place[0] + 6} ${place[1] - 4}`;
    marks.push(`<path class="fold-arrow" d="${arc}" marker-end="url(#fold-arrowhead)" />`);
    marks.push(`<g class="fold-mark place"><circle cx="${place[0]}" cy="${place[1]}" r="15" /><text x="${place[0] - 20}" y="${place[1] + 4}" text-anchor="end">B place</text></g>`);
  }
  if (stage === 'done') marks.push(`<g class="fold-mark done"><circle cx="${place[0]}" cy="${place[1]}" r="11" /><path d="M${place[0] - 5} ${place[1]}l4 4 7-8" /></g>`);
  return `
    <defs><marker id="fold-arrowhead" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" class="fold-arrowhead" /></marker></defs>
    <line class="fold-line" x1="${foldX}" y1="${cornerA[1] - 6}" x2="${foldX}" y2="${farY + 6}" />
    ${stage === 'grasp' ? `<text class="fold-line-label" x="${foldX}" y="${cornerA[1] - 12}" text-anchor="middle">FOLD LINE</text>` : ''}
    ${marks.join('')}
    <text class="fold-stage" x="96" y="112">${FOLD_STAGES[stage]}</text>`;
}

/**
 * The cloth advances only with an accepted control frame. This is deliberately
 * outside either renderer: 2-D and 3-D therefore display the same buffered
 * physical state rather than integrating separate, refresh-rate-dependent
 * towels.
 */
function advanceClothPhysics() {
  if (!isClothFoldTask(state.currentWorkflow)) return;
  const targets = activeArms().map((armState) => {
    const tip = tipOf(armState);
    const [ox, oy] = clothOrigin();
    return { x: (tip[0] - ox) / 100, y: (tip[1] - oy) / 100, z: tip[2] / 100 };
  });
  // The fold plan says when each gripper closes and opens; a manually driven
  // arm has no such command and grasps by proximity instead.
  const grips = state.policy.grips?.[state.policy.pathIndex - 1];
  // Two fixed cloth steps per 25 Hz command give the springs time to settle
  // between waypoints without making rendering cadence part of the dynamics.
  for (let substep = 0; substep < 2; substep += 1) {
    state.cloth2d.step({ targetA: targets[0], targetB: targets[1], grips });
  }
  const metrics = state.cloth2d.getFoldMetrics();
  state.policy.stageScore = scoreTaskStages(state.currentWorkflow, { cloth: state.cloth2d, tips: activeArms().map(tipOf) });
  const held = state.cloth2d.captured.map((value, index) => `${index ? 'B' : 'A'} ${value ? 'held' : 'free'}`).join(' · ');
  const score = state.policy.stageScore;
  // A half fold is measured corner-to-partner; Task 7 by its two held corners.
  const fold = score.metrics
    ? `corners ${score.metrics.cornerGaps.map((gap) => fmt(gap * 10, 1)).join(' / ')} cm${score.complete ? ' · folded' : ''}`
    : `${fmt(metrics.frontDistance * 10, 1)} cm${metrics.folded ? ' · folded' : ''}`;
  $('#cloth-status').textContent = `Cloth frames ${state.cloth2d.history.length} / 120 · ${held} · ${score.stage} ${Math.round(score.reward * 100)}% · fold: ${fold}`;
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
  const planning = state.policy.planning;
  const pct = planning ? 18 : computePolicyProgress(current, total);

  fill.style.width = `${pct}%`;
  if (label) label.textContent = planning ? 'search' : `${pct}%`;
  if (wrap) {
    wrap.setAttribute('aria-valuenow', String(pct));
    wrap.setAttribute('aria-busy', String(planning));
    wrap.classList.toggle('running', state.policy.status === HALT.RUNNING);
    wrap.classList.toggle('planning', planning);
    wrap.classList.toggle('reached', state.policy.status === HALT.REACHED);
  }
  updateSolverTicker();
}

/**
 * Live solver progress ticker, shown for every task - not just the
 * bimanual fold - since every task runs the same demo planner underneath.
 * Ticks on its own interval (below) so the elapsed time keeps advancing
 * between discrete policy steps, the same way the sim-time clock does.
 */
function updateSolverTicker() {
  const el = $('#solver-ticker');
  if (!el) return;
  const planning = state.policy.planning;
  const total = state.policy.path?.length || (state.policy.status === HALT.RUNNING ? policyBudget() : 0);
  const elapsedMs = state.policy.solveStartedAt !== null ? performance.now() - state.policy.solveStartedAt : 0;
  el.textContent = planning
    ? `Planning · ${state.policy.planningPhase || 'starting'} · ${(elapsedMs / 1000).toFixed(1)}s`
    : formatSolverTicker({ steps: state.policy.steps, totalSteps: total, elapsedMs });
  el.classList.toggle('running', state.policy.status === HALT.RUNNING || planning);
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
    if (historyEntry.rigidSnapshot) state.rigid?.restore(historyEntry.rigidSnapshot);
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
    arms: activeArms().map((armState, index) => ({
      id: armState.arm.id,
      q: [...armState.q],
      goal: [...armState.goal],
      gripping: gripping(index),
      fingerHalfGap: state.rigid ? state.rigid.grippers[index]?.halfGap ?? FINGER.openHalfGap : null,
    })),
    rigid: state.rigid ? { spec: state.rigidSpec, snapshot: state.rigid.snapshot() } : null,
    taskId: state.currentWorkflow.id,
    policyProgress: state.policy.path?.length ? state.policy.pathIndex / state.policy.path.length : 0,
    clothSnapshot: state.cloth2d.snapshot(),
    clothOrigin: clothOrigin(),
    foldGuide: state.currentWorkflow.id === 'fold' ? {
      stage: foldGuideStage(state.cloth2d),
      cornerB: clothPointToScene(state.cloth2d, state.cloth2d.anchorsB[0]),
      pinA: state.cloth2d.tablePinA ? [325 + state.cloth2d.tablePinA.x * 100, 240 + state.cloth2d.tablePinA.y * 100] : null,
    } : null,
  });
}

const tipOf = (armState) => forwardKinematics(armState.q, armState.arm).points.at(-1);
/** Whether arm `index` has a towel corner in its closed gripper. */
const gripping = (index) => (state.rigid
  ? Boolean(state.rigid.grippers[index]?.closed)
  : isClothFoldTask(state.currentWorkflow) && Boolean(state.cloth2d.captured[index]));
const armError = (armState) => distance(tipOf(armState), armState.goal);

function updateTelemetry() {
  if (state.rigid) { updateRigidTelemetry(); return; }
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

/** Rigid tasks score the object, not the tool: its distance to the goal, and whether it is placed and at rest. */
function updateRigidTelemetry() {
  const outcome = rigidOutcome(state.rigidSpec, state.rigid);
  $('#distance').textContent = `object ${fmt(outcome.error / 10, 1)} cm`;
  $('#tool-height').textContent = activeArms().map((armState) => `${fmt(tipOf(armState)[2] / 10, 1)}`).join(' / ') + ' cm';
  $('#reward').textContent = fmt(outcome.success ? 0 : -outcome.error / 100, 3);
  $('#step').textContent = `${state.step} / ${compiled.environment.max_steps}`;
  const score = outcome.success ? 100 : clamp(90 - outcome.error / 2.7, 0, 90);
  $('#reward-bar').style.width = `${score}%`;
  $('#reward-bar-value').textContent = `${Math.round(score)}%`;
  const actualSafety = evaluateCellSafety(activeArms().map(({ q, arm }) => ({ q, arm })), compiled.environment.safety);
  const reason = state.safetyNotice || actualSafety.reason;
  const safetyCopy = { floor: 'Blocked at floor', workspace: 'Blocked at floor edge', collision: 'Blocked arm collision', rate: 'Blocked joint-step jump' };
  $('#safety-state').textContent = reason ? safetyCopy[reason] : outcome.held ? 'Holding object' : 'Within floor + collision limits';
  $('#safety-state').style.color = reason ? '#b04a24' : '#45861a';
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
  if (state.stack) { shootAt([point[0], point[1], 20]); return; }
  if (state.rigid) { moveRigidGoal(point); return; }
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
  $('#dataset-add').disabled = false;
  $('#replay').disabled = false;
  if (state.transitions.length === MAX_EPISODE_TRANSITIONS) stopRecording();
}

function stopRecording() {
  state.recording = false;
  $('#record').textContent = '● Record';
}

/* ---------- dataset export ---------- */

function currentEpisodeArtifact() {
  return buildEpisodeArtifact({
    environment: compiled.environment.id,
    task: state.currentWorkflow,
    transitions: state.transitions,
    voice: state.voice,
    arms: state.armCount,
    halt: state.policy.status,
  });
}

function downloadJson(filename, data) {
  const url = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }));
  const link = document.createElement('a');
  link.href = url; link.download = filename; link.click();
  URL.revokeObjectURL(url);
}

function setDatasetStatus(message, isError) {
  const status = $('#dataset-status');
  status.textContent = message;
  status.classList.toggle('ok', !isError);
  setHidden(status, false);
}

function updateDatasetUI() {
  const count = state.dataset.length;
  $('#dataset-count').textContent = count;
  $('#dataset-count-suffix').textContent = count === 1 ? '' : 's';
  $('#dataset-download').disabled = count === 0;
  $('#dataset-clear').disabled = count === 0;
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
      // Task 16: a click in 3-D fires along the view ray at whatever it hits.
      onShoot: (target, from) => shootAt(target, from),
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
    setLoaderProgress(70, 'Initializing spring-network cloth physics…');
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
  state.policy.grips = null;
  state.policy.pathIndex = 0;
  state.policy.accumulator = 0;
  // The fold plan's own deterministic frame count (approach, pin, lift,
  // retract, cross, place, then a settle tail long enough for the cloth
  // solver to relax) currently runs to ~477 steps; give it enough budget to
  // finish without truncating the settle tail, matching the raised ceiling
  // in policyBudget() and the slider's max in index.html.
  // The rigid tasks' pick-and-place plans (~380 steps with their settle
  // tail) also outrun a 200-step horizon, and share the same ceiling.
  state.policy.budget = runsFullPlan(workflow) ? FOLD_STEP_BUDGET : workflow.horizon_steps;
  state.policy.loop = isClothFoldTask(workflow);
  $('#policy-loop').checked = state.policy.loop;
  $('#policy-budget').value = state.policy.budget;
  $('#policy-budget-value').textContent = state.policy.budget;
  $('#policy-profile').textContent = policyRecipeFor(workflow).label;
  setHidden($('#cloth-status'), !isClothFoldTask(workflow));
  setHidden($('#cloth-settings-link'), !isClothFoldTask(workflow));
  setHidden($('#fold-direction-picker'), workflow.id !== 'fold_custom');
  setHidden($('#live-status'), !isLiveRigidTask(workflow));
  $('#cloth-status').textContent = 'Cloth frames 0 / 120';
  // Task 7 always grasps the true corners; only the configurable task varies
  // it, and its own selection re-applies below.
  if (workflow.id !== 'fold_custom') state.cloth2d?.setAnchors(0, state.cloth2d.columns);
  state.cloth2d?.reset();
  applyHalfFoldSelection();
  viewport.instance?.resetCloth?.();
  state.rigidSpec = isRigidTask(workflow) ? structuredClone(workflow.rigid) : null;
  buildRigidScene();
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
  [HALT.MISSED]: () => `Halted · plan finished at step ${state.policy.steps}, object not at goal`,
};

function renderHaltState() {
  const element = $('#halt-state');
  const planning = state.policy.planning;
  element.textContent = planning ? `Planning · ${state.policy.planningPhase || 'starting'}` : HALT_COPY[state.policy.status]();
  element.className = `halt-state ${planning ? 'planning' : state.policy.status}`;
  $('#run-policy').textContent = planning ? '■ Cancel solver' : state.policy.status === HALT.RUNNING ? '■ Halt policy' : 'Run demo policy';
  updatePolicyProgressBar();
}

/** Advance one prevalidated floor- and collision-safe control frame. */
function policyStep() {
  // Task 16's controller supplies each frame as it goes; it is checked below
  // exactly like a planned one.
  const liveFrame = state.stack ? state.stack.next(state.rigid, state.arms[0].q) : null;
  const next = liveFrame ? [liveFrame.q] : state.policy.path?.[state.policy.pathIndex];
  // Every frame in state.policy.path already passed evaluateCellSafety when
  // it was planned (planSafeCellMotion / reduceSafeCellMotion /
  // planTowelFoldMotion), so running out of frames means the plan finished
  // safely, not that anything was unsafe - report it as reached rather than
  // a safety halt.
  // A rigid task is scored on where the object physically came to rest.
  if (!next) return state.rigid && !rigidOutcome(state.rigidSpec, state.rigid).success ? HALT.MISSED : HALT.REACHED;
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
  advanceClothPhysics();
  advanceRigidPhysics(liveFrame ? [liveFrame.grip] : state.policy.grips?.[state.policy.pathIndex - 1]);
  updateArms();
  addTransition();
  if (liveFrame) {
    // Endless by design: no timeline history (it would grow without bound)
    // and no budget - it stops only for the operator or the safety envelope.
    updateLiveStatus();
    return HALT.RUNNING;
  }

  const clothSnap = state.cloth2d.snapshot();
  state.frameHistory[state.policy.steps] = {
    step: state.step,
    arms: activeArms().map((a) => ({ q: [...a.q], goal: [...a.goal], lastAction: [...a.lastAction] })),
    clothSnapshot: clothSnap,
    rigidSnapshot: state.rigid?.snapshot(),
  };
  updateTimelineSlider();

  // The fold task's per-arm goal only marks where each gripper starts (the
  // corner it grasps); the choreography then carries it well away from that
  // point (lift, cross the fold line, place, retract). Both arms briefly
  // sitting near their starting goals right after the approach phase would
  // satisfy the generic distance check long before any folding happens, so
  // fold runs to the end of its own deterministic, pre-validated plan
  // instead - reached via the frame-exhaustion check above, bounded by the
  // step budget below like any other task.
  // The rigid tasks' tool goal is likewise only a waypoint; they are
  // scored when their plan (which ends in a settle hold) runs out.
  if (runsFullPlan(state.currentWorkflow)) {
    return state.policy.steps >= policyBudget() ? HALT.BUDGET : HALT.RUNNING;
  }
  return haltState({ error, steps: state.policy.steps, budget: policyBudget() });
}

const policyBudget = () => clamp(Math.round(state.policy.budget), 1, FOLD_STEP_BUDGET);

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

const yieldForSolverFeedback = () => new Promise((resolve) => requestAnimationFrame(() => setTimeout(resolve, 0)));

async function runPolicyPlanning(token) {
  const isCurrent = () => state.policy.planning && token === state.policy.planningToken;
  if (!isCurrent()) return;
  clearTimeout(state.policy.restart);
  state.policy.restart = null;
  state.policy.planningPhase = 'trying IK candidates';
  renderHaltState();
  await yieldForSolverFeedback();
  if (!isCurrent()) return;
  if (!replanArms()) {
    state.policy.planning = false;
    state.policy.planningPhase = null;
    state.policy.status = HALT.SAFETY;
    renderHaltState();
    return;
  }

  const poses = activeArms().map(({ q, arm }) => ({ q, arm }));
  if (state.stack) {
    // Task 16 has no single plan: the controller decides frame by frame.
    state.stack.interrupt();
    state.policy.planning = false;
    state.policy.planningPhase = null;
    state.policy.path = null;
    state.policy.grips = null;
    state.policy.pathIndex = 0;
    state.policy.status = HALT.RUNNING;
    state.policy.steps = 0;
    state.policy.accumulator = 0;
    state.policy.solveStartedAt = performance.now();
    state.frameHistory = [];
    renderHaltState();
    state.policy.frame = requestAnimationFrame(policyFrame);
    return;
  }
  let motion = null;
  state.policy.planningPhase = 'checking collision-safe route';
  renderHaltState();
  await yieldForSolverFeedback();
  if (!isCurrent()) return;
  if (isClothFoldTask(state.currentWorkflow) && activeArms().length === 2) {
    state.policy.planningPhase = 'calibrating cloth-aware policy';
    renderHaltState();
    await yieldForSolverFeedback();
    if (!isCurrent()) return;
    const warmStart = bootstrapPolicy(state.currentWorkflow);
    if (state.currentWorkflow.id === 'fold_custom') {
      // Half fold: each arm carries its side's corner onto the one opposite.
      const { carried, partners } = halfFoldCorners();
      const carry = carried.map((idx, arm) => [clothRestScenePoint(idx), clothRestScenePoint(partners[arm])]);
      motion = planHalfFoldMotion(poses, compiled.environment.safety, { ...warmStart.profile, carry });
    } else {
      motion = planTowelFoldMotion(poses, compiled.environment.safety, warmStart.profile);
    }
  }
  if (state.rigid) {
    state.policy.planningPhase = 'planning top-down grasp';
    renderHaltState();
    await yieldForSolverFeedback();
    if (!isCurrent()) return;
    // Planned from the objects' live poses, not the task file: a cube that
    // has been knocked or moved is grasped where it actually is.
    motion = planRigidTask(state.rigidSpec, state.rigid, poses[0], compiled.environment.safety);
    // A rigid task has no fallback plan: the generic reach below would
    // arrive at its goal tool-first without ever closing the gripper.
    if (!motion) motion = false;
  }
  if (motion === null) {
    motion = planSafeCellMotion(
      poses,
      activeArms().map(({ plan }) => plan.q),
      compiled.environment.safety,
    );
    if (motion) {
      state.policy.planningPhase = 'reducing verified route';
      renderHaltState();
      await yieldForSolverFeedback();
      if (!isCurrent()) return;
      // First find a route, then reduce it. The reducer resamples and
      // validates every proposed shortcut, so fewer steps never means a
      // looser envelope.
      motion = reduceSafeCellMotion(poses, motion.frames, compiled.environment.safety);
    }
  }
  if (!motion) {
    state.policy.planning = false;
    state.policy.planningPhase = null;
    state.safetyNotice = 'workspace';
    state.policy.status = HALT.SAFETY;
    renderHaltState();
    updateTelemetry();
    return;
  }
  state.policy.planning = false;
  state.policy.planningPhase = null;
  state.policy.path = motion.frames;
  state.policy.grips = motion.grips || null;
  state.policy.pathIndex = 0;
  state.policy.status = HALT.RUNNING;
  state.policy.steps = 0;
  state.policy.accumulator = 0;
  state.policy.solveStartedAt = performance.now();
  state.timeline.scrubbing = false;

  const clothSnap = state.cloth2d.snapshot();
  state.frameHistory = [{
    step: state.step,
    arms: activeArms().map((a) => ({ q: [...a.q], goal: [...a.goal], lastAction: [...a.lastAction] })),
    clothSnapshot: clothSnap,
    rigidSnapshot: state.rigid?.snapshot(),
  }];

  renderHaltState();
  updateTimelineSlider();
  state.policy.frame = requestAnimationFrame(policyFrame);
}

function startPolicy() {
  if (state.policy.status === HALT.RUNNING || state.policy.planning) return;
  clearTimeout(state.policy.restart);
  state.policy.restart = null;
  state.policy.planning = true;
  state.policy.solveStartedAt = performance.now();
  const token = ++state.policy.planningToken;
  renderHaltState();
  // Yield once so the operator sees feedback before the synchronous IK/RRT
  // work begins. The same token makes a pending solve safely cancellable.
  requestAnimationFrame(() => setTimeout(() => {
    if (!state.policy.planning || token !== state.policy.planningToken) return;
    runPolicyPlanning(token);
  }, 0));
}

function haltPolicy(status = HALT.OPERATOR, { silent = false } = {}) {
  if (state.policy.frame !== null) cancelAnimationFrame(state.policy.frame);
  clearTimeout(state.policy.restart);
  state.policy.frame = null;
  state.policy.restart = null;
  state.policy.planning = false;
  state.policy.planningPhase = null;
  state.policy.planningToken += 1;
  if (silent) { state.policy.status = HALT.IDLE; state.policy.steps = 0; }
  else state.policy.status = status;
  renderHaltState();
}

function resetArms({ keepRigidSpec = false } = {}) {
  const workflow = state.currentWorkflow;
  for (const armState of state.arms) {
    armState.q = [...HOME_POSE];
    armState.lastAction = Array(6).fill(0);
  }
  // A full reset returns the entire task contract to its loaded state, not
  // merely the joints. This puts both goal cubes back at their scenario
  // locations/heights before the solver is asked to make a fresh safe plan.
  setArmGoal(state.arms[0], [...workflow.goal, workflow.goal_height], { replan: false });
  setArmGoal(state.arms[1], [...(workflow.goal_b || workflow.goal), workflow.goal_height], { replan: false });
  state.activeArm = 0;
  replanArms();
  state.step = 0;
  state.safetyNotice = null;
  state.policy.path = null;
  state.policy.grips = null;
  state.policy.pathIndex = 0;
  state.policy.accumulator = 0;
  state.policy.steps = 0;
  state.policy.solveStartedAt = null;
  state.frameHistory = [];
  state.timeline.currentStep = 0;
  state.timeline.scrubbing = false;
  state.cloth2d?.reset();
  viewport.instance?.resetCloth?.();
  // Objects go back to their start poses; a moved place target is kept only
  // when the move itself asked for the reset.
  if (!keepRigidSpec && isRigidTask(workflow)) state.rigidSpec = structuredClone(workflow.rigid);
  buildRigidScene();
  // The sim-time readout and any pending voice auto-stop are timers, not
  // policy/episode state, and neither was touched here — the clock kept
  // counting from page load and a running capture kept its own countdown
  // across a reset.
  state.startedAt = performance.now();
  if (state.voice.captureTimeout) {
    clearTimeout(state.voice.captureTimeout);
    state.voice.captureTimeout = null;
  }
  [...$('#arm-switch').children].forEach((chip, index) => chip.classList.toggle('active', index === state.activeArm));
  syncSliders();
  updateArms();
  updateTimelineSlider();
  updateSolverTicker();
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
  advanceRigidPhysics();
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
  $('#fold-direction').addEventListener('change', (event) => {
    state.foldDirection = event.target.value;
    applyHalfFoldSelection();
    haltPolicy(HALT.IDLE, { silent: true });
    resetArms();
  });
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
  // Tuning on the cloth settings page (usually another tab) takes effect on
  // the next cloth step without resetting the towel.
  onClothSettingsChange((settings) => state.cloth2d.configure(settings));
  $('#run-policy').addEventListener('click', () => {
    if (state.policy.status === HALT.RUNNING || state.policy.planning) haltPolicy(HALT.OPERATOR);
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
    $('#recording-count').textContent = '0'; $('#download').disabled = true; $('#dataset-add').disabled = true; $('#replay').disabled = true;
    $('#record').textContent = '■ Stop recording';
  });
  $('#voice-record').addEventListener('click', toggleVoiceCapture);
  $('#voice-play').addEventListener('click', () => { if (state.voice.dataUrl) new Audio(state.voice.dataUrl).play().catch(() => setVoiceStatus('Audio playback was blocked by the browser', true)); });
  $('#transcript').addEventListener('input', (event) => { state.voice.transcript = event.target.value; });
  $('#replay').addEventListener('click', replayEpisode);
  $('#import').addEventListener('click', () => $('#import-file').click());
  $('#import-file').addEventListener('change', (event) => { if (event.target.files[0]) importEpisode(event.target.files[0]); event.target.value = ''; });
  $('#download').addEventListener('click', () => {
    downloadJson(`armlab-${state.currentWorkflow.id}-${Date.now()}.json`, currentEpisodeArtifact());
  });
  $('#dataset-add').addEventListener('click', () => {
    if (!state.transitions.length) return;
    const episode = currentEpisodeArtifact();
    const mismatched = state.dataset.find((existing) => existing.arms !== episode.arms);
    if (mismatched) {
      setDatasetStatus(`This dataset already has a ${mismatched.arms}-arm episode; a ${episode.arms}-arm one can't join it. Download or clear the dataset first.`, true);
      return;
    }
    state.dataset.push(episode);
    setDatasetStatus(`Added a ${episode.transitions.length}-step episode.`, false);
    updateDatasetUI();
  });
  $('#dataset-download').addEventListener('click', () => {
    const manifest = buildDatasetManifest({ episodes: state.dataset, fps: compiled.environment.control_hz });
    if (manifest.error) { setDatasetStatus(manifest.message, true); return; }
    downloadJson(`armlab-dataset-${Date.now()}.json`, manifest);
    setDatasetStatus(`Downloaded ${manifest.info.total_episodes} episodes, ${manifest.info.total_frames} frames.`, false);
  });
  $('#dataset-clear').addEventListener('click', () => {
    state.dataset = [];
    setDatasetStatus('Dataset cleared.', false);
    updateDatasetUI();
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
    advanceRigidPhysics();
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

renderScenarioSelect(); renderFamilyFilters(); renderModels(); renderDatasets(); renderStudyPath(); makeSliders(); installListeners(); updateDatasetUI();
loadWorkflow(compiled.workflows[0].id);
renderHaltState();
setInterval(() => { $('#sim-time').textContent = `T + ${fmt((performance.now() - state.startedAt) / 1000, 1).padStart(4, '0')} s`; }, 100);
setInterval(updateSolverTicker, 100);
