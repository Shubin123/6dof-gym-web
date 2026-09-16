/**
 * Kinematics for the lab's 6-DOF arm.
 *
 * Each joint turns about an axis of its own moving frame — yaw, pitch, pitch,
 * yaw, pitch, yaw — so the chain bends out of any single plane. Every pose has
 * a real height and a real lateral offset, and both the top-down SVG scene and
 * the 3-D viewport are views of the same spatial chain.
 */
export const ARM = Object.freeze({
  id: 'A',
  base: [200, 345],
  baseHeight: 90,
  lengths: [110, 100, 90, 80, 75, 70],
  /** 'yaw' turns about the frame's up axis, 'pitch' about its lateral axis. */
  axes: ['yaw', 'pitch', 'pitch', 'yaw', 'pitch', 'yaw'],
  jointLimit: 1.7,
  yawLimit: Math.PI,
  maxActionDelta: 0.05,
  mirror: false,
});

/**
 * The right-hand arm of a bimanual cell.
 *
 * It is the same manipulator reflected about its own base column, so a joint
 * vector describes the mirrored posture rather than a second set of limits.
 * Only tasks that genuinely need two grippers declare it.
 */
export const ARM_B = Object.freeze({ ...ARM, id: 'B', base: [560, 345], mirror: true });

/** Height band a goal may occupy, in scene pixels above the table. */
export const GOAL_Z = Object.freeze({ min: 0, max: 160, rest: 60 });

export const MAX_EPISODE_TRANSITIONS = 200;
export const GOAL_TOLERANCE_PX = 8;

/** Why a policy run stopped. An episode always ends in one of these. */
export const HALT = Object.freeze({
  RUNNING: 'running',
  IDLE: 'idle',
  REACHED: 'reached',
  BUDGET: 'budget',
  OPERATOR: 'operator',
});

export const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

/** Joint 0 is the base yaw and swings further than the articulated joints. */
export const limitOf = (arm, index) => (index === 0 ? arm.yawLimit : arm.jointLimit);
export const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], (a[2] || 0) - (b[2] || 0));

const add = (a, b, scale = 1) => [a[0] + b[0] * scale, a[1] + b[1] * scale, a[2] + b[2] * scale];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const unit = (a) => { const length = norm(a) || 1; return [a[0] / length, a[1] / length, a[2] / length]; };

/** Rodrigues rotation of `v` about the unit axis `k` by `angle`. */
function rotate(v, k, angle) {
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  const crossed = cross(k, v);
  const scaled = dot(k, v) * (1 - cos);
  return [
    v[0] * cos + crossed[0] * sin + k[0] * scaled,
    v[1] * cos + crossed[1] * sin + k[1] * scaled,
    v[2] * cos + crossed[2] * sin + k[2] * scaled,
  ];
}

const mirrorPoint = (point, arm) => (arm.mirror ? [2 * arm.base[0] - point[0], point[1], point[2] ?? 0] : [point[0], point[1], point[2] ?? 0]);

/**
 * Walk the chain, carrying a moving frame.
 *
 * Returns the joint positions, the world-space rotation axis of every joint —
 * which the solver needs — and the tool's forward direction.
 */
export function forwardKinematics(q, arm = ARM) {
  let position = [arm.base[0], arm.base[1], arm.baseHeight];
  let forward = [0, -1, 0];
  let up = [0, 0, 1];
  const points = [position];
  const axes = [];

  for (let i = 0; i < arm.lengths.length; i += 1) {
    const lateral = cross(forward, up);
    const axis = arm.axes[i] === 'yaw' ? up : lateral;
    axes.push(axis);
    forward = unit(rotate(forward, axis, q[i]));
    up = unit(rotate(up, axis, q[i]));
    position = add(position, forward, arm.lengths[i]);
    points.push(position);
  }

  const mirrored = arm.mirror;
  const flip = (point) => (mirrored ? [2 * arm.base[0] - point[0], point[1], point[2]] : point);
  const flipVector = (vector) => (mirrored ? [-vector[0], vector[1], vector[2]] : vector);
  return {
    points: points.map(flip),
    axes: axes.map(flipVector),
    forward: flipVector(forward),
    up: flipVector(up),
  };
}

/**
 * Solve the arm for a point in space with deterministic multi-start CCD.
 *
 * Each joint turns about its own world-space axis by the angle that best
 * aligns the tool with the target in the plane perpendicular to that axis. A
 * fixed set of seed postures keeps the result reproducible after an operator
 * has moved the sliders, and a mirrored arm is solved in its reflected frame.
 */
export function solveInverseKinematics(target, arm = ARM) {
  const solverArm = arm.mirror ? { ...arm, mirror: false } : arm;
  const goal = mirrorPoint(target, arm);
  const seeds = [
    [0, 0.6, -1.1, 0, -0.6, 0],
    [-0.8, 0.7, -1.2, 0.3, -0.7, 0],
    [0.8, 0.7, -1.2, -0.3, -0.7, 0],
    [-1.6, 0.9, -1.4, 0, -0.5, 0],
    [1.6, 0.9, -1.4, 0, -0.5, 0],
    [-0.4, 1.2, -1.6, 0.5, -1, 0.3],
    [0.4, 0.3, -0.8, -0.5, -0.4, -0.3],
  ];

  let best = null;
  for (const seed of seeds) {
    const q = seed.map((value, index) => clamp(value, -limitOf(solverArm, index), limitOf(solverArm, index)));
    for (let iteration = 0; iteration < 120; iteration += 1) {
      for (let joint = q.length - 1; joint >= 0; joint -= 1) {
        const { points, axes } = forwardKinematics(q, solverArm);
        const pivot = points[joint];
        const axis = axes[joint];
        const toTip = sub(points.at(-1), pivot);
        const toGoal = sub(goal, pivot);
        const flatTip = sub(toTip, [axis[0] * dot(axis, toTip), axis[1] * dot(axis, toTip), axis[2] * dot(axis, toTip)]);
        const flatGoal = sub(toGoal, [axis[0] * dot(axis, toGoal), axis[1] * dot(axis, toGoal), axis[2] * dot(axis, toGoal)]);
        if (norm(flatTip) < 1e-6 || norm(flatGoal) < 1e-6) continue;
        const turn = Math.atan2(dot(cross(flatTip, flatGoal), axis), dot(flatTip, flatGoal));
        q[joint] = clamp(q[joint] + clamp(turn, -0.25, 0.25), -limitOf(solverArm, joint), limitOf(solverArm, joint));
      }
      if (distance(forwardKinematics(q, solverArm).points.at(-1), goal) < 0.5) break;
    }
    const candidate = { q, distance: distance(forwardKinematics(q, solverArm).points.at(-1), goal) };
    if (!best || candidate.distance < best.distance) best = candidate;
  }
  return best;
}

/** One safety-capped tracking update toward an already validated IK plan. */
export function guidedStep(q, target, arm = ARM, plan = solveInverseKinematics(target, arm)) {
  // Track the plan by the direct difference, never the shortest way round: a
  // wrapped path can point a limited joint at the limit it is already sitting on.
  const next = q.map((value, index) => clamp(
    value + clamp(plan.q[index] - value, -arm.maxActionDelta, arm.maxActionDelta),
    -limitOf(arm, index),
    limitOf(arm, index),
  ));
  const action = next.map((value, index) => value - q[index]);
  return { q: next, action, distance: distance(forwardKinematics(next, arm).points.at(-1), target) };
}

/**
 * Clamp a goal into the table bounds, the height band, and the arm's reach shell.
 *
 * The shell has an inner wall as well as an outer one: an articulated arm
 * cannot fold tightly enough to touch its own shoulder, and the higher a goal
 * sits the further out that wall moves, because the arm has to fold back over
 * itself to get above its own base. A goal inside the wall is pushed to the
 * nearest pose the arm can actually hold.
 */
export function projectToReachableWorkspace(point, workspace, arm = ARM) {
  const [minX, maxX, minY, maxY] = workspace.goal_workspace;
  const inBounds = (candidate) => [
    clamp(candidate[0], minX, maxX),
    clamp(candidate[1], minY, maxY),
    clamp(candidate[2], GOAL_Z.min, GOAL_Z.max),
  ];

  let candidate = inBounds([point[0], point[1], point[2] ?? GOAL_Z.rest]);
  // Height is kept and the correction is made horizontally, so raising a goal
  // slides it away from the column instead of sinking it back to the table.
  // Two passes: the table clamp can push a goal back inside the shell.
  for (let pass = 0; pass < 2; pass += 1) {
    const rise = candidate[2] - arm.baseHeight;
    const innerWall = (workspace.min_reach_px || 0) + Math.max(0, rise) * (workspace.min_reach_rise || 0);
    const minFlat = Math.sqrt(Math.max(0, innerWall ** 2 - rise ** 2));
    const maxFlat = Math.sqrt(Math.max(0, workspace.max_reach_px ** 2 - rise ** 2));

    let [dx, dy] = [candidate[0] - arm.base[0], candidate[1] - arm.base[1]];
    let flat = Math.hypot(dx, dy);
    // A goal directly over the column has no direction to be pushed along, so
    // give it one that points into the table rather than off the edge of it.
    if (flat < 1e-6) { [dx, dy, flat] = [arm.mirror ? -1 : 1, 0, 1]; }

    const corrected = clamp(flat, minFlat, maxFlat);
    if (Math.abs(corrected - flat) < 1e-9) break;
    candidate = inBounds([arm.base[0] + (dx / flat) * corrected, arm.base[1] + (dy / flat) * corrected, candidate[2]]);
  }
  return candidate;
}

/** The arm a click belongs to: the one whose base column is nearest. */
export function nearestArm(point, arms) {
  const flat = (base) => Math.hypot(point[0] - base[0], point[1] - base[1]);
  return arms.reduce((closest, arm) => (flat(arm.base) < flat(closest.base) ? arm : closest));
}

/**
 * Decide whether a running policy should stop, and say why.
 *
 * Every episode ends in a named halt state rather than simply running out of
 * frames, so an operator can tell a success apart from an exhausted budget.
 */
export function haltState({ error, steps, budget }) {
  if (error <= GOAL_TOLERANCE_PX) return HALT.REACHED;
  if (steps >= budget) return HALT.BUDGET;
  return HALT.RUNNING;
}

/** Portable browser episode envelope; voice is optional and self-contained. */
export function buildEpisodeArtifact({ environment, task, transitions, voice, arms = 1, halt = HALT.IDLE }) {
  return {
    schema: 'armlab-episode-preview/v0.4',
    environment,
    task,
    arms,
    halt,
    source: 'browser-simulation',
    transitions,
    voice: {
      transcript: voice?.transcript || '',
      mime_type: voice?.mimeType || null,
      audio_data_url: voice?.dataUrl || null,
    },
  };
}
