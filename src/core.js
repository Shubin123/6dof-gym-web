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

/** A folded, in-bounds rest pose that keeps mirrored arms on their own sides. */
export const HOME_POSE = Object.freeze([0.662, 1.276, 1.496, -1.465, 1.488, -1.217]);

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
  SAFETY: 'safety',
  // The plan ran to its end, but the physical outcome missed the goal (a
  // rigid-object task whose object did not come to rest where it should).
  MISSED: 'missed',
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

/** Shortest distance between two finite line segments in three dimensions. */
function segmentDistance(a0, a1, b0, b1) {
  const u = sub(a1, a0);
  const v = sub(b1, b0);
  const w = sub(a0, b0);
  const uu = dot(u, u);
  const uv = dot(u, v);
  const vv = dot(v, v);
  const uw = dot(u, w);
  const vw = dot(v, w);
  const denominator = uu * vv - uv * uv;
  let alongA = denominator < 1e-9 ? 0 : clamp((uv * vw - vv * uw) / denominator, 0, 1);
  let alongB = vv < 1e-9 ? 0 : clamp((uv * alongA + vw) / vv, 0, 1);
  alongA = uu < 1e-9 ? 0 : clamp((uv * alongB - uw) / uu, 0, 1);
  return distance(add(a0, u, alongA), add(b0, v, alongB));
}

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
 * Validate complete arm geometry, not merely its goal marker.
 *
 * Link centre lines must remain over the marked floor and at/above its plane.
 * In a bimanual cell, every link pair must also retain the configured physical
 * clearance. The renderers and controllers consume this single result.
 */
export function evaluateCellSafety(poses, safety = {}) {
  const bounds = safety.goal_workspace || [-Infinity, Infinity, -Infinity, Infinity];
  const [minX, maxX, minY, maxY] = bounds;
  const floorZ = safety.floor_z_px ?? 0;
  const requiredArmClearance = safety.arm_clearance_px ?? 0;
  const chains = poses.map(({ q, arm = ARM }) => ({ arm, points: forwardKinematics(q, arm).points }));
  let floorClearance = Infinity;
  let boundaryClearance = Infinity;
  for (const { points } of chains) {
    for (const [x, y, z] of points) {
      floorClearance = Math.min(floorClearance, z - floorZ);
      boundaryClearance = Math.min(boundaryClearance, x - minX, maxX - x, y - minY, maxY - y);
    }
  }

  let armClearance = Infinity;
  for (let first = 0; first < chains.length; first += 1) {
    for (let second = first + 1; second < chains.length; second += 1) {
      const a = chains[first].points;
      const b = chains[second].points;
      for (let ai = 0; ai < a.length - 1; ai += 1) {
        for (let bi = 0; bi < b.length - 1; bi += 1) {
          armClearance = Math.min(armClearance, segmentDistance(a[ai], a[ai + 1], b[bi], b[bi + 1]));
        }
      }
    }
  }

  let reason = null;
  if (floorClearance < -1e-6) reason = 'floor';
  else if (boundaryClearance < -1e-6) reason = 'workspace';
  else if (armClearance < requiredArmClearance - 1e-6) reason = 'collision';
  return {
    safe: reason === null,
    reason,
    floorClearance,
    boundaryClearance,
    armClearance,
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
/**
 * One cyclic-coordinate-descent sweep: rotate every joint, tip to base, to
 * point the tool as directly as each joint's own axis allows toward `goal`.
 * solveInverseKinematics loops this to convergence across many seeds; a
 * single sweep is also cheap enough to run once per pointer-move frame, for
 * an interactive live-drag follow (see liveDragStep) where a full multi-seed
 * search would stall the drag.
 */
function ccdSweep(q, goal, arm) {
  const next = [...q];
  for (let joint = next.length - 1; joint >= 0; joint -= 1) {
    const { points, axes } = forwardKinematics(next, arm);
    const pivot = points[joint];
    const axis = axes[joint];
    const toTip = sub(points.at(-1), pivot);
    const toGoal = sub(goal, pivot);
    const flatTip = sub(toTip, [axis[0] * dot(axis, toTip), axis[1] * dot(axis, toTip), axis[2] * dot(axis, toTip)]);
    const flatGoal = sub(toGoal, [axis[0] * dot(axis, toGoal), axis[1] * dot(axis, toGoal), axis[2] * dot(axis, toGoal)]);
    if (norm(flatTip) < 1e-6 || norm(flatGoal) < 1e-6) continue;
    const turn = Math.atan2(dot(cross(flatTip, flatGoal), axis), dot(flatTip, flatGoal));
    next[joint] = clamp(next[joint] + clamp(turn, -0.25, 0.25), -limitOf(arm, joint), limitOf(arm, joint));
  }
  return next;
}

export function solveInverseKinematics(target, arm = ARM, safety = {}, otherPoses = []) {
  const solverArm = arm.mirror ? { ...arm, mirror: false } : arm;
  const goal = mirrorPoint(target, arm);
  const seeds = [
    [...HOME_POSE],
    [0, 0.6, -1.1, 0, -0.6, 0],
    [-0.8, 0.7, -1.2, 0.3, -0.7, 0],
    [0.8, 0.7, -1.2, -0.3, -0.7, 0],
    [-1.6, 0.9, -1.4, 0, -0.5, 0],
    [1.6, 0.9, -1.4, 0, -0.5, 0],
    [-0.4, 1.2, -1.6, 0.5, -1, 0.3],
    [0.4, 0.3, -0.8, -0.5, -0.4, -0.3],
    [0.6, 1.3, 1.5, -1.45, 1.45, -1.2],
    [-0.6, 1.3, 1.5, 1.45, 1.45, 1.2],
    [1.2, 1.1, 1.4, -1.3, 1.35, -0.8],
    [-1.2, 1.1, 1.4, 1.3, 1.35, 0.8],
    [0.9, 1.5, 1.2, -1.5, 1.5, -0.5],
    [-0.9, 1.5, 1.2, 1.5, 1.5, 0.5],
  ];
  // Low-discrepancy restarts cover alternate elbow-up / elbow-down solutions.
  // They are deterministic, so a task produces the same plan on every run.
  const phases = [0.61803398875, 0.41421356237, 0.73205080757, 0.2360679775, 0.64575131106, 0.31662479036];
  for (let restart = 1; restart <= 24; restart += 1) {
    seeds.push(phases.map((phase, index) => {
      const limit = limitOf(solverArm, index);
      return (((restart * phase) % 1) * 2 - 1) * limit;
    }));
  }

  let best = null;
  let bestSafe = null;
  for (const seed of seeds) {
    const q = seed.map((value, index) => clamp(value, -limitOf(solverArm, index), limitOf(solverArm, index)));
    for (let iteration = 0; iteration < 120; iteration += 1) {
      q.splice(0, q.length, ...ccdSweep(q, goal, solverArm));
      if (distance(forwardKinematics(q, solverArm).points.at(-1), goal) < 0.5) break;
    }
    const candidate = { q, distance: distance(forwardKinematics(q, solverArm).points.at(-1), goal) };
    candidate.safety = evaluateCellSafety([...otherPoses, { q, arm }], safety);
    if (!best || candidate.distance < best.distance) best = candidate;
    if (candidate.safety.safe && (!bestSafe || candidate.distance < bestSafe.distance)) bestSafe = candidate;
  }
  const selected = bestSafe || best;
  // Reflecting a chain preserves all configured clearances, but callers expect
  // the safety result to describe the real arm rather than the solver frame.
  selected.safety = evaluateCellSafety([...otherPoses, { q: selected.q, arm }], safety);
  return selected;
}

/**
 * One interactive drag step toward `targetPoint`: a single CCD sweep (see
 * ccdSweep — cheap enough for every pointer-move frame, unlike the
 * multi-seed solveInverseKinematics search above), rate-capped exactly like
 * guidedStep, and rejected outright — the arm holds its prior pose — if the
 * resulting frame would violate floor/workspace/inter-arm safety. This is
 * what lets an operator drag the tool itself around the scene, rather than
 * only a goal marker the arm chases afterward, while recording transitions.
 */
export function liveDragStep(q, targetPoint, arm = ARM, safety = {}, otherPoses = []) {
  const solverArm = arm.mirror ? { ...arm, mirror: false } : arm;
  const goal = mirrorPoint(targetPoint, arm);
  const swept = ccdSweep(q, goal, solverArm);
  const next = swept.map((value, index) => clamp(
    q[index] + clamp(value - q[index], -arm.maxActionDelta, arm.maxActionDelta),
    -limitOf(arm, index),
    limitOf(arm, index),
  ));
  const safetyResult = evaluateCellSafety([...otherPoses, { q: next, arm }], safety);
  if (!safetyResult.safe) {
    return { q: [...q], moved: false, safety: evaluateCellSafety([...otherPoses, { q, arm }], safety) };
  }
  return { q: next, moved: true, safety: safetyResult };
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

const jointDistance = (a, b) => Math.hypot(...a.map((value, index) => value - b[index]));
const maxJointDistance = (a, b) => Math.max(...a.map((value, index) => Math.abs(value - b[index])));
const interpolateJoints = (a, b, amount) => a.map((value, index) => value + (b[index] - value) * amount);

function safeEdge(from, to, isSafe, maxDelta) {
  const steps = Math.max(1, Math.ceil(maxJointDistance(from, to) / maxDelta));
  const samples = [];
  for (let step = 1; step <= steps; step += 1) {
    const q = interpolateJoints(from, to, step / steps);
    if (!isSafe(q)) return null;
    samples.push(q);
  }
  return samples;
}

/** Find a deterministic, safety-checked joint path for one arm. */
export function planSafeMotion(start, target, arm = ARM, safety = {}, otherPoses = [], salt = 0) {
  const isSafe = (q) => evaluateCellSafety([...otherPoses, { q, arm }], safety).safe;
  if (!isSafe(start) || !isSafe(target)) return null;
  const direct = safeEdge(start, target, isSafe, arm.maxActionDelta * 0.96);
  if (direct) return direct;

  // A compact deterministic RRT supplies a route around floor, table-edge,
  // and other-arm constraints when straight joint interpolation is unsafe.
  let randomState = (123456789 + salt * 2654435761) >>> 0;
  const random = () => {
    randomState = (1664525 * randomState + 1013904223) >>> 0;
    return randomState / 4294967296;
  };
  const nodes = [{ q: [...start], parent: -1 }];
  for (let iteration = 0; iteration < 15000; iteration += 1) {
    const sample = iteration % 7 === 0
      ? target
      : arm.lengths.map((_, index) => (random() * 2 - 1) * limitOf(arm, index));
    let nearest = 0;
    let nearestDistance = Infinity;
    for (let index = 0; index < nodes.length; index += 1) {
      const candidateDistance = jointDistance(nodes[index].q, sample);
      if (candidateDistance < nearestDistance) {
        nearest = index;
        nearestDistance = candidateDistance;
      }
    }
    const amount = Math.min(1, 0.35 / (nearestDistance || 1));
    const next = interpolateJoints(nodes[nearest].q, sample, amount);
    if (!safeEdge(nodes[nearest].q, next, isSafe, arm.maxActionDelta * 0.96)) continue;
    nodes.push({ q: next, parent: nearest });
    const finish = safeEdge(next, target, isSafe, arm.maxActionDelta * 0.96);
    if (!finish) continue;

    const waypoints = [];
    for (let index = nodes.length - 1; index > 0; index = nodes[index].parent) waypoints.push(nodes[index].q);
    waypoints.reverse();
    waypoints.push(target);
    const route = [start, ...waypoints];
    const smoothed = [start];
    for (let from = 0; from < route.length - 1;) {
      let to = route.length - 1;
      while (to > from + 1 && !safeEdge(route[from], route[to], isSafe, arm.maxActionDelta * 0.96)) to -= 1;
      smoothed.push(route[to]);
      from = to;
    }
    const path = [];
    let previous = start;
    for (const waypoint of smoothed.slice(1)) {
      path.push(...safeEdge(previous, waypoint, isSafe, arm.maxActionDelta * 0.96));
      previous = waypoint;
    }
    return path;
  }
  return null;
}

/** 3x6 tool-tip Jacobian by central differences, in scene pixels per radian. */
function tipJacobian(q, arm) {
  const h = 1e-4;
  const columns = q.map((_, joint) => {
    const plus = [...q];
    const minus = [...q];
    plus[joint] += h;
    minus[joint] -= h;
    const a = forwardKinematics(plus, arm).points.at(-1);
    const b = forwardKinematics(minus, arm).points.at(-1);
    return [0, 1, 2].map((axis) => (a[axis] - b[axis]) / (2 * h));
  });
  return [0, 1, 2].map((row) => columns.map((column) => column[row]));
}

const dot6 = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);

/** Solve the 3x3 system m x = v by Cramer's rule. */
function solve3(m, v) {
  const det = (a) => a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
    - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
    + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
  const d = det(m);
  return [0, 1, 2].map((column) => det(m.map((row, r) => row.map((value, c) => (c === column ? v[r] : value)))) / d);
}

/**
 * Move one arm's tool tip along straight lines through `waypoints`, one
 * rate-capped frame at a time: resolved-rate control with a damped
 * least-squares step (dq = Jᵀ (J Jᵀ + λ² I)⁻¹ dx) toward the next point on
 * the line. A joint-space route between the same endpoints can swing the tip
 * across half the table; a gripper holding cloth must not. Returns null if a
 * segment cannot be followed within tolerance or a frame fails safety, so the
 * caller can choose another route.
 */
export function planCartesianMotion(start, waypoints, arm = ARM, safety = {}, otherPoses = [], { stepPx = 2.5, tolerancePx = 1.5 } = {}) {
  const cap = arm.maxActionDelta * 0.96;
  const damping = 6;
  const isSafe = (q) => evaluateCellSafety([...otherPoses, { q, arm }], safety).safe;
  const tipOf = (q) => forwardKinematics(q, arm).points.at(-1);
  if (!isSafe(start)) return null;

  const frames = [];
  let q = [...start];
  // Converge on `target` in one frame if the cap allows, otherwise as far as it does.
  const advance = (target) => {
    const frameStart = [...q];
    let next = [...q];
    for (let iteration = 0; iteration < 16; iteration += 1) {
      const error = sub(target, tipOf(next));
      if (norm(error) < 0.2) break;
      const full = tipJacobian(next, arm);
      const low = next.map((value, joint) => Math.max(-limitOf(arm, joint), frameStart[joint] - cap));
      const high = next.map((value, joint) => Math.min(limitOf(arm, joint), frameStart[joint] + cap));
      // Joints already against a bound and pushed further into it are taken
      // out of the Jacobian, so the rest of the chain carries the motion.
      const active = next.map(() => true);
      let dq = null;
      for (let pass = 0; pass < 4; pass += 1) {
        const j = full.map((row) => row.map((value, joint) => (active[joint] ? value : 0)));
        const jjt = [0, 1, 2].map((r) => [0, 1, 2].map((c) => dot6(j[r], j[c]) + (r === c ? damping ** 2 : 0)));
        const y = solve3(jjt, error);
        dq = next.map((_, joint) => j[0][joint] * y[0] + j[1][joint] * y[1] + j[2][joint] * y[2]);
        // Null-space pull toward mid-range keeps redundant joints off their
        // limits without disturbing the tool tip.
        const centre = next.map((value, joint) => (active[joint] ? -0.08 * value / limitOf(arm, joint) : 0));
        const jc = [0, 1, 2].map((r) => dot6(j[r], centre));
        const yc = solve3(jjt, jc);
        dq = dq.map((value, joint) => value + centre[joint] - (j[0][joint] * yc[0] + j[1][joint] * yc[1] + j[2][joint] * yc[2]));
        let blocked = false;
        next.forEach((value, joint) => {
          if (!active[joint]) return;
          if ((value <= low[joint] + 1e-9 && dq[joint] < 0) || (value >= high[joint] - 1e-9 && dq[joint] > 0)) {
            active[joint] = false;
            blocked = true;
          }
        });
        if (!blocked) break;
      }
      next = next.map((value, joint) => clamp(value + dq[joint], low[joint], high[joint]));
    }
    return next;
  };

  for (const waypoint of waypoints) {
    const from = tipOf(q);
    const samples = Math.max(1, Math.ceil(distance(from, waypoint) / stepPx));
    for (let s = 1; s <= samples; s += 1) {
      const t = s / samples;
      const target = [0, 1, 2].map((axis) => from[axis] + (waypoint[axis] - from[axis]) * t);
      // A capped frame may fall short; spend extra frames catching up.
      for (let attempt = 0; attempt < 8; attempt += 1) {
        q = advance(target);
        if (!isSafe(q)) return null;
        frames.push([...q]);
        if (distance(tipOf(q), target) < tolerancePx) break;
      }
      if (distance(tipOf(q), target) >= tolerancePx) return null;
    }
  }
  return frames;
}

/**
 * Plan a whole cell by moving one arm at a time and trying both bimanual
 * orders. Every returned frame has already passed the shared safety check.
 */
export function planSafeCellMotion(poses, targets, safety = {}) {
  const orders = poses.length === 2 ? [[0, 1], [1, 0]] : [poses.map((_, index) => index)];
  let best = null;
  for (const order of orders) {
    const working = poses.map(({ q, arm }) => ({ q: [...q], arm }));
    const frames = [];
    let failed = false;
    for (const moving of order) {
      const otherPoses = working.filter((_, index) => index !== moving);
      const path = planSafeMotion(working[moving].q, targets[moving], working[moving].arm, safety, otherPoses, moving + order[0] * 9);
      if (!path) { failed = true; break; }
      for (const q of path) {
        working[moving].q = [...q];
        frames.push(working.map((pose) => [...pose.q]));
      }
    }
    if (!failed && (!best || frames.length < best.frames.length)) best = { frames, order };
  }
  return best;
}

/**
 * Second-stage, collision-checked path reduction for a cell trajectory.
 *
 * The first planner deliberately favors finding a safe path. This reducer
 * reruns the safety test over longer candidate edges and keeps a shortcut
 * only when every resampled frame obeys the same joint-delta, floor,
 * workspace, and inter-arm rules. `requiredFrameIndexes` preserve semantic
 * stages such as a towel grasp, lift, and placement.
 */
export function reduceSafeCellMotion(poses, frames, safety = {}, { requiredFrameIndexes = [], keepTailFrames = 0 } = {}) {
  if (!frames?.length) return { frames: [], sourceFrames: 0, reducedBy: 0 };
  const arms = poses.map(({ arm = ARM }) => arm);
  const validFrame = (frame) => evaluateCellSafety(frame.map((q, index) => ({ q, arm: arms[index] })), safety).safe;
  const safeCellEdge = (from, to) => {
    const steps = Math.max(1, ...to.map((q, armIndex) => Math.ceil(maxJointDistance(from[armIndex], q) / (arms[armIndex].maxActionDelta * 0.96))));
    const samples = [];
    for (let step = 1; step <= steps; step += 1) {
      const frame = to.map((q, armIndex) => interpolateJoints(from[armIndex], q, step / steps));
      if (!validFrame(frame)) return null;
      samples.push(frame);
    }
    return samples;
  };

  const stops = new Set([frames.length - 1]);
  requiredFrameIndexes.forEach((index) => { if (index >= 0 && index < frames.length) stops.add(index); });
  for (let index = Math.max(0, frames.length - keepTailFrames); index < frames.length; index += 1) stops.add(index);
  const endpoints = [...stops].sort((a, b) => a - b);
  const reduced = [];
  let sourceStart = -1;
  let current = poses.map(({ q }) => [...q]);

  for (const endpoint of endpoints) {
    let cursor = sourceStart;
    while (cursor < endpoint) {
      let accepted = null;
      let acceptedIndex = cursor + 1;
      for (let candidate = endpoint; candidate > cursor; candidate -= 1) {
        const shortcut = safeCellEdge(current, frames[candidate]);
        if (!shortcut) continue;
        accepted = shortcut;
        acceptedIndex = candidate;
        break;
      }
      // The original adjacent frame is guaranteed to be a valid fallback for
      // planner output; keep it defensively if a caller supplied bad input.
      if (!accepted) {
        const fallback = frames[cursor + 1];
        if (!validFrame(fallback)) return null;
        accepted = [fallback.map((q) => [...q])];
      }
      reduced.push(...accepted.map((frame) => frame.map((q) => [...q])));
      current = reduced.at(-1);
      cursor = acceptedIndex;
    }
    sourceStart = endpoint;
  }
  return { frames: reduced, sourceFrames: frames.length, reducedBy: frames.length - reduced.length };
}

// Keyed by corner pair + timing profile: a Task 8 run with different
// corners (or heights) must not reuse a path planned for Task 7's, and vice
// versa - the frames encode where the towel actually is.
const cachedFoldPaths = new Map();

/**
 * Specialized bimanual motion planner for the laundry folding demo (Task 07).
 *
 * Coordinates both arms through the teachable folding sequence:
 *  1. Both arms approach above their towel corners (free space, joint route)
 *  2. Each tool descends straight onto its corner and closes
 *  3. Arm B lifts its corner along an arc over the fold line while A pins
 *  4. Arm A opens, rises straight off the towel and returns home
 *  5. Arm B completes the arc and places its corner on A's
 *  6. Holds the fold while the cloth settles
 *
 * Every stage that carries cloth is a straight-line tool path
 * (planCartesianMotion), and `grips` gives the gripper command per frame.
 * Every frame is validated against floor, workspace, and inter-arm clearance limits.
 */
export function planTowelFoldMotion(poses, safety = {}, profile = {}) {
  if (poses.length < 2) return null;
  const armA = poses[0].arm || ARM;
  const armB = poses[1].arm || ARM_B;

  // Towel corners in scene pixels - the cloth sim's front corners by
  // default, or any other pair a caller passes in profile.cornerA/cornerB
  // (see clothCorners() in cloth.js for the four valid grid corners) to fold
  // along a different edge. The heights the grippers work at follow below.
  const cornerA = profile.cornerA ?? [250, 180];
  const cornerB = profile.cornerB ?? [400, 180];
  const hoverZ = profile.hoverZ ?? 40;
  const graspZ = profile.graspZ ?? 4;
  const arcHeight = profile.arcHeight ?? 48;
  const placeHeight = profile.placeHeight ?? 6;
  const settleFrames = profile.settleFrames ?? 50;
  const dwellFrames = 3;

  // Solve at the grasp itself, where the solver can pick a posture that is
  // safe with the tool down on the table; each descent is then the reverse of
  // a straight rise from that posture, so it ends exactly on the corner.
  const graspA = solveInverseKinematics([...cornerA, graspZ], armA, safety);
  const graspB = solveInverseKinematics([...cornerB, graspZ], armB, safety, [{ q: graspA.q, arm: armA }]);
  if (!graspA?.safety?.safe || !graspB?.safety?.safe) return null;
  const riseB = planCartesianMotion(graspB.q, [[...cornerB, hoverZ]], armB, safety, [{ q: graspA.q, arm: armA }]);
  const riseA = riseB && planCartesianMotion(graspA.q, [[...cornerA, hoverZ]], armA, safety, [{ q: riseB.at(-1), arm: armB }]);
  if (!riseA) return null;
  const planA = { ...graspA, q: riseA.at(-1) };
  const planB = { ...graspB, q: riseB.at(-1) };

  const isHomeA = jointDistance(poses[0].q, HOME_POSE) < 1e-3;
  const isHomeB = jointDistance(poses[1].q, HOME_POSE) < 1e-3;
  const cacheKey = JSON.stringify([cornerA, cornerB, hoverZ, graspZ, arcHeight, placeHeight, settleFrames]);
  const cachedFoldPath = cachedFoldPaths.get(cacheKey);
  if (isHomeA && isHomeB && cachedFoldPath) {
    return { ...cachedFoldPath, frames: cachedFoldPath.frames.map((f) => [[...f[0]], [...f[1]]]), grips: cachedFoldPath.grips.map((g) => [...g]), planA, planB };
  }

  const frames = [];
  const grips = [];
  const stageEnds = [];
  let curA = poses[0].q;
  let curB = poses[1].q;
  let grip = [false, false];
  const push = (qA, qB) => {
    curA = qA;
    curB = qB;
    frames.push([[...qA], [...qB]]);
    grips.push([...grip]);
  };
  const dwell = (count) => { for (let i = 0; i < count; i += 1) push(curA, curB); };
  const endStage = () => stageEnds.push(frames.length - 1);

  // 1. Approach: free space, no cloth held, so any safe joint route will do.
  //    It ends above the corners so no sweep passes through a grasp.
  const approach = planSafeCellMotion(poses, [planA.q, planB.q], safety);
  if (!approach) return null;
  for (const [qA, qB] of approach.frames) push(qA, qB);
  endStage();

  // 2. Descend straight onto each corner and close: the grasp lands on the
  //    corner instead of snapping it across from wherever the tool passed.
  const descend = (rise, graspQ) => [...rise.slice(0, -1).reverse(), graspQ];
  for (const q of descend(riseA, graspA.q)) push(q, curB);
  for (const q of descend(riseB, graspB.q)) push(curA, q);
  grip = [true, true];
  dwell(dwellFrames);
  endStage();

  // 3. Lift B's corner along an arc over the fold line while A pins its own.
  //    Every carried segment is a straight tool line (planCartesianMotion).
  //    The arc travels along the line from B's corner to A's - horizontal for
  //    Task 7's left/right pair, but any direction for a differently chosen
  //    corner pair - climbing to arcHeight at its midpoint (angle = PI/2).
  const foldMid = [(cornerA[0] + cornerB[0]) / 2, (cornerA[1] + cornerB[1]) / 2];
  const foldVector = [cornerA[0] - cornerB[0], cornerA[1] - cornerB[1]];
  const radius = Math.hypot(foldVector[0], foldVector[1]) / 2;
  const foldUnit = radius > 1e-6 ? [foldVector[0] / (2 * radius), foldVector[1] / (2 * radius)] : [1, 0];
  const arcPoint = (angle) => [
    foldMid[0] - foldUnit[0] * radius * Math.cos(angle),
    foldMid[1] - foldUnit[1] * radius * Math.cos(angle),
    graspZ + arcHeight * Math.sin(angle),
  ];
  const liftArc = [1, 2, 3, 4].map((i) => arcPoint((i / 4) * (Math.PI / 2)));
  const lift = planCartesianMotion(curB, liftArc, armB, safety, [{ q: curA, arm: armA }]);
  if (!lift) return null;
  for (const q of lift) push(curA, q);
  endStage();

  // 4. A lets go, rises straight off the towel, then clears the fold side so
  //    B can lay the corner where A was holding.
  grip = [false, true];
  dwell(dwellFrames);
  const rise = planCartesianMotion(curA, [[...cornerA, hoverZ]], armA, safety, [{ q: curB, arm: armB }]);
  if (!rise) return null;
  for (const q of rise) push(q, curB);
  const retract = planSafeMotion(curA, [...HOME_POSE], armA, safety, [{ q: curB, arm: armB }]);
  if (!retract) return null;
  for (const q of retract) push(q, curB);
  endStage();

  // 5. Finish the arc and place the corner on A's, just above the lower
  //    layer; the cloth's thickness barrier supplies the final separation.
  const foldArc = [5, 6, 7].map((i) => arcPoint((i / 8) * Math.PI));
  const place = planCartesianMotion(curB, [...foldArc, [...cornerA, placeHeight]], armB, safety, [{ q: curA, arm: armA }]);
  if (!place) return null;
  for (const q of place) push(curA, q);
  endStage();

  // 6. Hold while the towel settles into its folded rest state.
  dwell(settleFrames);

  // Unlike free-space reaches, the fold phases drive a dynamic cloth model.
  // Keep their full rate-capped samples: reducing them makes the grasped edge
  // move too abruptly and stretches the simulated fabric. Generic policies
  // still receive the second-stage reducer in startPolicy().
  const result = { frames, grips, sourceFrames: frames.length, reducedBy: 0, stageEnds, planA, planB };
  if (isHomeA && isHomeB) cachedFoldPaths.set(cacheKey, { ...result, frames: frames.map((f) => [[...f[0]], [...f[1]]]), grips: grips.map((g) => [...g]) });
  return result;
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

/**
 * Bundle several recorded episodes (each already an armlab-episode-preview
 * artifact from buildEpisodeArtifact) into one LeRobotDataset-shaped
 * manifest: the info/tasks/episodes/frame-table structure LeRobot's
 * `datasets` loader expects, short of the parquet/video encoding a static
 * browser page cannot produce. scripts/lerobot_export.py turns this JSON
 * into an actual on-disk LeRobotDataset directory.
 *
 * Every episode must share one arm count: LeRobotDataset expects a single
 * fixed observation/action shape per dataset, and this project's single-arm
 * (22/7) and bimanual (44/14) episodes are not the same shape.
 */
export function buildDatasetManifest({ episodes = [], fps = 25 } = {}) {
  if (!episodes.length) return { error: 'empty', message: 'Add at least one recorded episode before downloading a dataset.' };
  const arms = episodes[0].arms;
  const mismatched = episodes.find((episode) => episode.arms !== arms);
  if (mismatched) {
    return {
      error: 'mixed-arms',
      message: `Every episode in one dataset must use the same arm count (found ${arms} and ${mismatched.arms}). Download or clear this dataset before recording a different task type.`,
    };
  }

  const firstTransition = episodes.flatMap((episode) => episode.transitions).find(Boolean);
  const stateDim = firstTransition?.observation.length ?? 0;
  const actionDim = firstTransition?.action_after_safety_clamp.length ?? 0;

  const taskIndexOf = new Map();
  const tasks = [];
  const frames = [];
  const episodeRecords = [];
  let globalIndex = 0;

  episodes.forEach((episode, episodeIndex) => {
    const taskText = episode.task?.instruction || episode.task?.id || 'unspecified task';
    if (!taskIndexOf.has(taskText)) taskIndexOf.set(taskText, tasks.push({ task_index: tasks.length, task: taskText }) - 1);
    const taskIndex = taskIndexOf.get(taskText);

    episode.transitions.forEach((transition, frameIndex) => {
      const isLastFrame = frameIndex === episode.transitions.length - 1;
      frames.push({
        episode_index: episodeIndex,
        frame_index: frameIndex,
        index: globalIndex,
        timestamp: transition.timestamp_ms / 1000,
        'observation.state': transition.observation,
        action: transition.action_after_safety_clamp,
        'next.done': isLastFrame,
        'next.success': isLastFrame && episode.halt === HALT.REACHED,
        task_index: taskIndex,
      });
      globalIndex += 1;
    });

    episodeRecords.push({
      episode_index: episodeIndex,
      task_index: taskIndex,
      length: episode.transitions.length,
      halt: episode.halt,
    });
  });

  return {
    info: {
      codebase_version: 'armlab-dataset-preview/v1',
      robot_type: arms === 2 ? 'armlab-6dof-bimanual' : 'armlab-6dof',
      fps,
      total_episodes: episodes.length,
      total_frames: frames.length,
      total_tasks: tasks.length,
      features: {
        'observation.state': { dtype: 'float32', shape: [stateDim] },
        action: { dtype: 'float32', shape: [actionDim] },
        'next.done': { dtype: 'bool', shape: [1] },
        'next.success': { dtype: 'bool', shape: [1] },
        timestamp: { dtype: 'float32', shape: [1] },
        episode_index: { dtype: 'int64', shape: [1] },
        frame_index: { dtype: 'int64', shape: [1] },
        index: { dtype: 'int64', shape: [1] },
        task_index: { dtype: 'int64', shape: [1] },
      },
      source: 'browser-simulation',
    },
    tasks,
    episodes: episodeRecords,
    frames,
  };
}

/** Calculate 0-100 progress percentage for demo policy execution. */
export function computePolicyProgress(steps, totalSteps) {
  if (!totalSteps || totalSteps <= 0) return 0;
  const clampedSteps = Math.max(0, steps || 0);
  return Math.min(100, Math.round((clampedSteps / totalSteps) * 100));
}

/**
 * Format a live solver progress ticker: step count, percent, and elapsed
 * wall time. Every task runs the same demo planner (planSafeCellMotion, or
 * planTowelFoldMotion for the bimanual fold), so this is task-agnostic by
 * construction - there is nothing fold-specific to gate it behind.
 */
export function formatSolverTicker({ steps = 0, totalSteps = 0, elapsedMs = 0 } = {}) {
  const safeTotal = Math.max(0, Math.round(totalSteps || 0));
  if (safeTotal === 0) return 'Solver idle';
  const safeSteps = clamp(Math.round(steps || 0), 0, safeTotal);
  const pct = computePolicyProgress(safeSteps, safeTotal);
  const seconds = Math.max(0, elapsedMs || 0) / 1000;
  return `Solving · step ${safeSteps}/${safeTotal} · ${pct}% · ${seconds.toFixed(1)}s`;
}
