/**
 * Tool-down motion planning for rigid-object pick and place.
 *
 * core.js's solver and straight-line planner control only where the tool tip
 * is, not which way the tool points, so an arm can arrive at an object with
 * its fingers lying along the table. That is fine for chasing a marker or
 * pinching a towel corner, but a gripper that has to straddle a cube needs
 * its fingers vertical. Everything here adds that one constraint - the
 * tool's forward axis points straight down - on top of the same position
 * targets, rate cap and cell-safety check the rest of the lab uses. Yaw about
 * the vertical is left free, so the 6-joint arm keeps one redundant degree
 * of freedom for staying inside its limits.
 */
import { ARM, clamp, distance, evaluateCellSafety, forwardKinematics, HOME_POSE, limitOf, planSafeMotion } from './core.js';

/** Weight on the two orientation rows, in scene pixels per unit of tilt. */
const TILT_WEIGHT = 120;
/** A tool within this tilt of vertical (sin of the angle) counts as pointing down. */
export const TOOL_DOWN_TOLERANCE = 0.03;

const tipOf = (q, arm) => forwardKinematics(q, arm).points.at(-1);

/** Residual of the tool-down pose task: tip position error, then weighted tilt. */
function residual(q, arm, target) {
  const { points, forward } = forwardKinematics(q, arm);
  const tip = points.at(-1);
  return [target[0] - tip[0], target[1] - tip[1], target[2] - tip[2], -forward[0] * TILT_WEIGHT, -forward[1] * TILT_WEIGHT];
}

/** 5x6 Jacobian of (tip xyz, weighted forward xy) by central differences. */
function jacobian(q, arm) {
  const h = 1e-4;
  const rows = [[], [], [], [], []];
  for (let joint = 0; joint < q.length; joint += 1) {
    const plus = [...q];
    const minus = [...q];
    plus[joint] += h;
    minus[joint] -= h;
    const a = forwardKinematics(plus, arm);
    const b = forwardKinematics(minus, arm);
    const ta = a.points.at(-1);
    const tb = b.points.at(-1);
    for (let axis = 0; axis < 3; axis += 1) rows[axis].push((ta[axis] - tb[axis]) / (2 * h));
    rows[3].push(((a.forward[0] - b.forward[0]) / (2 * h)) * TILT_WEIGHT);
    rows[4].push(((a.forward[1] - b.forward[1]) / (2 * h)) * TILT_WEIGHT);
  }
  return rows;
}

/** Solve a small dense system m x = v by Gaussian elimination with partial pivoting. */
function solveLinear(m, v) {
  const n = v.length;
  const a = m.map((row, r) => [...row, v[r]]);
  for (let col = 0; col < n; col += 1) {
    let pivot = col;
    for (let r = col + 1; r < n; r += 1) if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
    [a[col], a[pivot]] = [a[pivot], a[col]];
    const d = a[col][col] || 1e-12;
    for (let r = col + 1; r < n; r += 1) {
      const f = a[r][col] / d;
      for (let c = col; c <= n; c += 1) a[r][c] -= f * a[col][c];
    }
  }
  const x = Array(n).fill(0);
  for (let r = n - 1; r >= 0; r -= 1) {
    let sum = a[r][n];
    for (let c = r + 1; c < n; c += 1) sum -= a[r][c] * x[c];
    x[r] = sum / (a[r][r] || 1e-12);
  }
  return x;
}

/**
 * One damped least-squares step toward the tool-down pose at `target`, with
 * every joint held inside [low, high]. Joints pinned against a bound are
 * dropped from the Jacobian so the rest of the chain carries the motion, as
 * in core.js's planCartesianMotion, and a gentle null-space pull toward
 * mid-range keeps the redundant yaw off its limits.
 */
function dlsStep(q, arm, target, low, high, damping) {
  const error = residual(q, arm, target);
  const full = jacobian(q, arm);
  const active = q.map(() => true);
  let dq = null;
  for (let pass = 0; pass < 4; pass += 1) {
    const j = full.map((row) => row.map((value, joint) => (active[joint] ? value : 0)));
    const jjt = j.map((ri) => j.map((rj) => ri.reduce((sum, value, k) => sum + value * rj[k], 0)));
    jjt.forEach((row, r) => { row[r] += damping ** 2; });
    const y = solveLinear(jjt, error);
    dq = q.map((_, joint) => j.reduce((sum, row, r) => sum + row[joint] * y[r], 0));
    const centre = q.map((value, joint) => (active[joint] ? -0.05 * value / limitOf(arm, joint) : 0));
    const yc = solveLinear(jjt, j.map((row) => row.reduce((sum, value, k) => sum + value * centre[k], 0)));
    dq = dq.map((value, joint) => value + centre[joint] - j.reduce((sum, row, r) => sum + row[joint] * yc[r], 0));
    let blocked = false;
    q.forEach((value, joint) => {
      if (!active[joint]) return;
      if ((value <= low[joint] + 1e-9 && dq[joint] < 0) || (value >= high[joint] - 1e-9 && dq[joint] > 0)) {
        active[joint] = false;
        blocked = true;
      }
    });
    if (!blocked) break;
  }
  return q.map((value, joint) => clamp(value + dq[joint], low[joint], high[joint]));
}

/** Position and tilt error of `q` against the tool-down pose at `target`. */
export function toolDownError(q, arm, target) {
  const { points, forward } = forwardKinematics(q, arm);
  return { position: distance(points.at(-1), target), tilt: Math.hypot(forward[0], forward[1]), down: forward[2] < 0 };
}

const TOOL_DOWN_SEEDS = [
  [...HOME_POSE],
  [0, 0.4, 1.2, 0, 1.4, 0],
  [0.5, 0.4, 1.2, 0, 1.4, 0],
  [-0.5, 0.4, 1.2, 0, 1.4, 0],
  [0, -0.3, 1.5, 0, 1.2, 0],
  [0.8, 0.2, 1.4, 0, 1.4, 0],
  [-0.8, 0.2, 1.4, 0, 1.4, 0],
  [0, 0.8, 0.8, 0, 1.4, 0],
  [0, 0.6, -1.1, 0, -0.6, 0],
  [0.3, 1.0, 1.0, -0.5, 1.2, 0.4],
  [-0.3, 1.0, 1.0, 0.5, 1.2, -0.4],
];
// Low-discrepancy restarts, as in core.js's solver, reach the elbow and
// wrist branches the hand-picked seeds above miss. Deterministic, so a task
// always gets the same plan.
const PHASES = [0.61803398875, 0.41421356237, 0.73205080757, 0.2360679775, 0.64575131106, 0.31662479036];
for (let restart = 1; restart <= 16; restart += 1) {
  TOOL_DOWN_SEEDS.push(PHASES.map((phase, joint) => (((restart * phase) % 1) * 2 - 1) * (joint === 0 ? Math.PI : 1.7)));
}

/**
 * Solve for a safe pose with the tool tip at `target` and the tool pointing
 * straight down. Deterministic multi-start; among the safe converged
 * solutions it returns the one nearest `prefer` in joint space, so
 * consecutive grasp and place poses stay on the same elbow branch. Returns
 * null when no seed converges to a safe pose.
 */
export function solveToolDownIK(target, arm = ARM, safety = {}, otherPoses = [], { prefer = HOME_POSE } = {}) {
  const lowLimit = arm.lengths.map((_, joint) => -limitOf(arm, joint));
  const highLimit = arm.lengths.map((_, joint) => limitOf(arm, joint));
  let best = null;
  for (const seed of [prefer, ...TOOL_DOWN_SEEDS]) {
    let q = seed.map((value, joint) => clamp(value, lowLimit[joint], highLimit[joint]));
    for (let iteration = 0; iteration < 160; iteration += 1) {
      const low = q.map((value, joint) => Math.max(lowLimit[joint], value - 0.25));
      const high = q.map((value, joint) => Math.min(highLimit[joint], value + 0.25));
      q = dlsStep(q, arm, target, low, high, 4);
      const err = toolDownError(q, arm, target);
      if (err.position < 0.3 && err.tilt < TOOL_DOWN_TOLERANCE * 0.3) break;
    }
    const err = toolDownError(q, arm, target);
    if (err.position > 0.5 || err.tilt > TOOL_DOWN_TOLERANCE || !err.down) continue;
    if (!evaluateCellSafety([...otherPoses, { q, arm }], safety).safe) continue;
    const cost = Math.hypot(...q.map((value, joint) => value - prefer[joint]));
    if (!best || cost < best.cost) best = { q, cost };
  }
  return best ? best.q : null;
}

/**
 * Carry the tool tip along straight lines through `waypoints` with the tool
 * pointing down the whole way, one rate-capped frame at a time. The start
 * pose must already be tool-down. Returns the frames, or null if any segment
 * cannot be tracked within tolerance or any frame fails the cell-safety
 * check - a carried object must never be swung sideways or tipped.
 */
export function planToolDownMotion(start, waypoints, arm = ARM, safety = {}, otherPoses = [], { stepPx = 2.5, tolerancePx = 1.5 } = {}) {
  const cap = arm.maxActionDelta * 0.96;
  const isSafe = (q) => evaluateCellSafety([...otherPoses, { q, arm }], safety).safe;
  if (!isSafe(start)) return null;
  const frames = [];
  let q = [...start];
  const advance = (target) => {
    const low = q.map((value, joint) => Math.max(-limitOf(arm, joint), value - cap));
    const high = q.map((value, joint) => Math.min(limitOf(arm, joint), value + cap));
    let next = [...q];
    for (let iteration = 0; iteration < 12; iteration += 1) {
      const err = toolDownError(next, arm, target);
      if (err.position < 0.2 && err.tilt < TOOL_DOWN_TOLERANCE * 0.2) break;
      next = dlsStep(next, arm, target, low, high, 6);
    }
    return next;
  };
  for (const waypoint of waypoints) {
    const from = tipOf(q, arm);
    const samples = Math.max(1, Math.ceil(distance(from, waypoint) / stepPx));
    for (let s = 1; s <= samples; s += 1) {
      const t = s / samples;
      const target = [0, 1, 2].map((axis) => from[axis] + (waypoint[axis] - from[axis]) * t);
      let err = null;
      for (let attempt = 0; attempt < 8; attempt += 1) {
        q = advance(target);
        if (!isSafe(q)) return null;
        frames.push([...q]);
        err = toolDownError(q, arm, target);
        if (err.position < tolerancePx && err.tilt < TOOL_DOWN_TOLERANCE) break;
      }
      if (err.position >= tolerancePx || err.tilt >= TOOL_DOWN_TOLERANCE) return null;
    }
  }
  return frames;
}

/** Frames of a straight segment walked backwards: from its last frame back to `from`. */
const reverseSegment = (from, frames) => [...frames.slice(0, -1).reverse(), from];

/**
 * The straight tool-down chain grasp -> above grasp -> above release ->
 * release, as four frame lists. Every segment is a straight line, so a chain
 * planned from the release end is equally valid walked forwards; which end
 * it is solved from decides which elbow branch it runs on, and a branch
 * that is comfortable at one end can run into a joint limit at the other.
 * Returns { grasp, lift, carry, lower } (lift/carry/lower each end at their
 * segment's goal), or null.
 */
function toolDownChain(pick, place, hoverZ, arm, safety) {
  const above = (point) => [point[0], point[1], hoverZ];
  const forward = () => {
    const grasp = solveToolDownIK(pick, arm, safety);
    const lift = grasp && planToolDownMotion(grasp, [above(pick)], arm, safety);
    const carry = lift && planToolDownMotion(lift.at(-1), [above(place)], arm, safety);
    const lower = carry && planToolDownMotion(carry.at(-1), [place], arm, safety);
    return lower ? { grasp, lift, carry, lower } : null;
  };
  const backward = () => {
    const release = solveToolDownIK(place, arm, safety);
    const rise = release && planToolDownMotion(release, [above(place)], arm, safety);
    const back = rise && planToolDownMotion(rise.at(-1), [above(pick)], arm, safety);
    const descend = back && planToolDownMotion(back.at(-1), [pick], arm, safety);
    if (!descend) return null;
    return {
      grasp: descend.at(-1),
      lift: reverseSegment(back.at(-1), descend),
      carry: reverseSegment(rise.at(-1), back),
      lower: reverseSegment(release, rise),
    };
  };
  return forward() || backward();
}

/**
 * Plan a single-arm pick and place with a top-down grasp.
 *
 *  1. Free-space approach to a tool-down pose above the object
 *  2. Straight descent around the object, fingers open, then close
 *  3. Straight lift to the carry height
 *  4. Straight carry at that height to above the place point
 *  5. Straight descent to the release height, open
 *  6. Straight rise clear of the object, then a free-space return home
 *  7. Hold while the released object settles
 *
 * `pick` and `place` are scene points [x, y, z] for the tool tip at grasp
 * and release. Returns { frames, grips, stageEnds } with one gripper command
 * per frame (true = closed), or null if any stage fails safety. Every frame
 * is a one-arm cell frame ([[q]]) so main.js's generic policy stepper can
 * run it like any other plan.
 */
export function planPickPlaceMotion(pose, { pick, place, hoverZ = 90, settleFrames = 45, dwellFrames = 4 }, safety = {}) {
  const arm = pose.arm || ARM;
  const chain = toolDownChain(pick, place, hoverZ, arm, safety);
  if (!chain) return null;
  const hover = chain.lift.at(-1);

  const frames = [];
  const grips = [];
  const stageEnds = [];
  let grip = false;
  let q = pose.q;
  const push = (next) => { q = next; frames.push([[...next]]); grips.push([grip]); };
  const pushAll = (list) => list.forEach(push);
  const dwell = (count) => { for (let i = 0; i < count; i += 1) push(q); };
  const endStage = () => stageEnds.push(frames.length - 1);

  // 1. Approach in joint space. It ends high above the object and the
  //    objects all sit well below the home pose's tool, so the swing clears them.
  const approach = planSafeMotion(q, hover, arm, safety);
  if (!approach) return null;
  pushAll(approach);
  endStage();

  // 2. Descend around the object (the lift walked backwards) and close on it.
  pushAll(reverseSegment(chain.grasp, chain.lift));
  grip = true;
  dwell(dwellFrames);
  endStage();

  // 3-4. Lift, then carry level to above the place point.
  pushAll(chain.lift);
  pushAll(chain.carry);
  endStage();

  // 5. Lower to the release height and let go.
  pushAll(chain.lower);
  dwell(dwellFrames);
  grip = false;
  dwell(dwellFrames);
  endStage();

  // 6. Rise straight off the object (the lowering walked backwards) before
  //    any joint-space move, then return home.
  pushAll(reverseSegment(chain.carry.at(-1), chain.lower));
  const home = planSafeMotion(q, [...HOME_POSE], arm, safety);
  if (home) pushAll(home);
  endStage();

  // 7. Let the released object come to rest before the run is scored.
  dwell(settleFrames);
  return { frames, grips, stageEnds, sourceFrames: frames.length, reducedBy: 0 };
}
