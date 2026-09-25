/**
 * Boundary-aware straight-line tool tracking.
 *
 * core.js's planCartesianMotion drives the tool tip along a line and simply
 * gives up the moment a frame fails the cell-safety check. Near the arm's
 * own base - e.g. a towel's back corners, which sit just in front of the
 * columns - that happens even though every point on the line has a safe
 * pose: the tracker only controls the tip, so the elbow or wrist drifts over
 * the table edge on the way. The arm has six joints for a three-number tip
 * task, which leaves three to spare; this tracker spends them keeping every
 * link point a margin inside the floor, the table edge and the other arm,
 * by descending a clearance penalty in the tip task's null space. The tip
 * motion and every safety rule are unchanged.
 */
import { ARM, clamp, distance, evaluateCellSafety, forwardKinematics, limitOf } from './core.js';

/** Clearance (scene px) below which a link point is pushed back. */
const MARGIN = 18;
/** Distance (rad) from a joint limit below which the joint is eased back, and its weight in px. */
const LIMIT_MARGIN = 0.3;
const LIMIT_WEIGHT = 60;

const tipOf = (q, arm) => forwardKinematics(q, arm).points.at(-1);

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

/** 3x6 tip Jacobian by central differences. */
function jacobian(q, arm) {
  const h = 1e-4;
  const rows = [[], [], []];
  for (let joint = 0; joint < q.length; joint += 1) {
    const plus = [...q];
    const minus = [...q];
    plus[joint] += h;
    minus[joint] -= h;
    const a = tipOf(plus, arm);
    const b = tipOf(minus, arm);
    for (let axis = 0; axis < 3; axis += 1) rows[axis].push((a[axis] - b[axis]) / (2 * h));
  }
  return rows;
}

/**
 * Squared shortfall of every clearance below MARGIN beyond its limit: link
 * points to the floor and table edge, (with other arms present) the nearest
 * link-to-link distance to arm_clearance_px, and each joint's distance to
 * its travel limit - a joint parked on its limit can no longer help the
 * tip follow the line. Zero when comfortably clear of all of them.
 */
function penalty(q, arm, safety, otherPoses) {
  const result = evaluateCellSafety([...otherPoses, { q, arm }], safety);
  const shortfall = (clearance) => Math.max(0, MARGIN - clearance) ** 2;
  let total = shortfall(result.floorClearance) + shortfall(result.boundaryClearance);
  if (otherPoses.length) total += shortfall(result.armClearance - (safety.arm_clearance_px ?? 0));
  q.forEach((value, joint) => {
    total += (Math.max(0, LIMIT_MARGIN - (limitOf(arm, joint) - Math.abs(value))) * LIMIT_WEIGHT) ** 2;
  });
  return total;
}

/**
 * One tracking update toward `target`: a damped least-squares tip step,
 * plus the clearance-penalty gradient projected into the tip Jacobian's
 * null space so it cannot move the tip, all inside [low, high].
 */
function trackStep(q, arm, target, low, high, safety, otherPoses) {
  const j = jacobian(q, arm);
  const jjt = j.map((ri) => j.map((rj) => ri.reduce((sum, value, k) => sum + value * rj[k], 0)));
  jjt.forEach((row, r) => { row[r] += 36; });
  const tip = tipOf(q, arm);
  const error = [0, 1, 2].map((axis) => target[axis] - tip[axis]);
  const y = solveLinear(jjt, error);
  const task = q.map((_, joint) => j.reduce((sum, row, r) => sum + row[joint] * y[r], 0));

  const base = penalty(q, arm, safety, otherPoses);
  let away = q.map(() => 0);
  if (base > 0) {
    const h = 1e-3;
    const gradient = q.map((_, joint) => {
      const plus = [...q];
      plus[joint] += h;
      return (penalty(plus, arm, safety, otherPoses) - base) / h;
    });
    const scale = 0.02 / (Math.hypot(...gradient) || 1);
    const descent = gradient.map((value) => -value * scale);
    const yn = solveLinear(jjt, j.map((row) => row.reduce((sum, value, k) => sum + value * descent[k], 0)));
    away = descent.map((value, joint) => value - j.reduce((sum, row, r) => sum + row[joint] * yn[r], 0));
  }
  return q.map((value, joint) => clamp(value + task[joint] + away[joint], low[joint], high[joint]));
}

/**
 * Drive the tool tip along straight lines through `waypoints`, one
 * rate-capped frame at a time, keeping the links clear of every safety
 * boundary with the arm's spare joints. Same contract as core.js's
 * planCartesianMotion: returns the frames, or null if the tip cannot follow
 * a segment within `tolerancePx` or any frame fails the cell-safety check.
 */
export function planClearCartesianMotion(start, waypoints, arm = ARM, safety = {}, otherPoses = [], { stepPx = 2.5, tolerancePx = 1.5 } = {}) {
  const cap = arm.maxActionDelta * 0.96;
  const isSafe = (q) => evaluateCellSafety([...otherPoses, { q, arm }], safety).safe;
  if (!isSafe(start)) return null;
  const frames = [];
  let q = [...start];
  for (const waypoint of waypoints) {
    const from = tipOf(q, arm);
    const samples = Math.max(1, Math.ceil(distance(from, waypoint) / stepPx));
    for (let s = 1; s <= samples; s += 1) {
      const t = s / samples;
      const target = [0, 1, 2].map((axis) => from[axis] + (waypoint[axis] - from[axis]) * t);
      for (let attempt = 0; attempt < 8; attempt += 1) {
        const low = q.map((value, joint) => Math.max(-limitOf(arm, joint), value - cap));
        const high = q.map((value, joint) => Math.min(limitOf(arm, joint), value + cap));
        let next = [...q];
        for (let iteration = 0; iteration < 12; iteration += 1) {
          next = trackStep(next, arm, target, low, high, safety, otherPoses);
          if (distance(tipOf(next, arm), target) < 0.2 && penalty(next, arm, safety, otherPoses) === 0) break;
        }
        q = next;
        if (!isSafe(q)) return null;
        frames.push([...q]);
        if (distance(tipOf(q, arm), target) < tolerancePx) break;
      }
      if (distance(tipOf(q, arm), target) >= tolerancePx) return null;
    }
  }
  return frames;
}

// Deterministic restart postures for solveClearIK: a spread over the joint
// box (low-discrepancy, as in core.js's solver) plus the home pose.
const PHASES = [0.61803398875, 0.41421356237, 0.73205080757, 0.2360679775, 0.64575131106, 0.31662479036];
const CLEAR_SEEDS = [[0.662, 1.276, 1.496, -1.465, 1.488, -1.217]];
for (let restart = 1; restart <= 40; restart += 1) {
  CLEAR_SEEDS.push(PHASES.map((phase, joint) => (((restart * phase) % 1) * 2 - 1) * (joint === 0 ? Math.PI : 1.7)));
}

/**
 * Safe poses with the tip on `target`, best first: every seed is converged
 * with the same clearance-seeking step the tracker uses, and the survivors
 * are ranked by how much clearance and joint travel they keep. A pose that
 * starts with room to spare can follow a line out of a cramped spot - the
 * nearest-to-limit pose a plain IK returns there often cannot.
 */
export function solveClearIK(target, arm = ARM, safety = {}, otherPoses = [], { limit = 4 } = {}) {
  const lowLimit = arm.lengths.map((_, joint) => -limitOf(arm, joint));
  const highLimit = arm.lengths.map((_, joint) => limitOf(arm, joint));
  const found = [];
  for (const seed of CLEAR_SEEDS) {
    let q = seed.map((value, joint) => clamp(value, lowLimit[joint], highLimit[joint]));
    for (let iteration = 0; iteration < 150; iteration += 1) {
      const low = q.map((value, joint) => Math.max(lowLimit[joint], value - 0.2));
      const high = q.map((value, joint) => Math.min(highLimit[joint], value + 0.2));
      q = trackStep(q, arm, target, low, high, safety, otherPoses);
      // Converged on the tip; the remaining passes only buy clearance, so
      // stop once there is nothing left to buy.
      if (distance(tipOf(q, arm), target) < 0.3 && (iteration >= 60 || penalty(q, arm, safety, otherPoses) === 0)) break;
    }
    if (distance(tipOf(q, arm), target) > 0.5) continue;
    if (!evaluateCellSafety([...otherPoses, { q, arm }], safety).safe) continue;
    if (found.some((other) => Math.max(...other.q.map((value, joint) => Math.abs(value - q[joint]))) < 0.05)) continue;
    found.push({ q, cost: penalty(q, arm, safety, otherPoses) });
  }
  return found.sort((a, b) => a.cost - b.cost).slice(0, limit).map(({ q }) => q);
}
