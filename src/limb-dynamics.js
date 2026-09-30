/**
 * Higher-derivative weighted arm kinematics.
 *
 * A joint path that only respects a per-step rate cap can still change
 * velocity abruptly at every corner - an infinite jerk a real gearbox would
 * feel. Everything here weighs the third to sixth time derivatives of the
 * joint angles:
 *
 *   jerk    (d3q, "jolt")
 *   snap    (d4q, "jounce")
 *   crackle (d5q)
 *   pop     (d6q)
 *
 * smoothJointPath trades those off against staying on a pre-verified plan,
 * offline. LimbController does the same live, one control tick at a time,
 * for an operator dragging a single limb. Neither ever relaxes a physical
 * constraint: every frame they produce is re-checked by the caller's
 * validator, and a frame that fails is never emitted.
 */
import { clamp, forwardKinematics, limitOf } from './core.js';

/** Relative weight of each derivative; higher orders are weighted less. */
export const KINEMATIC_WEIGHTS = Object.freeze({ jerk: 1, snap: 0.5, crackle: 0.25, pop: 0.125 });
const ORDERS = Object.freeze({ jerk: 3, snap: 4, crackle: 5, pop: 6 });

/** Binomial finite-difference stencil of order k: sum_i c_i q[t+i] = d^k q (unit step). */
function stencil(order) {
  const c = [1];
  for (let k = 0; k < order; k += 1) {
    const next = Array(c.length + 1).fill(0);
    c.forEach((value, i) => { next[i] -= value; next[i + 1] += value; });
    c.splice(0, c.length, ...next);
  }
  return c;
}

/**
 * Peak and RMS magnitude of every weighted derivative of a joint path
 * (per control step), taken over all joints. Used to show a smoothed path
 * really is smoother, not just different.
 */
export function derivativeProfile(frames) {
  const profile = {};
  for (const [name, order] of Object.entries(ORDERS)) {
    const c = stencil(order);
    let peak = 0;
    let sum = 0;
    let count = 0;
    for (let t = 0; t + order < frames.length; t += 1) {
      for (let joint = 0; joint < frames[0].length; joint += 1) {
        const value = c.reduce((acc, coef, i) => acc + coef * frames[t + i][joint], 0);
        peak = Math.max(peak, Math.abs(value));
        sum += value * value;
        count += 1;
      }
    }
    profile[name] = { peak, rms: count ? Math.sqrt(sum / count) : 0 };
  }
  return profile;
}

/**
 * Solve the symmetric positive-definite banded system A x = b in place by
 * Cholesky. `band[i][d]` holds A[i][i + d] for d = 0..width.
 */
function solveBanded(band, b, width) {
  const n = b.length;
  const l = band.map((row) => [...row]);
  for (let i = 0; i < n; i += 1) {
    for (let d = 0; d <= width && i + d < n; d += 1) {
      let sum = l[i][d];
      for (let k = Math.max(0, i + d - width); k < i; k += 1) {
        if (i - k <= width && i + d - k <= width) sum -= l[k][i - k] * l[k][i + d - k];
      }
      if (d === 0) l[i][0] = Math.sqrt(Math.max(sum, 1e-12));
      else l[i][d] = sum / l[i][0];
    }
  }
  const y = [...b];
  for (let i = 0; i < n; i += 1) {
    for (let k = Math.max(0, i - width); k < i; k += 1) y[i] -= l[k][i - k] * y[k];
    y[i] /= l[i][0];
  }
  for (let i = n - 1; i >= 0; i -= 1) {
    for (let d = 1; d <= width && i + d < n; d += 1) y[i] -= l[i][d] * y[i + d];
    y[i] /= l[i][0];
  }
  return y;
}

/**
 * Smooth one segment: minimise
 *   sum_t |q_t - ref_t|^2 + sum_k w_k (T^k d^k q)^2
 * per joint, where T is the smoothing horizon in frames. `before` and
 * `after` are fixed neighbouring frames (what the arm just did, and where it
 * goes next) that the derivative terms see but the solve cannot move; with
 * the segment's own end frames pinned, it joins its neighbours without a
 * new corner and can ease out of a standstill instead of keeping the step.
 */
function smoothSegment(segment, weights, horizon, before = [], after = []) {
  const ref = [...before, ...segment, ...after];
  const n = ref.length;
  const first = before.length;
  const last = first + segment.length - 1;
  if (segment.length <= 2) return segment.map((q) => [...q]);
  const width = 6;
  const band = Array.from({ length: n }, () => Array(width + 1).fill(0));
  for (let i = 0; i < n; i += 1) band[i][0] = 1;
  for (const [name, order] of Object.entries(ORDERS)) {
    const w = (weights[name] ?? 0) * horizon ** (2 * order);
    if (!w) continue;
    const c = stencil(order);
    for (let t = 0; t + order < n; t += 1) {
      for (let a = 0; a <= order; a += 1) {
        for (let b = a; b <= order; b += 1) band[t + a][b - a] += w * c[a] * c[b];
      }
    }
  }
  const pinned = (i) => i <= first || i >= last;
  const out = ref.map((q) => [...q]);
  for (let joint = 0; joint < ref[0].length; joint += 1) {
    const rhs = ref.map((q) => q[joint]);
    const sys = band.map((row) => [...row]);
    // Eliminate pinned frames: move their columns to the right-hand side.
    for (let i = 0; i < n; i += 1) {
      for (let d = 1; d <= width && i + d < n; d += 1) {
        const j = i + d;
        if (pinned(i) && !pinned(j)) rhs[j] -= sys[i][d] * ref[i][joint];
        if (pinned(j) && !pinned(i)) rhs[i] -= sys[i][d] * ref[j][joint];
        if (pinned(i) || pinned(j)) sys[i][d] = 0;
      }
      if (pinned(i)) { sys[i][0] = 1; rhs[i] = ref[i][joint]; }
    }
    solveBanded(sys, rhs, width).forEach((value, i) => { out[i][joint] = value; });
  }
  return out.slice(first, last + 1);
}

/** Resample a segment to `length` frames by linear interpolation: re-timing it slower. */
function stretchSegment(ref, length) {
  const n = ref.length;
  return Array.from({ length }, (_, i) => {
    const at = (i * (n - 1)) / (length - 1);
    const lo = Math.floor(at);
    const hi = Math.min(n - 1, lo + 1);
    const t = at - lo;
    return { q: ref[lo].map((value, joint) => value + (ref[hi][joint] - value) * t), at };
  });
}

/**
 * Re-time a segment so it slows smoothly into and out of every corner and
 * both ends, staying exactly on its path. Speed along the source frames
 * follows a sin^2 ramp of `radius` frames down to `floor` of full speed at
 * each event, so velocity - and with it every higher derivative - changes
 * gradually instead of in one step. Suits paths that must not leave their
 * line (a carried module), where rounding a corner is not allowed.
 */
function easeSegment(ref, { radius = 6, floor = 0.12, turn: sharp = 0.25, cap = 0.05 } = {}) {
  const n = ref.length;
  const events = [0, n - 1];
  for (let t = 1; t < n - 1; t += 1) {
    const turn = Math.max(...ref[t].map((value, joint) => Math.abs(ref[t + 1][joint] - 2 * value + ref[t - 1][joint])));
    if (turn > sharp * cap) events.push(t);
  }
  const speed = (at) => events.reduce((low, e) => {
    const x = Math.abs(at - e) / radius;
    return Math.min(low, x >= 1 ? 1 : floor + (1 - floor) * Math.sin((Math.PI / 2) * x) ** 2);
  }, 1);
  // Integrate time over source index on a fine grid, then sample it at whole steps.
  const grid = 8;
  const times = [0];
  for (let k = 1; k <= (n - 1) * grid; k += 1) {
    const mid = (k - 0.5) / grid;
    times.push(times[k - 1] + 1 / (grid * speed(mid)));
  }
  const total = times.at(-1);
  const length = Math.max(2, Math.ceil(total) + 1);
  const out = [];
  let k = 0;
  for (let i = 0; i < length; i += 1) {
    const time = (i * total) / (length - 1);
    while (k < times.length - 2 && times[k + 1] < time) k += 1;
    const span = times[k + 1] - times[k] || 1;
    const at = Math.min(n - 1, (k + (time - times[k]) / span) / grid);
    const lo = Math.floor(at);
    const hi = Math.min(n - 1, lo + 1);
    const t = at - lo;
    out.push({ q: ref[lo].map((value, joint) => value + (ref[hi][joint] - value) * t), at });
  }
  return out;
}

/**
 * Smooth a verified joint path under the weighted jerk/snap/crackle/pop cost.
 *
 * `anchors` are frame indexes that must not move (gripper open/close, stage
 * ends); the path is smoothed between them. A planned path usually already
 * runs near the rate cap, and a smooth start or stop has to be slower than
 * a step change, so each segment may also be re-timed before it is smoothed:
 * eased into its corners (`'ease'`, see easeSegment) or stretched uniformly
 * by a factor, trying each entry of `stretches` in turn. A candidate is kept only if every
 * frame stays inside the joint limits, the rate cap (`maxDelta`) and
 * `validate(q, sourceIndex)` - sourceIndex being the (fractional) index in
 * the original path the frame stands in for. The first re-timing and largest
 * horizon that pass win (horizon 0: re-timed only); a segment that never passes keeps its frames.
 *
 * Returns { frames, source, smoothedSegments, keptSegments }, where
 * source[i] is output frame i's source index (integer at every anchor).
 */
export function smoothJointPath(frames, {
  arm,
  weights = KINEMATIC_WEIGHTS,
  horizon = 5,
  anchors = [],
  validate = () => true,
  maxDelta = arm.maxActionDelta,
  stretches = ['ease', 1, 1.2, 1.4],
  ease = {},
} = {}) {
  const cuts = [...new Set([0, frames.length - 1, ...anchors.filter((i) => i > 0 && i < frames.length - 1)])].sort((a, b) => a - b);
  const out = [[...frames[0]]];
  const source = [0];
  let smoothedSegments = 0;
  let keptSegments = 0;
  // A plan that rides a joint limit smooths to a hair past it: clamp, then check.
  const intoLimits = (q) => q.map((value, joint) => clamp(value, -limitOf(arm, joint), limitOf(arm, joint)));
  const passes = (candidate, at) => candidate.every((q, i) => {
    const prev = i ? candidate[i - 1] : out.at(-1);
    return q.every((value, joint) => Math.abs(value - prev[joint]) <= maxDelta + 1e-9)
      && validate(q, at[i]);
  });
  for (let s = 0; s < cuts.length - 1; s += 1) {
    const from = cuts[s];
    const to = cuts[s + 1];
    const ref = frames.slice(from, to + 1);
    let accepted = null;
    search: for (const stretch of stretches) {
      const timed = stretch === 'ease'
        ? easeSegment(ref, { ...ease, cap: maxDelta })
        : stretchSegment(ref, stretch === 1 ? ref.length : Math.round((ref.length - 1) * stretch) + 1);
      const at = timed.map(({ at: t }) => from + t);
      // Context: the frames already emitted, and the raw path beyond `to`.
      const before = out.slice(-3);
      const after = frames.slice(to + 1, to + 4);
      // Largest horizon first; a horizon of 0 is the re-timing alone, which
      // stays exactly on the verified path.
      for (const h of [horizon, horizon / 2, horizon / 4, 0]) {
        const raw = timed.map(({ q }) => q);
        const candidate = (h ? smoothSegment(raw, weights, h, before, after) : raw).map(intoLimits);
        if (passes(candidate, at)) { accepted = { candidate, at }; break search; }
      }
    }
    // The segment's first frame is the previous segment's last: skip it.
    const keep = accepted || { candidate: ref, at: ref.map((_, i) => from + i) };
    keep.candidate.slice(1).forEach((q) => out.push([...q]));
    source.push(...keep.at.slice(1));
    if (accepted) smoothedSegments += 1;
    else keptSegments += 1;
  }
  return { frames: out, source, smoothedSegments, keptSegments };
}

/**
 * Live per-limb follower.
 *
 * The commanded pose passes through a cascade of four first-order stages,
 * one per weighted derivative, so the arm's jerk, snap, crackle and pop are
 * all continuous: a limb eases into motion and out of it instead of
 * snapping to the pointer. Each stage's gain is 1 / (1 + stiffness * w):
 * the heavier a derivative is weighted, the slower its stage, and at the
 * default stiffness the cascade settles in about fifteen ticks.
 *
 * step() returns the next pose, or null (and halts the cascade on the
 * current pose) when the next pose would break a constraint - the arm stops
 * against the limit rather than passing through it.
 */
export class LimbController {
  constructor(q, arm, { weights = KINEMATIC_WEIGHTS, stiffness = 6 } = {}) {
    this.arm = arm;
    this.gains = Object.keys(ORDERS).map((name) => 1 / (1 + stiffness * (weights[name] ?? 0)));
    this.reset(q);
  }

  /** Put every stage of the cascade at rest on `q`. */
  reset(q) {
    this.q = [...q];
    this.target = [...q];
    this.stages = this.gains.map(() => [...q]);
  }

  setTarget(target) {
    this.target = target.map((value, joint) => clamp(value, -limitOf(this.arm, joint), limitOf(this.arm, joint)));
  }

  /** True once the arm has come to rest on its target. */
  get settled() {
    return this.stages.every((stage) => stage.every((value, joint) => Math.abs(value - this.target[joint]) < 1e-4))
      && this.q.every((value, joint) => Math.abs(value - this.target[joint]) < 1e-4);
  }

  /** Advance one control tick. `isSafe(q)` is the caller's full constraint check. */
  step(isSafe = () => true) {
    let input = this.target;
    const stages = this.stages.map((stage, k) => {
      const next = stage.map((value, joint) => value + this.gains[k] * (input[joint] - value));
      input = next;
      return next;
    });
    const cap = this.arm.maxActionDelta;
    const q = this.q.map((value, joint) => value + clamp(input[joint] - value, -cap, cap));
    if (!isSafe(q)) {
      this.reset(this.q);
      return null;
    }
    this.stages = stages;
    this.q = q;
    return [...q];
  }
}

/**
 * One damped least-squares step moving the far end of limb `limb` (the
 * point after joint `limb`) toward `target`, using only joints 0..limb.
 * Joints beyond the grabbed limb are left alone, so distal limbs ride along.
 */
export function limbDragTarget(q, arm, limb, target, { damping = 8, maxStep = 0.25 } = {}) {
  const tip = (pose) => forwardKinematics(pose, arm).points[limb + 1];
  const at = tip(q);
  const error = [0, 1, 2].map((axis) => target[axis] - at[axis]);
  const h = 1e-4;
  const columns = [];
  for (let joint = 0; joint <= limb; joint += 1) {
    const plus = [...q];
    const minus = [...q];
    plus[joint] += h;
    minus[joint] -= h;
    const a = tip(plus);
    const b = tip(minus);
    columns.push([0, 1, 2].map((axis) => (a[axis] - b[axis]) / (2 * h)));
  }
  // dq = J^T (J J^T + lambda^2 I)^-1 e, with J 3 x (limb + 1).
  const jjt = [0, 1, 2].map((r) => [0, 1, 2].map((c) => columns.reduce((sum, col) => sum + col[r] * col[c], 0) + (r === c ? damping ** 2 : 0)));
  const y = solve3x3(jjt, error);
  const next = [...q];
  columns.forEach((col, joint) => {
    const dq = col[0] * y[0] + col[1] * y[1] + col[2] * y[2];
    next[joint] = clamp(q[joint] + clamp(dq, -maxStep, maxStep), -limitOf(arm, joint), limitOf(arm, joint));
  });
  return next;
}

function solve3x3(m, v) {
  const det = (a) => a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
  const d = det(m) || 1e-12;
  return [0, 1, 2].map((col) => det(m.map((row, r) => row.map((value, c) => (c === col ? v[r] : value)))) / d);
}
