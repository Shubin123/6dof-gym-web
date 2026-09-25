/**
 * Bimanual half fold: "four corners into two".
 *
 * Task 7 pins one corner with Arm A and carries the other with Arm B, which
 * moves a single corner; the rest of that edge trails behind. A true half
 * fold carries both corners of one edge together and lays each on the
 * corner opposite it, so the towel's four corners end as two stacked pairs.
 * Each arm carries the corner on its own side along an arc over the fold
 * line; the arcs run at the same time and every combined frame is checked
 * against the full cell-safety envelope (floor, table edge, arm-to-arm).
 *
 * Near the arm bases the towel's corners are cramped, which is why the
 * carried segments use arm-track.js's clearance-seeking tracker rather than
 * core.js's plain straight-line planner.
 */
import { evaluateCellSafety, HOME_POSE, planSafeCellMotion, planSafeMotion } from './core.js';
import { planClearCartesianMotion, solveClearIK } from './arm-track.js';

const hold = (frames, length) => {
  const last = frames.at(-1);
  return [...frames, ...Array(Math.max(0, length - frames.length)).fill(last)];
};
const reverseSegment = (from, frames) => [...frames.slice(0, -1).reverse(), from];

/** Points along a half-circle arc from `from` to `to` (scene [x, y]), peaking at `height`. */
function arcPoints(from, to, { startZ, endZ, height, count = 8 }) {
  const points = [];
  for (let i = 1; i <= count; i += 1) {
    const angle = (i / count) * Math.PI;
    const t = (1 - Math.cos(angle)) / 2;
    const z = i === count ? endZ : startZ + (endZ - startZ) * t + height * Math.sin(angle);
    points.push([from[0] + (to[0] - from[0]) * t, from[1] + (to[1] - from[1]) * t, z]);
  }
  return points;
}

/**
 * One arm's carried motion: grasp pose, straight rise off it (walked
 * backwards, this is the descent onto the corner), and the fold arc. Tried
 * from the grasp end and, failing that, planned from the place end and
 * walked backwards - the arc is a fixed tool path, so either direction
 * gives the same tip motion, but the two ends can need different elbow
 * branches. Returns { grasp, rise, arc } or null.
 */
function carryPlan(arm, from, to, { graspZ, hoverZ, placeZ, arcHeight }, safety) {
  const arc = arcPoints(from, to, { startZ: graspZ, endZ: placeZ, height: arcHeight });
  for (const grasp of solveClearIK([...from, graspZ], arm, safety, [], { limit: 3 })) {
    const rise = planClearCartesianMotion(grasp, [[...from, hoverZ]], arm, safety);
    const carry = rise && planClearCartesianMotion(grasp, arc, arm, safety);
    if (carry) return { grasp, rise, arc: carry };
  }
  const back = arcPoints(to, from, { startZ: placeZ, endZ: graspZ, height: arcHeight });
  for (const place of solveClearIK([...to, placeZ], arm, safety, [], { limit: 3 })) {
    const reversed = planClearCartesianMotion(place, back, arm, safety);
    if (!reversed) continue;
    const grasp = reversed.at(-1);
    const rise = planClearCartesianMotion(grasp, [[...from, hoverZ]], arm, safety);
    if (rise) return { grasp, rise, arc: reverseSegment(place, reversed) };
  }
  return null;
}

/**
 * Plan the half fold for a two-arm cell.
 *
 * `profile.carry` is [[fromA, toA], [fromB, toB]] in scene pixels: the
 * corner each arm grasps and the corner it lays that one on.
 *
 *  1. Approach above both corners (free-space joint route)
 *  2. Descend straight onto the corners and close
 *  3. Carry both corners over the fold line together
 *  4. Open, rise straight off the towel
 *  5. Return home and hold while the towel settles
 *
 * Returns { frames, grips, stageEnds } - one [qA, qB] cell frame and one
 * [closedA, closedB] gripper command per step - or null if no safe plan.
 */
// Plans from the home pose, keyed by the fold profile: the carry search is
// the slow part (a few seconds), and a looping demo replans every episode.
const cachedHalfFolds = new Map();
const atHome = (q) => q.every((value, joint) => Math.abs(value - HOME_POSE[joint]) < 1e-3);

export function planHalfFoldMotion(poses, safety = {}, profile = {}) {
  if (poses.length < 2) return null;
  const fromHome = poses.every(({ q }) => atHome(q));
  const cacheKey = JSON.stringify([profile, safety]);
  if (fromHome && cachedHalfFolds.has(cacheKey)) return structuredClone(cachedHalfFolds.get(cacheKey));
  const result = planHalfFold(poses, safety, profile);
  if (fromHome && result) cachedHalfFolds.set(cacheKey, structuredClone(result));
  return result;
}

function planHalfFold(poses, safety, profile) {
  const arms = poses.map(({ arm }) => arm);
  const options = {
    graspZ: profile.graspZ ?? 4,
    hoverZ: profile.hoverZ ?? 40,
    placeZ: profile.placeHeight ?? 8,
    arcHeight: profile.arcHeight ?? 45,
  };
  const settleFrames = profile.settleFrames ?? 60;
  const dwellFrames = 3;
  // Each arm works in its own lane: its carry is planned with the table
  // edge pulled in to the cell midline, less half the arm-to-arm clearance.
  // Planned one arm at a time, both arms' links could otherwise meet in the
  // middle - with the towel centred between mirrored arms they do, exactly.
  const [minX, maxX, minY, maxY] = safety.goal_workspace;
  const midline = (arms[0].base[0] + arms[1].base[0]) / 2;
  const half = (safety.arm_clearance_px ?? 0) / 2;
  const lanes = [
    { ...safety, goal_workspace: [minX, midline - half, minY, maxY] },
    { ...safety, goal_workspace: [midline + half, maxX, minY, maxY] },
  ];
  const carries = profile.carry.map(([from, to], index) => carryPlan(arms[index], from, to, options, lanes[index]));
  if (carries.some((carry) => !carry)) return null;

  const frames = [];
  const grips = [];
  const stageEnds = [];
  let grip = [false, false];
  let current = poses.map(({ q }) => [...q]);
  const cellSafe = (frame) => evaluateCellSafety(frame.map((q, index) => ({ q, arm: arms[index] })), safety).safe;
  const push = (frame) => { current = frame.map((q) => [...q]); frames.push(current.map((q) => [...q])); grips.push([...grip]); };
  const dwell = (count) => { for (let i = 0; i < count; i += 1) push(current); };
  const endStage = () => stageEnds.push(frames.length - 1);
  // Both arms move at once; the shorter track holds its last pose. Every
  // combined frame must pass the two-arm check, not just each arm alone.
  const together = (tracks) => {
    const length = Math.max(...tracks.map((track) => track.length));
    const padded = tracks.map((track) => hold(track, length));
    for (let i = 0; i < length; i += 1) {
      const frame = padded.map((track) => track[i]);
      if (!cellSafe(frame)) return false;
      push(frame);
    }
    return true;
  };

  // 1. Approach: free space, nothing held, so any safe joint route will do.
  const approach = planSafeCellMotion(poses, carries.map(({ rise }) => rise.at(-1)), safety);
  if (!approach) return null;
  approach.frames.forEach(push);
  endStage();

  // 2. Straight down onto both corners, then close.
  if (!together(carries.map(({ grasp, rise }) => reverseSegment(grasp, rise)))) return null;
  grip = [true, true];
  dwell(dwellFrames);
  endStage();

  // 3. Carry both corners over the fold line.
  if (!together(carries.map(({ arc }) => arc))) return null;
  dwell(dwellFrames);
  endStage();

  // 4. Let go and rise straight off the towel.
  grip = [false, false];
  dwell(dwellFrames);
  const rises = current.map((q, index) => {
    const tip = profile.carry[index][1];
    return planClearCartesianMotion(q, [[tip[0], tip[1], options.hoverZ]], arms[index], lanes[index]);
  });
  if (rises.some((rise) => !rise) || !together(rises)) return null;
  endStage();

  // 5. Home one arm at a time, then hold while the cloth settles.
  for (let index = 0; index < 2; index += 1) {
    const others = current.flatMap((q, other) => (other === index ? [] : [{ q, arm: arms[other] }]));
    const home = planSafeMotion(current[index], [...HOME_POSE], arms[index], safety, others);
    if (!home) continue;
    for (const q of home) push(current.map((pose, other) => (other === index ? q : pose)));
  }
  endStage();
  dwell(settleFrames);
  return { frames, grips, stageEnds, sourceFrames: frames.length, reducedBy: 0 };
}
