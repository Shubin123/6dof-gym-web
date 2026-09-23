/**
 * The Task 7 (towel fold) guide both viewports draw in place of per-arm goal
 * cubes: which corner each gripper takes, where the fold line is, where Arm
 * B's corner goes, and which step of the fold the towel is actually at.
 */

/** Scene-pixel landmarks of the fold, matching the towel's rest footprint. */
export const FOLD_GUIDE = Object.freeze({
  cornerA: Object.freeze([250, 180]),
  cornerB: Object.freeze([400, 180]),
  foldX: 325,
  // Arm B lays its corner on Arm A's.
  place: Object.freeze([250, 180]),
  // The towel's far edge, for drawing the fold line across it.
  farY: 300,
});

export const FOLD_STAGES = Object.freeze({
  grasp: '1 · Grasp both front corners',
  lift: '2 · A pins its corner, B lifts over the fold line',
  place: "3 · A lets go; B lays its corner on A's",
  done: 'Folded',
});

/**
 * The fold step the towel is at, read from the cloth state rather than from
 * plan progress, so manual runs and scrubbed timelines show the truth.
 */
export function foldGuideStage(cloth) {
  if (!cloth) return 'grasp';
  const { captured = [], wasCaptured = [], tablePinA = null } = cloth;
  if (!(captured[1] || wasCaptured[1]) || !(captured[0] || wasCaptured[0])) return 'grasp';
  if (captured[0]) return 'lift';
  if (tablePinA && cloth.getFrontFoldDistance?.() < 0.05) return 'done';
  return 'place';
}

/** Scene position of simulator point `index` (cloth-local units, centred on 325, 240). */
export function clothPointToScene(cloth, index) {
  const p = cloth.positions;
  return [325 + p[index * 3] * 100, 240 + p[index * 3 + 1] * 100, p[index * 3 + 2] * 100];
}
