/**
 * Task layer for the rigid-object scenarios: where to grasp and release,
 * read from the live physics state rather than the task file, and whether
 * the physical outcome meets the task's goal.
 *
 * A task's `rigid.goal` is one of
 *   { type: 'zone',  object, position: [x, y], size: [w, d] }  object resting inside a floor zone
 *   { type: 'tray',  object, fixture }                         object inside a tray's walls
 *   { type: 'stack', object, on, tolerance }                   object resting squarely on another
 *   { type: 'tower', position: [x, y], height, tolerance }     a column of `height` cubes (the live task)
 */
import { planPickPlaceMotion } from './rigid-plan.js';
import { autoPropagateOutcome, planAutoPropagateTask } from './auto-propagate.js';

/** Tool tip height above an object's bottom face at the grasp (mm). */
export const GRASP_HEIGHT = 8;
/** Gap left under an object when it is released (mm). */
const RELEASE_GAP = 2;

export const isRigidTask = (workflow) => Boolean(workflow?.rigid);
/** A task whose physics runs continuously and whose arm is driven by a controller, not one plan. */
export const isLiveRigidTask = (workflow) => Boolean(workflow?.rigid?.live);

/**
 * The cubes that currently form the tower at `goal.position`, bottom first:
 * level k must rest k cube-heights up, within `tolerance` of the column's
 * axis and of the cube below. Counting stops at the first gap, so a cube
 * left on a knocked-over tower's rubble does not count. Held cubes never do.
 * Counted by position alone unless `speed` (m/s) is given: stacked cubes
 * jitter slightly in the solver, and a tower must not flicker in and out.
 */
export function towerStack(rigid, scene, { speed = Infinity } = {}) {
  const { goal } = rigid;
  const tolerance = goal.tolerance ?? 10;
  const held = new Set(scene.grippers.map((gripper) => gripper.held?.id).filter(Boolean));
  const cubes = scene.objects
    .filter((object) => object.shape === 'box' && !held.has(object.id) && object.body.velocity.length() < speed)
    .map((object) => ({ object, center: scene.objectCenter(object.id) }))
    .filter(({ center }) => flat(center, goal.position) <= tolerance)
    .sort((a, b) => a.center[2] - b.center[2]);
  const tower = [];
  for (const entry of cubes) {
    const level = tower.length;
    const expected = entry.object.size / 2 + level * entry.object.size;
    if (Math.abs(entry.center[2] - expected) > 5) continue;
    if (level && flat(entry.center, tower[level - 1].center) > tolerance) break;
    tower.push(entry);
  }
  return tower.map(({ object }) => object.id);
}

const flat = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

/** Scene-space footprint center [x, y] of a task's goal, for drawing and for the planner. */
export function goalCenter(rigid) {
  const { goal } = rigid;
  if (goal.type === 'zone' || goal.type === 'tower' || goal.type === 'propagate') return goal.position;
  if (goal.type === 'tray') return rigid.fixtures.find((fixture) => fixture.id === goal.fixture).position;
  return null; // 'stack' follows the base object, which is live state
}

/**
 * Tool-tip grasp and release points for the task's pick object in `scene`'s
 * current state. Grasp: GRASP_HEIGHT above the object's bottom, straight
 * over its center. Release: the object's bottom RELEASE_GAP above the
 * surface it goes onto (table, tray floor, or the top of the base object).
 */
export function pickPlacePoints(rigid, scene) {
  const { goal } = rigid;
  const object = scene.object(goal.object);
  const half = object.size / 2;
  const center = scene.objectCenter(goal.object);
  // The tip holds the object GRASP_HEIGHT above its bottom; that offset is
  // what carries over to the release.
  const pick = [center[0], center[1], center[2] - half + GRASP_HEIGHT];
  let surface = 0;
  let at = goalCenter(rigid);
  if (goal.type === 'stack') {
    const base = scene.object(goal.on);
    const baseCenter = scene.objectCenter(goal.on);
    at = baseCenter;
    surface = baseCenter[2] + base.size / 2;
  }
  const place = [at[0], at[1], surface + RELEASE_GAP + GRASP_HEIGHT];
  return { pick, place };
}

/** Plan the task's pick and place for the arm at `pose` from `scene`'s current state. */
export function planRigidTask(rigid, scene, pose, safety) {
  if (rigid.goal.type === 'propagate') {
    return planAutoPropagateTask(rigid, scene, pose, safety);
  }
  const { pick, place } = pickPlacePoints(rigid, scene);
  return planPickPlaceMotion(pose, { pick, place, hoverZ: rigid.hover_z ?? 90 }, safety);
}

/**
 * Score the physical outcome. `success` needs the object where the goal
 * says, resting there (not still sliding or in the gripper), and for a stack
 * the base object not knocked out of place. `error` is the horizontal
 * distance to the goal in scene px, for the telemetry readout and reward.
 */
export function rigidOutcome(rigid, scene) {
  const { goal } = rigid;
  if (goal.type === 'propagate') {
    return autoPropagateOutcome(rigid, scene);
  }
  if (goal.type === 'tower') {
    const height = towerStack(rigid, scene).length;
    const placed = height >= goal.height;
    return { success: placed, placed, resting: true, held: scene.grippers.some((gripper) => gripper.held), error: (goal.height - height) * 30, height };
  }
  const object = scene.object(goal.object);
  const center = scene.objectCenter(goal.object);
  const half = object.size / 2;
  const held = scene.grippers.some((gripper) => gripper.held?.id === goal.object);
  const resting = !held && scene.atRest();
  let error;
  let placed;
  if (goal.type === 'zone') {
    const [cx, cy] = goal.position;
    const [w, d] = goal.size;
    error = flat(center, goal.position);
    placed = Math.abs(center[0] - cx) <= w / 2 && Math.abs(center[1] - cy) <= d / 2 && Math.abs(center[2] - half) < 3;
  } else if (goal.type === 'tray') {
    const tray = rigid.fixtures.find((fixture) => fixture.id === goal.fixture);
    const [cx, cy] = tray.position;
    const [w, d] = tray.inner;
    error = flat(center, tray.position);
    placed = Math.abs(center[0] - cx) <= w / 2 && Math.abs(center[1] - cy) <= d / 2 && center[2] < (tray.height ?? 18);
  } else {
    const base = scene.object(goal.on);
    const baseCenter = scene.objectCenter(goal.on);
    const baseStart = base.position;
    error = flat(center, baseCenter);
    placed = error <= (goal.tolerance ?? 10)
      && Math.abs(center[2] - (baseCenter[2] + base.size / 2 + half)) < 4
      && flat(baseCenter, baseStart) < 15;
  }
  return { success: placed && resting, placed, resting, held, error };
}
