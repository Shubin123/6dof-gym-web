import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, evaluateCellSafety, forwardKinematics, HOME_POSE } from '../src/core.js';
import { RigidScene, sceneYaw, snapshotObject, toolBasis } from '../src/rigid.js';
import { planPickPlaceMotion, solveToolDownIK, TOOL_DOWN_TOLERANCE } from '../src/rigid-plan.js';
import { isRigidTask, pickPlacePoints, planRigidTask, rigidOutcome } from '../src/rigid-tasks.js';

const safety = compiled.environment.safety;
const rigidTasks = compiled.workflows.filter(isRigidTask);
const home = () => ({ q: [...HOME_POSE], arm: ARM });

/** Run a plan through a scene; return the frame index each grasp took hold, if any. */
function runPlan(scene, plan) {
  let heldAt = null;
  plan.frames.forEach(([q], index) => {
    scene.step({ arms: [{ q, arm: ARM }], grips: plan.grips[index] });
    if (heldAt === null && scene.grippers[0].held) heldAt = index;
  });
  return heldAt;
}

test('the rigid family declares cube and sphere tasks with real object specs', () => {
  assert.deepEqual(rigidTasks.map((task) => task.id), ['pick_cube', 'pick_sphere', 'stack_cubes']);
  const shapes = new Set(rigidTasks.flatMap((task) => task.rigid.objects.map((object) => object.shape)));
  assert.deepEqual([...shapes].sort(), ['box', 'sphere']);
  for (const task of rigidTasks) {
    assert.equal(task.family, 'rigid');
    assert.ok(task.rigid.objects.some((object) => object.id === task.rigid.goal.object), `${task.id} goal names a declared object`);
  }
});

test('objects rest on the table under gravity instead of floating or sinking', () => {
  const scene = new RigidScene(rigidTasks.find((task) => task.id === 'stack_cubes').rigid, { arms: 1 });
  for (let i = 0; i < 25; i += 1) scene.step({ arms: [home()] });
  assert.ok(Math.abs(scene.objectCenter('top')[2] - 15) < 0.5, 'a 30 mm cube rests with its center 15 mm up');
  assert.ok(scene.atRest());
});

test('a released object falls: gravity acts on anything not held', () => {
  const rigid = { objects: [{ id: 'cube', shape: 'box', size: 30, position: [450, 200], mass: 0.06 }] };
  const scene = new RigidScene(rigid, { arms: 1 });
  scene.object('cube').body.position.y = 0.2; // 200 mm above the table
  for (let i = 0; i < 25; i += 1) scene.step({ arms: [home()] });
  assert.ok(Math.abs(scene.objectCenter('cube')[2] - 15) < 1, 'lands on the table within a second');
});

test('tool-down IK points the tool straight down with the jaws opening horizontally', () => {
  const q = solveToolDownIK([450, 180, 8], ARM, safety);
  assert.ok(q, 'the pick point is reachable tool-down');
  assert.ok(evaluateCellSafety([{ q, arm: ARM }], safety).safe);
  const { forward, up, points } = forwardKinematics(q, ARM);
  assert.ok(Math.hypot(forward[0], forward[1]) < TOOL_DOWN_TOLERANCE && forward[2] < 0);
  assert.ok(Math.abs(points.at(-1)[2] - 8) < 0.5);
  // World y is up: a horizontal jaw axis has no y component.
  assert.ok(Math.abs(toolBasis(forward, up).lateral[1]) < TOOL_DOWN_TOLERANCE);
});

for (const task of rigidTasks) {
  test(`${task.number} ${task.name}: the planned pick and place succeeds in physics`, () => {
    const scene = new RigidScene(task.rigid, { arms: 1 });
    const start = task.rigid.objects.map((object) => scene.objectCenter(object.id));
    const plan = planRigidTask(task.rigid, scene, home(), safety);
    assert.ok(plan, 'plans a safe route');
    let previous = HOME_POSE;
    for (const [q] of plan.frames) {
      assert.ok(q.every((value, joint) => Math.abs(value - previous[joint]) <= ARM.maxActionDelta + 1e-9), 'every frame respects the joint-delta cap');
      assert.ok(evaluateCellSafety([{ q, arm: ARM }], safety).safe, 'every frame passes the cell-safety check');
      previous = q;
    }
    const heldAt = runPlan(scene, plan);
    assert.ok(heldAt !== null, 'the jaws closed on the object');
    const outcome = rigidOutcome(task.rigid, scene);
    assert.ok(outcome.success, `outcome ${JSON.stringify(outcome)}`);
    // Only the picked object may have moved.
    task.rigid.objects.forEach((object, index) => {
      if (object.id === task.rigid.goal.object) return;
      const moved = Math.hypot(...scene.objectCenter(object.id).map((value, axis) => value - start[index][axis]));
      assert.ok(moved < 3, `${object.id} was disturbed by ${moved.toFixed(1)} mm`);
    });
  });
}

test('the grasp is contact-gated: a stale plan closes on air and the task fails', () => {
  const task = rigidTasks.find((entry) => entry.id === 'pick_cube');
  const scene = new RigidScene(task.rigid, { arms: 1 });
  const plan = planRigidTask(task.rigid, scene, home(), safety);
  // Move the cube 60 mm after planning, as if it were bumped.
  scene.object('cube').body.position.x += 0.06;
  const heldAt = runPlan(scene, plan);
  assert.equal(heldAt, null, 'nothing between the jaws, nothing held');
  const outcome = rigidOutcome(task.rigid, scene);
  assert.equal(outcome.success, false);
  assert.ok(outcome.error > 100, 'the cube is still far from the zone');
});

test('a held cube is still a collider: it cannot be lowered through another cube', () => {
  const task = rigidTasks.find((entry) => entry.id === 'stack_cubes');
  const scene = new RigidScene(task.rigid, { arms: 1 });
  const { pick } = pickPlacePoints(task.rigid, scene);
  // Aim the release at the table under the base cube instead of its top.
  const plan = planPickPlaceMotion(home(), { pick, place: [460, 320, 10] }, safety);
  assert.ok(plan);
  runPlan(scene, plan);
  const top = scene.objectCenter('top');
  const base = scene.objectCenter('base');
  // Driving the held cube down pushes the base aside or ends with the top
  // cube above it; the two never occupy the same space.
  const overlap = Math.abs(top[0] - base[0]) < 29 && Math.abs(top[1] - base[1]) < 29 && Math.abs(top[2] - base[2]) < 29;
  assert.equal(overlap, false, `top ${top.map(Math.round)} base ${base.map(Math.round)}`);
});

test('snapshot and restore reproduce the physical state for timeline scrubbing', () => {
  const task = rigidTasks.find((entry) => entry.id === 'pick_cube');
  const scene = new RigidScene(task.rigid, { arms: 1 });
  const plan = planRigidTask(task.rigid, scene, home(), safety);
  const midpoint = plan.stageEnds[2];
  plan.frames.slice(0, midpoint).forEach(([q], index) => scene.step({ arms: [{ q, arm: ARM }], grips: plan.grips[index] }));
  const saved = scene.snapshot();
  assert.ok(saved.grippers[0].held, 'mid-carry the cube is held');
  const carried = scene.objectCenter('cube');

  const replay = new RigidScene(task.rigid, { arms: 1 });
  replay.restore(JSON.parse(JSON.stringify(saved)));
  assert.deepEqual(replay.objectCenter('cube').map((v) => v.toFixed(3)), carried.map((v) => v.toFixed(3)));
  plan.frames.slice(midpoint).forEach(([q], index) => replay.step({ arms: [{ q, arm: ARM }], grips: plan.grips[midpoint + index] }));
  assert.ok(rigidOutcome(task.rigid, replay).success, 'a restored scene finishes the task like the original');

  const drawn = snapshotObject(saved, 'cube');
  assert.deepEqual(drawn.center.map((v) => v.toFixed(3)), carried.map((v) => v.toFixed(3)));
});

test('the cube is drawn at its declared yaw', () => {
  const task = rigidTasks.find((entry) => entry.id === 'pick_cube');
  const scene = new RigidScene(task.rigid, { arms: 1 });
  const { quaternion } = snapshotObject(scene.snapshot(), 'cube');
  assert.ok(Math.abs(sceneYaw(quaternion) - task.rigid.objects[0].yaw) < 1e-6);
});
