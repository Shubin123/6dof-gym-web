// Task 16 (Stack under fire): the closed-loop stacker, run headless against
// the real physics with scripted shots. Its own file: each scenario is a few
// thousand simulated control steps.
import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, evaluateCellSafety, HOME_POSE } from '../src/core.js';
import { insideFence, RigidScene } from '../src/rigid.js';
import { StackController } from '../src/stack-controller.js';
import { rigidOutcome, towerStack } from '../src/rigid-tasks.js';

const safety = compiled.environment.safety;
const task = compiled.workflows.find((workflow) => workflow.id === 'stack_fire');

/**
 * Run the controller for `steps` frames, checking every frame the way
 * main.js's policyStep does. `onStep(step, scene)` may fire shots.
 */
function run(steps, onStep = () => {}) {
  const scene = new RigidScene(task.rigid, { arms: 1, armColliders: true });
  const controller = new StackController({ rigid: task.rigid, arm: ARM, safety });
  let q = [...HOME_POSE];
  const heights = [];
  for (let step = 0; step < steps; step += 1) {
    const frame = controller.next(scene, q);
    assert.ok(frame.q.every((value, joint) => Math.abs(value - q[joint]) <= ARM.maxActionDelta + 1e-9), `step ${step}: joint-rate cap`);
    assert.ok(evaluateCellSafety([{ q: frame.q, arm: ARM }], safety).safe, `step ${step}: cell safety`);
    q = frame.q;
    scene.step({ arms: [{ q, arm: ARM }], grips: [frame.grip] });
    onStep(step, scene, controller);
    heights.push(towerStack(task.rigid, scene).length);
  }
  return { scene, controller, heights };
}

test('Task 16 is a live rigid task with a three-cube tower goal and spare cubes', () => {
  assert.equal(task.family, 'rigid');
  assert.equal(task.rigid.live, true);
  assert.equal(task.rigid.goal.type, 'tower');
  assert.ok(task.rigid.objects.length > task.rigid.goal.height, 'spare cubes to rebuild with');
});

test('undisturbed, the arm builds the tower and then holds', () => {
  const { scene, controller, heights } = run(1300);
  assert.equal(rigidOutcome(task.rigid, scene).height, task.rigid.goal.height);
  assert.match(controller.status().phase, /complete/);
  assert.equal(controller.status().recoveries, 0);
  // It never knocks its own tower down on the way up.
  const firstFull = heights.indexOf(task.rigid.goal.height);
  assert.ok(firstFull > 0 && heights.slice(firstFull).every((h) => h === task.rigid.goal.height));
});

test('shot down, the tower is rebuilt; a hit on the carried cube knocks it loose and is recovered from', () => {
  let heldShotAt = null;
  const { scene, controller, heights } = run(3200, (step, sceneNow) => {
    // Knock the finished tower over, side-on at mid height, from inside the pen.
    if (step === 1300) sceneNow.spawnProjectile([473, 340, 45], [-1440, -3190, 300]);
    // Then hit the next cube the arm carries.
    if (step > 1500 && heldShotAt === null && sceneNow.grippers[0].held) {
      const c = sceneNow.objectCenter(sceneNow.grippers[0].held.id);
      sceneNow.spawnProjectile([c[0] + 60, c[1], c[2]], [-4000, 0, 0]);
      heldShotAt = step;
    }
  });
  assert.ok(heights.slice(1300, 1600).some((h) => h < task.rigid.goal.height), 'the shot brought the tower down');
  assert.ok(heldShotAt !== null, 'the arm was carrying a cube to shoot at');
  const status = controller.status();
  assert.ok(status.drops >= 1, 'the hit knocked the cube out of the gripper');
  assert.ok(status.recoveries >= 1);
  assert.equal(rigidOutcome(task.rigid, scene).height, task.rigid.goal.height, `rebuilt to ${task.rigid.goal.height}: ${JSON.stringify(status)}`);
});

test('a slow ball does not knock a held cube loose', () => {
  let shot = false;
  const { controller } = run(900, (step, sceneNow) => {
    if (!shot && sceneNow.grippers[0].held) {
      const c = sceneNow.objectCenter(sceneNow.grippers[0].held.id);
      sceneNow.spawnProjectile([c[0] + 30, c[1], c[2]], [-800, 0, 0]);
      shot = true;
    }
  });
  assert.ok(shot);
  assert.equal(controller.status().drops, 0);
});

test('every cube stays in play: none is ever left outside the pen', () => {
  let shots = 0;
  run(2400, (step, sceneNow) => {
    // Hit whatever the arm is carrying, hard, every time it lifts one.
    if (sceneNow.grippers[0].held && step % 20 === 0 && shots < 6) {
      const c = sceneNow.objectCenter(sceneNow.grippers[0].held.id);
      sceneNow.spawnProjectile([c[0] + 50, c[1], c[2] + 10], [-5000, 0, 800]);
      shots += 1;
    }
    for (const object of sceneNow.objects) {
      const center = sceneNow.objectCenter(object.id);
      const held = sceneNow.grippers[0].held?.id === object.id;
      const resting = object.body.sleepState === 2 || object.body.velocity.length() < 0.01;
      if (!held && resting) assert.ok(insideFence(task.rigid.fence, center), `step ${step}: ${object.id} left at ${center.map(Math.round)}`);
    }
  });
  assert.ok(shots > 0);
});

test('balls are real bodies: they fall, and old ones are cleared', () => {
  const scene = new RigidScene(task.rigid, { arms: 1, armColliders: true });
  for (let i = 0; i < 20; i += 1) scene.spawnProjectile([200 + i * 10, 150, 100], [0, 0, 0]);
  assert.ok(scene.projectiles.length <= 16, 'capped');
  for (let i = 0; i < 25; i += 1) scene.step({ arms: [{ q: [...HOME_POSE], arm: ARM }] });
  const ball = scene.projectiles.at(-1);
  assert.ok(Math.abs(ball.body.position.y * 1000 - 8) < 1, 'a ball comes to rest on the table');
  const saved = JSON.parse(JSON.stringify(scene.snapshot()));
  const copy = new RigidScene(task.rigid, { arms: 1, armColliders: true });
  copy.restore(saved);
  assert.equal(copy.projectiles.length, scene.projectiles.length, 'snapshot/restore keeps the balls');
});
