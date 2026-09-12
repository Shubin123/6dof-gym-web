import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, distance, forwardKinematics, guidedStep, projectToReachableWorkspace } from '../src/core.js';

test('compiled environment has the browser task contract', () => {
  assert.equal(compiled.environment.id, 'Arm6-Reach-v0');
  assert.equal(compiled.environment.observation_dim, 22);
  assert.equal(compiled.environment.action_dim, 7);
  assert.equal(compiled.environment.control_hz, 25);
});
test('every workflow is bounded by the declared reachable workspace', () => {
  const [minX, maxX, minY, maxY] = compiled.environment.safety.goal_workspace;
  for (const { goal } of compiled.workflows) {
    assert.ok(goal[0] >= minX && goal[0] <= maxX && goal[1] >= minY && goal[1] <= maxY);
    assert.ok(distance(goal, ARM.base) <= compiled.environment.safety.max_reach_px);
  }
});
test('model and dataset cards retain a primary URL', () => {
  for (const item of [...compiled.models, ...compiled.datasets]) assert.match(item.url, /^https:\/\//);
});

test('guidance lowers distance for every supplied arm-use workflow', () => {
  for (const { goal } of compiled.workflows) {
    let q = [-0.45, 0.2, 0.3, -0.2, -0.1, 0.15];
    const before = distance(forwardKinematics(q).points.at(-1), goal);
    for (let step = 0; step < compiled.environment.max_steps; step += 1) {
      const update = guidedStep(q, goal); q = update.q;
      assert.ok(update.action.every((delta) => Math.abs(delta) <= ARM.maxActionDelta + 1e-9));
    }
    const after = distance(forwardKinematics(q).points.at(-1), goal);
    assert.ok(after < 12, `${goal} finished ${after.toFixed(1)}px from target`);
    assert.ok(after < before, 'guidance should reduce target distance');
  }
});

test('dragged guidance target is projected safely into the reachable workspace', () => {
  const target = projectToReachableWorkspace([760, 0], compiled.environment.safety);
  assert.ok(distance(target, ARM.base) <= compiled.environment.safety.max_reach_px + 1e-9);
});
