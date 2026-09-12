import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };

test('compiled environment has the browser task contract', () => {
  assert.equal(compiled.environment.id, 'Arm6-Reach-v0');
  assert.equal(compiled.environment.observation_dim, 22);
  assert.equal(compiled.environment.action_dim, 7);
  assert.equal(compiled.environment.control_hz, 25);
});
test('every workflow is bounded by the declared workspace', () => {
  const [minX, maxX, minY, maxY] = compiled.environment.safety.goal_workspace;
  for (const { goal } of compiled.workflows) assert.ok(goal[0] >= minX && goal[0] <= maxX && goal[1] >= minY && goal[1] <= maxY);
});
test('model and dataset cards retain a primary URL', () => {
  for (const item of [...compiled.models, ...compiled.datasets]) assert.match(item.url, /^https:\/\//);
});
