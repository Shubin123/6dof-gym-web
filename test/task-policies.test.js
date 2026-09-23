import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ClothSimulator } from '../src/cloth.js';
import { bootstrapPolicy, policyRecipeFor, scoreTaskStages } from '../src/task-policies.js';

test('task-policy recipes provide a declarative warm-start for new task use cases', () => {
  const fold = compiled.workflows.find((task) => task.id === 'fold');
  const recipe = policyRecipeFor(fold);
  const warmStart = bootstrapPolicy(fold);
  assert.equal(recipe.kind, 'cloth-fold');
  assert.equal(warmStart.profile.placeHeight, 6);
  assert.equal(recipe.stages.reduce((total, stage) => total + stage.weight, 0), 1);
});

test('cloth fold stage score rewards intermediate contact and placement milestones', () => {
  const cloth = new ClothSimulator({ columns: 8, rows: 6 });
  const task = compiled.workflows.find((entry) => entry.id === 'fold');
  const before = scoreTaskStages(task, { cloth, tips: [[250, 180, 15], [400, 180, 15]] });
  cloth.wasCaptured = [true, true];
  cloth.released = [true, false];
  cloth.tablePinA = { x: -0.75, y: -0.6, z: cloth.tableZ };
  const after = scoreTaskStages(task, { cloth, tips: [[250, 180, 15], [300, 180, 20]] });
  assert.ok(after.reward > before.reward);
  assert.equal(after.stage, 'place on folded target');
});
