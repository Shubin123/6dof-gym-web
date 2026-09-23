import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, ARM_B, evaluateCellSafety, HOME_POSE, planSafeCellMotion, projectToReachableWorkspace, reduceSafeCellMotion, solveInverseKinematics } from '../src/core.js';
import { ClothSimulator } from '../src/cloth.js';
import { policyRecipeFor, scoreTaskStages } from '../src/task-policies.js';

// Regression coverage for exported contracts that the rest of the suite
// exercises only indirectly through full-workflow integration tests. These
// lock in behavior at module boundaries so a future refactor of core.js or
// task-policies.js gets a direct failure instead of a subtle drift.

const HOME = HOME_POSE;
const safety = compiled.environment.safety;

function planReducibleRoute(workflowId) {
  const workflow = compiled.workflows.find((entry) => entry.id === workflowId);
  const target = projectToReachableWorkspace([...workflow.goal, workflow.goal_height], safety, ARM);
  const plan = solveInverseKinematics(target, ARM, safety, []);
  const poses = [{ q: [...HOME], arm: ARM }];
  const motion = planSafeCellMotion(poses, [plan.q], safety);
  assert.ok(motion && motion.frames.length > 4, `${workflowId} should plan a multi-step route for this regression test`);
  return { poses, frames: motion.frames };
}

test('reduceSafeCellMotion returns an empty, zero-cost result for an empty frame list', () => {
  const poses = [{ q: [...HOME], arm: ARM }];
  const result = reduceSafeCellMotion(poses, [], safety);
  assert.deepEqual(result, { frames: [], sourceFrames: 0, reducedBy: 0 });
});

test('reduceSafeCellMotion keeps every frame when keepTailFrames covers the whole route', () => {
  const { poses, frames } = planReducibleRoute('stack');
  const untouched = reduceSafeCellMotion(poses, frames, safety, { keepTailFrames: frames.length });
  assert.equal(untouched.frames.length, frames.length, 'a fully protected tail must not be shortcut');
  assert.equal(untouched.reducedBy, 0);
  frames.forEach((frame, index) => assert.deepEqual(untouched.frames[index], frame, `frame ${index} was altered despite keepTailFrames`));
});

test('reduceSafeCellMotion still shortens a route once the fully protected reference above is relaxed', () => {
  const { poses, frames } = planReducibleRoute('stack');
  const shortcut = reduceSafeCellMotion(poses, frames, safety);
  assert.ok(shortcut.frames.length <= frames.length, 'default reduction must not add steps');
  assert.equal(shortcut.sourceFrames, frames.length);
  assert.equal(shortcut.reducedBy, frames.length - shortcut.frames.length);
});

test('reduceSafeCellMotion requiredFrameIndexes forces a stop the unconstrained reducer would otherwise skip', () => {
  const { poses, frames } = planReducibleRoute('stack');
  const unconstrained = reduceSafeCellMotion(poses, frames, safety);
  const midIndex = Math.floor(frames.length / 2);
  const pinned = reduceSafeCellMotion(poses, frames, safety, { requiredFrameIndexes: [midIndex] });
  assert.ok(pinned.frames.length >= unconstrained.frames.length, 'pinning a mid-route frame cannot make the path shorter');
  const hitsMidpoint = pinned.frames.some((frame) => JSON.stringify(frame) === JSON.stringify(frames[midIndex]));
  assert.ok(hitsMidpoint, 'the pinned mid-route frame must appear verbatim in the reduced output');
  let previous = poses.map(({ q }) => q);
  for (const frame of pinned.frames) {
    assert.equal(evaluateCellSafety(frame.map((q, index) => ({ q, arm: poses[index].arm })), safety).safe, true);
    previous = frame;
  }
});

test('policyRecipeFor falls back to the default geometric recipe for a task with no matching id', () => {
  assert.equal(policyRecipeFor(undefined).kind, 'safe-IK');
  assert.equal(policyRecipeFor({ id: 'not-a-real-task' }).kind, 'safe-IK');
  assert.equal(policyRecipeFor(compiled.workflows.find((task) => task.id === 'fold')).kind, 'cloth-fold');
});

test('scoreTaskStages returns the neutral default for non-cloth tasks, with or without a cloth sim', () => {
  const reach = compiled.workflows.find((task) => task.id === 'reach');
  const cloth = new ClothSimulator({ columns: 8, rows: 6 });
  assert.deepEqual(scoreTaskStages(reach, { cloth, tips: [] }), { reward: 0, stage: 'route', complete: false });
  assert.deepEqual(scoreTaskStages(reach, {}), { reward: 0, stage: 'route', complete: false });
});

test('scoreTaskStages returns the neutral default for the fold task when no cloth sim is supplied', () => {
  const fold = compiled.workflows.find((task) => task.id === 'fold');
  assert.deepEqual(scoreTaskStages(fold, {}), { reward: 0, stage: 'route', complete: false });
});

test('scoreTaskStages reaches full reward and the final settle stage once every milestone is met', () => {
  const fold = compiled.workflows.find((task) => task.id === 'fold');
  const cloth = new ClothSimulator({ columns: 8, rows: 6 });
  cloth.wasCaptured = [true, true];
  cloth.released = [true, true];
  cloth.tablePinA = { x: -0.75, y: -0.6, z: cloth.tableZ };
  cloth.getFoldMetrics = () => ({ folded: true, frontDistance: 0, stretchError: 0 });
  const result = scoreTaskStages(fold, { cloth, tips: [[250, 180, 15], [300, 180, 20]] });
  assert.equal(result.complete, true);
  assert.equal(result.stage, 'settle without overstretch');
  assert.ok(Math.abs(result.reward - 1) < 1e-9, `expected reward 1, got ${result.reward}`);
});
