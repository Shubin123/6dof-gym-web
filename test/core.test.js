import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, buildEpisodeArtifact, distance, forwardKinematics, guidedStep, MAX_EPISODE_TRANSITIONS, projectToReachableWorkspace, solveInverseKinematics } from '../src/core.js';

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
    const plan = solveInverseKinematics(goal);
    assert.ok(plan.distance < 2, `${goal} should have a valid IK plan`);
    const before = distance(forwardKinematics(q).points.at(-1), goal);
    for (let step = 0; step < compiled.environment.max_steps; step += 1) {
      const update = guidedStep(q, goal, ARM, plan); q = update.q;
      assert.ok(update.action.every((delta) => Math.abs(delta) <= ARM.maxActionDelta + 1e-9));
    }
    const after = distance(forwardKinematics(q).points.at(-1), goal);
    assert.ok(after < 12, `${goal} finished ${after.toFixed(1)}px from target`);
    assert.ok(after < before, 'guidance should reduce target distance');
  }
});

test('guidance recovers from representative manual poses throughout the workspace', () => {
  const starts = [
    [-0.45, 0.2, 0.3, -0.2, -0.1, 0.15], [0, 0, 0, 0, 0, 0],
    [1.2, -1.1, 0.8, -0.6, 0.4, -0.2], [-1.2, 1.1, -0.8, 0.6, -0.4, 0.2],
  ];
  const targets = [];
  for (let x = 80; x <= 640; x += 70) for (let y = 90; y <= 400; y += 62) {
    targets.push(projectToReachableWorkspace([x, y], compiled.environment.safety));
  }
  for (const target of targets) {
    const plan = solveInverseKinematics(target);
    assert.ok(plan.distance < 2, `${target} should be plannable`);
    for (const start of starts) {
      let q = [...start];
      for (let step = 0; step < compiled.environment.max_steps; step += 1) {
        const update = guidedStep(q, target, ARM, plan);
        assert.ok(update.action.every((delta) => Math.abs(delta) <= ARM.maxActionDelta + 1e-9));
        q = update.q;
      }
      assert.ok(distance(forwardKinematics(q).points.at(-1), target) < 2, `${start} did not recover to ${target}`);
    }
  }
});

test('dragged guidance target is projected safely into the reachable workspace', () => {
  const target = projectToReachableWorkspace([760, 0], compiled.environment.safety);
  assert.ok(distance(target, ARM.base) <= compiled.environment.safety.max_reach_px + 1e-9);
});

test('episode exports retain optional replayable voice fields', () => {
  const artifact = buildEpisodeArtifact({ environment: 'Arm6-Reach-v0', task: compiled.workflows[0], transitions: [], voice: { transcript: 'place the cube', mimeType: 'audio/webm', dataUrl: 'data:audio/webm;base64,AA==' } });
  assert.equal(artifact.schema, 'armlab-episode-preview/v0.2');
  assert.equal(artifact.voice.transcript, 'place the cube');
  assert.match(artifact.voice.audio_data_url, /^data:audio\/webm;base64,/);
});

test('browser episode buffer has a finite task-aligned bound', () => {
  assert.equal(MAX_EPISODE_TRANSITIONS, compiled.environment.max_steps);
});
