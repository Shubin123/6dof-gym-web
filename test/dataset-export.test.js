import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { buildDatasetManifest, buildEpisodeArtifact, HALT } from '../src/core.js';

// Mirrors main.js's addTransition() shape: 6 joints + 6 reserved + xyz + a
// 4-value pose placeholder + a 3-value goal = 22 single-arm observation
// features, and 6 action deltas + 1 gripper channel = 7 action dimensions.
const fakeTransition = (step, { done = false } = {}) => ({
  index: step,
  timestamp_ms: step * 40,
  observation: [...Array(6).fill(0.1 * step), ...Array(6).fill(0), 250, 180, 15, 1, 0, 0, 0, 250, 180, 15],
  action: [0.01, 0, 0, 0, 0, 0, 0],
  action_after_safety_clamp: [0.01, 0, 0, 0, 0, 0, 0],
  task: 'reach',
  arms: 1,
  reward: -0.1,
});

const reachTask = compiled.workflows.find((workflow) => workflow.id === 'reach');
const sortTask = compiled.workflows.find((workflow) => workflow.id === 'sort');
const foldTask = compiled.workflows.find((workflow) => workflow.id === 'fold');

function makeEpisode(task, stepCount, halt = HALT.REACHED, arms = 1) {
  const transitions = Array.from({ length: stepCount }, (_, i) => fakeTransition(i));
  return buildEpisodeArtifact({ environment: compiled.environment.id, task, transitions, arms, halt });
}

test('buildDatasetManifest refuses an empty episode list with a friendly message', () => {
  const result = buildDatasetManifest({ episodes: [] });
  assert.equal(result.error, 'empty');
  assert.match(result.message, /at least one/i);
});

test('buildDatasetManifest refuses episodes with mismatched arm counts', () => {
  const single = makeEpisode(reachTask, 5, HALT.REACHED, 1);
  const bimanual = makeEpisode(foldTask, 5, HALT.REACHED, 2);
  const result = buildDatasetManifest({ episodes: [single, bimanual] });
  assert.equal(result.error, 'mixed-arms');
  assert.match(result.message, /same arm count/i);
});

test('buildDatasetManifest assembles a LeRobotDataset-shaped info/tasks/episodes/frames bundle', () => {
  const first = makeEpisode(reachTask, 3, HALT.REACHED);
  const second = makeEpisode(sortTask, 2, HALT.BUDGET);
  const third = makeEpisode(reachTask, 4, HALT.REACHED); // repeats the first task text
  const manifest = buildDatasetManifest({ episodes: [first, second, third], fps: 25 });

  assert.equal(manifest.error, undefined);
  assert.equal(manifest.info.fps, 25);
  assert.equal(manifest.info.total_episodes, 3);
  assert.equal(manifest.info.total_frames, 9);
  assert.equal(manifest.info.robot_type, 'armlab-6dof');
  assert.deepEqual(manifest.info.features['observation.state'].shape, [22]);
  assert.deepEqual(manifest.info.features.action.shape, [7]);

  // Repeated task text is deduplicated into one task_index.
  assert.equal(manifest.tasks.length, 2);
  assert.equal(manifest.episodes.length, 3);
  assert.equal(manifest.episodes[0].task_index, manifest.episodes[2].task_index);
  assert.equal(manifest.episodes[1].task_index === manifest.episodes[0].task_index, false);
  assert.equal(manifest.episodes[1].halt, HALT.BUDGET);

  // Frame indexing is contiguous within an episode and globally unique.
  assert.equal(manifest.frames.length, 9);
  assert.deepEqual(manifest.frames.map((frame) => frame.index), [0, 1, 2, 3, 4, 5, 6, 7, 8]);
  const secondEpisodeFrames = manifest.frames.filter((frame) => frame.episode_index === 1);
  assert.deepEqual(secondEpisodeFrames.map((frame) => frame.frame_index), [0, 1]);

  // Only the last frame of a reached episode is marked a success.
  const firstEpisodeFrames = manifest.frames.filter((frame) => frame.episode_index === 0);
  assert.deepEqual(firstEpisodeFrames.map((frame) => frame['next.done']), [false, false, true]);
  assert.deepEqual(firstEpisodeFrames.map((frame) => frame['next.success']), [false, false, true]);
  const budgetEpisodeFrames = manifest.frames.filter((frame) => frame.episode_index === 1);
  assert.equal(budgetEpisodeFrames.at(-1)['next.success'], false, 'a budget halt is not a success');
});
