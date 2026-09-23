import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, ARM_B, distance, evaluateCellSafety, forwardKinematics, HOME_POSE, planTowelFoldMotion } from '../src/core.js';

// Split out of core.test.js so its full fold-trajectory search runs in its
// own process, in parallel with the rest of the suite, instead of adding to
// a single file's sequential total.
const HOME = HOME_POSE;

test('planTowelFoldMotion produces a valid collision-free bimanual folding trajectory', () => {
  const safety = compiled.environment.safety;
  const poses = [{ q: [...HOME], arm: ARM }, { q: [...HOME], arm: ARM_B }];
  const motion = planTowelFoldMotion(poses, safety);
  assert.ok(motion, 'Towel fold motion should be planned');
  assert.ok(motion.frames.length > 50, 'Folding motion has multiple trajectory phases');

  let previous = poses.map(({ q }) => q);
  for (const frame of motion.frames) {
    const assessment = evaluateCellSafety(frame.map((q, index) => ({ q, arm: index ? ARM_B : ARM })), safety);
    assert.equal(assessment.safe, true, `Fold frame crossed ${assessment.reason}`);
    frame.forEach((q, armIndex) => q.forEach((value, joint) => {
      assert.ok(Math.abs(value - previous[armIndex][joint]) <= ARM.maxActionDelta + 1e-9, 'Fold frame exceeded delta cap');
    }));
    previous = frame;
  }

  const tipA = forwardKinematics(previous[0], ARM).points.at(-1);
  const tipB = forwardKinematics(previous[1], ARM_B).points.at(-1);
  assert.ok(distance(tipA, [250, 180, 15]) < 2, 'Arm A pinned at left edge');
  assert.ok(tipB[0] < 325, 'Arm B folded across towel midline');
  assert.ok(distance(tipA, tipB) < 70, 'Folded edge is close to pinned edge');
});
