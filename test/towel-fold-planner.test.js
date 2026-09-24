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
  assert.equal(motion.reducedBy, 0, 'Dynamic fold stages retain their physical-rate frames');

  let previous = poses.map(({ q }) => q);
  for (const frame of motion.frames) {
    const assessment = evaluateCellSafety(frame.map((q, index) => ({ q, arm: index ? ARM_B : ARM })), safety);
    assert.equal(assessment.safe, true, `Fold frame crossed ${assessment.reason}`);
    frame.forEach((q, armIndex) => q.forEach((value, joint) => {
      assert.ok(Math.abs(value - previous[armIndex][joint]) <= ARM.maxActionDelta + 1e-9, 'Fold frame exceeded delta cap');
    }));
    previous = frame;
  }

  // One gripper command per frame: both close on the corners, A opens before
  // B lays the fold over its corner, and B holds to the end.
  assert.equal(motion.grips.length, motion.frames.length);
  const firstClosed = motion.grips.findIndex(([a, b]) => a && b);
  assert.ok(firstClosed > 0, 'both grippers close');
  const tipAt = (index, arm) => forwardKinematics(motion.frames[index][arm], arm ? ARM_B : ARM).points.at(-1);
  assert.ok(distance(tipAt(firstClosed, 0), [250, 180, 4]) < 1.5, 'Arm A closes on the front-left corner');
  assert.ok(distance(tipAt(firstClosed, 1), [400, 180, 4]) < 1.5, 'Arm B closes on the front-right corner');
  const aOpens = motion.grips.findIndex(([a], index) => index > firstClosed && !a);
  assert.ok(aOpens > firstClosed && motion.grips.slice(aOpens).every(([a, b]) => !a && b), 'A releases for good while B keeps hold');

  // While B holds cloth its tool moves in short straight steps: no frame
  // jumps, and it never wanders beyond the fold's own span.
  for (let index = firstClosed + 1; index < motion.frames.length; index += 1) {
    const step = distance(tipAt(index, 1), tipAt(index - 1, 1));
    assert.ok(step < 4, `carried corner moved ${step.toFixed(1)} px in one frame`);
    const [x, y, z] = tipAt(index, 1);
    assert.ok(x > 245 && x < 405 && Math.abs(y - 180) < 2 && z < 60, `carried corner left the fold path at (${x.toFixed(0)}, ${y.toFixed(0)}, ${z.toFixed(0)})`);
  }

  const tipA = forwardKinematics(previous[0], ARM).points.at(-1);
  const tipB = forwardKinematics(previous[1], ARM_B).points.at(-1);
  assert.ok(distance(tipA, forwardKinematics(HOME_POSE, ARM).points.at(-1)) < 2, 'Arm A retracts clear after pinning');
  assert.ok(tipB[0] < 325, 'Arm B folded across towel midline');
  assert.ok(distance(tipB, [250, 180, 6]) < 2, 'Arm B places the folded edge at the pinned-side target');
});

test('planTowelFoldMotion folds along a different corner pair when profile.cornerA/cornerB override the default', () => {
  const safety = compiled.environment.safety;
  const poses = [{ q: [...HOME], arm: ARM }, { q: [...HOME], arm: ARM_B }];
  // Task 12's "inset" vertex on both sides - one grid column in from Task 7's
  // true corners, still left-right and still each arm's own natural side.
  // This is the actual validated-safe range: this rig's two fixed-base arms
  // cannot safely reach the towel's back corners, or swap which one pins and
  // which carries - both were tried and rejected by the same safety check a
  // live run would hit (see the "Task 8 base" decision in this session).
  const colPx = (1.5 / 14) * 100;
  const cornerA = [250 + colPx, 180];
  const cornerB = [400 - colPx, 180];
  const motion = planTowelFoldMotion(poses, safety, { cornerA, cornerB });
  assert.ok(motion, 'An inset corner pair should still plan successfully');
  assert.ok(motion.frames.length > 50);

  let previous = poses.map(({ q }) => q);
  for (const frame of motion.frames) {
    const assessment = evaluateCellSafety(frame.map((q, index) => ({ q, arm: index ? ARM_B : ARM })), safety);
    assert.equal(assessment.safe, true, `Fold frame crossed ${assessment.reason}`);
    previous = frame;
  }

  const tipAt = (index, arm) => forwardKinematics(motion.frames[index][arm], arm ? ARM_B : ARM).points.at(-1);
  const firstClosed = motion.grips.findIndex(([a, b]) => a && b);
  assert.ok(distance(tipAt(firstClosed, 0), [...cornerA, 4]) < 1.5, 'Arm A closes on the requested pin vertex');
  assert.ok(distance(tipAt(firstClosed, 1), [...cornerB, 4]) < 1.5, 'Arm B closes on the requested carried vertex');

  // While carried, the corner travels toward cornerA (this fold's axis is
  // still x, same as Task 7's default) and stays within the fold's span.
  for (let index = firstClosed + 1; index < motion.frames.length; index += 1) {
    const [x, y] = tipAt(index, 1);
    assert.ok(x > cornerA[0] - 5 && x < cornerB[0] + 5, `carried corner left the fold span at x=${x.toFixed(0)}`);
    assert.ok(Math.abs(y - cornerA[1]) < 2, `carried corner drifted in y to ${y.toFixed(0)}`);
  }

  const tipB = forwardKinematics(previous[1], ARM_B).points.at(-1);
  assert.ok(distance(tipB, [...cornerA, 6]) < 2, 'Arm B places the folded edge at the requested pin vertex');
});
