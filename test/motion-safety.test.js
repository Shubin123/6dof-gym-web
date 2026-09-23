import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, ARM_B, distance, evaluateCellSafety, forwardKinematics, HOME_POSE, planSafeCellMotion, projectToReachableWorkspace, reduceSafeCellMotion, solveInverseKinematics } from '../src/core.js';

// Split out of core.test.js so its heavy per-workflow search runs in its own
// process, in parallel with the rest of the suite, instead of adding to a
// single file's sequential total.
const HOME = HOME_POSE;
const armsOf = (workflow) => (workflow.arms === 2 ? [[ARM, workflow.goal], [ARM_B, workflow.goal_b]] : [[ARM, workflow.goal]]);
const targetOf = (workflow, arm, goal) => projectToReachableWorkspace([...goal, workflow.goal_height], compiled.environment.safety, arm);

test('every policy path stays over the floor, inside its edges, and clear of the other arm', () => {
  const safety = compiled.environment.safety;
  for (const workflow of compiled.workflows) {
    const definitions = armsOf(workflow);
    const plans = [];
    definitions.forEach(([arm, goal], index) => {
      const otherPoses = plans.map((plan, otherIndex) => ({ q: plan.q, arm: definitions[otherIndex][0] }));
      plans[index] = solveInverseKinematics(targetOf(workflow, arm, goal), arm, safety, otherPoses);
    });
    const poses = definitions.map(([arm]) => ({ q: [...HOME], arm }));
    const motion = planSafeCellMotion(poses, plans.map((plan) => plan.q), safety);
    assert.ok(motion, `${workflow.id} should have a safe policy path`);
    assert.ok(motion.frames.length <= workflow.horizon_steps, `${workflow.id} needs ${motion.frames.length}/${workflow.horizon_steps} steps`);
    const reduced = reduceSafeCellMotion(poses, motion.frames, safety);
    assert.ok(reduced, `${workflow.id} should retain a safe reduced policy path`);
    assert.ok(reduced.frames.length <= motion.frames.length, `${workflow.id} reducer must not add steps`);
    let previous = poses.map(({ q }) => q);
    for (const frame of reduced.frames) {
      const assessment = evaluateCellSafety(frame.map((q, index) => ({ q, arm: definitions[index][0] })), safety);
      assert.equal(assessment.safe, true, `${workflow.id} crossed its ${assessment.reason} boundary`);
      frame.forEach((q, armIndex) => q.forEach((value, joint) => {
        assert.ok(Math.abs(value - previous[armIndex][joint]) <= definitions[armIndex][0].maxActionDelta + 1e-9, `${workflow.id} exceeded the joint-delta cap`);
      }));
      previous = frame;
    }
    const finalErrors = definitions.map(([arm, goal], index) => distance(forwardKinematics(previous[index], arm).points.at(-1), targetOf(workflow, arm, goal)));
    assert.ok(finalErrors.every((error) => error < 2), `${workflow.id} did not finish at its goals: ${finalErrors}`);
  }
});
