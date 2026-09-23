import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, ARM_B, distance, forwardKinematics, guidedStep, HOME_POSE, projectToReachableWorkspace, solveInverseKinematics } from '../src/core.js';

// Split out of core.test.js so its dense workspace sweep runs in its own
// process, in parallel with the rest of the suite, instead of adding to a
// single file's sequential total.

test('guidance recovers from representative manual poses across the workspace', () => {
  const starts = [HOME_POSE, [0, 0, 0, 0, 0, 0], [1.2, 1.1, -0.8, 0.6, -0.4, 0.2], [-1.2, 0.5, -1.5, -0.6, -0.9, -0.2]];
  for (const arm of [ARM, ARM_B]) {
    for (let x = 100; x <= 620; x += 130) for (let y = 110; y <= 390; y += 90) for (const z of [10, 80, 150]) {
      const target = projectToReachableWorkspace([x, y, z], compiled.environment.safety, arm);
      const plan = solveInverseKinematics(target, arm);
      // Guidance converges on the plan; whether the plan reaches the target is
      // the solver's business, and the task-goal test in core.test.js covers that.
      const planned = forwardKinematics(plan.q, arm).points.at(-1);
      for (const start of starts) {
        let q = [...start];
        for (let step = 0; step < compiled.environment.max_steps; step += 1) {
          const update = guidedStep(q, target, arm, plan);
          assert.ok(update.action.every((delta) => Math.abs(delta) <= arm.maxActionDelta + 1e-9));
          q = update.q;
        }
        assert.ok(distance(forwardKinematics(q, arm).points.at(-1), planned) < 2, `${start} did not converge on the plan for ${target}`);
      }
    }
  }
});
