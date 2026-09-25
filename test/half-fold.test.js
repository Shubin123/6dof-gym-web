// Task 12's half fold, end to end. Its own file: the carry search takes a few
// seconds per direction and would otherwise lengthen another file's run.
import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, ARM_B, evaluateCellSafety, forwardKinematics, HOME_POSE } from '../src/core.js';
import { ClothSimulator, clothCorners } from '../src/cloth.js';
import { planHalfFoldMotion } from '../src/half-fold.js';
import { bootstrapPolicy, scoreTaskStages } from '../src/task-policies.js';

const safety = compiled.environment.safety;
const task = compiled.workflows.find((workflow) => workflow.id === 'fold_custom');
// Task 12's towel sits on the cell midline (main.js clothOrigin).
const ORIGIN = [380, 240];
const ARMS = [ARM, ARM_B];

function setUp(direction) {
  const cloth = new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2 });
  const k = clothCorners(cloth.columns, cloth.rows);
  const back = [k.backLeft, k.backRight];
  const front = [k.frontLeft, k.frontRight];
  const [carried, partners] = direction === 'back-to-front' ? [back, front] : [front, back];
  cloth.setAnchors(carried[0], carried[1], { foldAxis: 'rows', pinOnRelease: false, partners });
  cloth.reset();
  const scene = (idx) => [ORIGIN[0] + cloth.restPositions[idx * 3] * 100, ORIGIN[1] + cloth.restPositions[idx * 3 + 1] * 100];
  const carry = carried.map((idx, arm) => [scene(idx), scene(partners[arm])]);
  return { cloth, carried, partners, carry };
}

/** Drive the cloth through a plan exactly as main.js does: two cloth steps per control frame. */
function run(cloth, plan) {
  plan.frames.forEach((frame, index) => {
    const [a, b] = frame.map((q, arm) => {
      const tip = forwardKinematics(q, ARMS[arm]).points.at(-1);
      return { x: (tip[0] - ORIGIN[0]) / 100, y: (tip[1] - ORIGIN[1]) / 100, z: tip[2] / 100 };
    });
    for (let step = 0; step < 2; step += 1) cloth.step({ targetA: a, targetB: b, grips: plan.grips[index] });
  });
}

for (const direction of ['back-to-front', 'front-to-back']) {
  test(`Task 12 half fold (${direction}): four corners become two`, () => {
    const { cloth, carried, partners, carry } = setUp(direction);
    const poses = ARMS.map((arm) => ({ q: [...HOME_POSE], arm }));
    const plan = planHalfFoldMotion(poses, safety, { ...bootstrapPolicy(task).profile, carry });
    assert.ok(plan, 'a safe half-fold plan exists');

    let previous = poses.map(({ q }) => q);
    for (const frame of plan.frames) {
      assert.ok(evaluateCellSafety(frame.map((q, arm) => ({ q, arm: ARMS[arm] })), safety).safe, 'every frame is cell-safe, both arms together');
      frame.forEach((q, arm) => q.forEach((value, joint) => {
        assert.ok(Math.abs(value - previous[arm][joint]) <= ARM.maxActionDelta + 1e-9, 'every frame respects the joint-delta cap');
      }));
      previous = frame;
    }
    assert.ok(plan.grips.some(([a, b]) => a && b), 'both grippers close together');
    assert.deepEqual(plan.grips.at(-1), [false, false], 'both corners are let go');

    run(cloth, plan);
    assert.deepEqual(cloth.wasCaptured, [true, true], 'each arm grasped its corner');
    const metrics = cloth.getHalfFoldMetrics();
    metrics.cornerGaps.forEach((gap, arm) => assert.ok(gap < 0.15, `arm ${arm} corner is ${(gap * 10).toFixed(1)} cm from its partner`));
    assert.ok(metrics.edgeGap < 0.2, `folded edge is ${(metrics.edgeGap * 10).toFixed(1)} cm from the edge under it`);
    // Each carried corner lies on top of the corner it was laid on.
    carried.forEach((idx, arm) => assert.ok(cloth.positions[idx * 3 + 2] > cloth.positions[partners[arm] * 3 + 2]));
    const score = scoreTaskStages(task, { cloth });
    assert.ok(score.complete, `stage ${score.stage}, reward ${score.reward.toFixed(2)}`);
  });
}

test('the half-fold score does not call a flat towel folded', () => {
  const { cloth } = setUp('back-to-front');
  const score = scoreTaskStages(task, { cloth });
  assert.equal(score.complete, false);
  assert.equal(score.stage, 'secure both corners');
});
