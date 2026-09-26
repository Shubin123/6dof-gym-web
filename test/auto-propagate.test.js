import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, clamp, evaluateCellSafety, HOME_POSE } from '../src/core.js';
import { RigidScene } from '../src/rigid.js';
import { goalCenter, planRigidTask, rigidOutcome } from '../src/rigid-tasks.js';
import {
  autoPropagateOutcome,
  planAutoPropagateTask,
  PROPAGATE_BUILD_WORKSPACE,
  PROPAGATE_MODULES,
  PROPAGATE_STEP_BUDGET,
  scoreAutoPropagateStages,
} from '../src/auto-propagate.js';
import { policyRecipeFor, scoreTaskStages } from '../src/task-policies.js';

const safety = compiled.environment.safety;
const task = compiled.workflows.find((w) => w.id === 'auto_propagate');
const family = compiled.families.find((f) => f.id === 'auto_propagate');

test('Category "auto_propagate" and Task 18 specification', () => {
  assert.ok(family, 'auto_propagate category must exist in compiled.families');
  assert.equal(family.id, 'auto_propagate');
  assert.equal(family.name, 'Auto propagate');
  assert.ok(family.level.length > 3);
  assert.ok(family.blurb.length > 20);

  assert.ok(task, 'Task 18 (auto_propagate) must exist in compiled.workflows');
  assert.equal(task.id, 'auto_propagate');
  assert.equal(task.number, '18');
  assert.equal(task.family, 'auto_propagate');
  assert.equal(task.arms, 1, 'Single autonomous arm builds the replica arm');
  assert.ok(task.rigid, 'Task 18 declares rigid physics scene');
  assert.equal(task.rigid.goal.type, 'propagate');
  assert.equal(task.rigid.goal.marker, 'podium', 'Goal marker is base mounting podium');
  assert.ok(task.curriculum.length >= 3);
  assert.ok(task.failure_modes.length >= 2);
  assert.ok(task.guardrail.length > 10);
  assert.ok(task.summary.length > 20);
  assert.ok(task.success.length > 20);
  assert.ok(task.sensors.length > 10);
  assert.ok(task.horizon_steps <= compiled.environment.max_steps);
});

test('User-defined deployment area repositioning and workspace bounds', () => {
  const { minX, maxX, minY, maxY } = PROPAGATE_BUILD_WORKSPACE;
  assert.ok(minX >= safety.goal_workspace[0], 'minX is within safety bounds');
  assert.ok(maxX <= safety.goal_workspace[1], 'maxX is within safety bounds');
  assert.ok(minY >= safety.goal_workspace[2], 'minY is within safety bounds');
  assert.ok(maxY <= safety.goal_workspace[3], 'maxY is within safety bounds');

  // Verify repositioning clamping
  const testPicks = [
    [100, 100], // Far out -> clamped to [minX, minY]
    [600, 500], // Far out -> clamped to [maxX, maxY]
    [430, 270], // Inside -> untouched
  ];

  for (const [x, y] of testPicks) {
    const clampedX = clamp(x, minX, maxX);
    const clampedY = clamp(y, minY, maxY);
    assert.ok(clampedX >= minX && clampedX <= maxX);
    assert.ok(clampedY >= minY && clampedY <= maxY);
  }

  // goalCenter resolves propagate position
  assert.deepEqual(goalCenter(task.rigid), task.rigid.goal.position);
});

test('Multi-module auto-propagate motion plan is collision-free and rate-capped across user sites', () => {
  const testSites = [
    [400, 250],
    [440, 280],
    [460, 270],
  ];

  for (const site of testSites) {
    const customRigid = structuredClone(task.rigid);
    customRigid.goal.position = [...site];
    const scene = new RigidScene(customRigid, { arms: 1 });
    const plan = planAutoPropagateTask(customRigid, scene, { q: [...HOME_POSE], arm: ARM }, safety);

    assert.ok(plan, `Must find valid assembly plan for site (${site[0]}, ${site[1]})`);
    assert.ok(plan.frames.length > 700, `Has full multi-stage frames: ${plan.frames.length}`);
    assert.ok(plan.frames.length <= PROPAGATE_STEP_BUDGET, `Fits within step budget: ${plan.frames.length}`);
    assert.equal(plan.frames.length, plan.grips.length);
    assert.equal(plan.stageEnds.length, 3, 'Marks stage milestone for each module');

    let prevQ = HOME_POSE;
    for (let f = 0; f < plan.frames.length; f += 1) {
      const [q] = plan.frames[f];
      assert.ok(
        q.every((val, joint) => Math.abs(val - prevQ[joint]) <= ARM.maxActionDelta + 1e-9),
        `Frame ${f} exceeded joint action delta limit`,
      );
      const safeCheck = evaluateCellSafety([{ q, arm: ARM }], safety);
      assert.equal(safeCheck.safe, true, `Frame ${f} violated cell safety: ${safeCheck.reason}`);
      prevQ = q;
    }
  }
});

test('Physics simulation constructs secondary robot arm at user-picked location', () => {
  const scene = new RigidScene(task.rigid, { arms: 1 });
  const startCenters = task.rigid.objects.map((obj) => scene.objectCenter(obj.id));

  // Verify all 3 modules initially in supply depot
  for (const center of startCenters) {
    assert.ok(Math.abs(center[0] - 420) <= 65, 'Module staged in parts depot');
    assert.ok(Math.abs(center[1] - 165) <= 25, 'Module staged in parts depot');
    assert.ok(center[2] >= 10, 'Module rests on depot floor');
  }

  // Generate plan via generic planRigidTask
  const plan = planRigidTask(task.rigid, scene, { q: [...HOME_POSE], arm: ARM }, safety);
  assert.ok(plan, 'planRigidTask successfully routes to auto-propagation');

  // Step physics simulation
  for (let f = 0; f < plan.frames.length; f += 1) {
    const [q] = plan.frames[f];
    const grip = plan.grips[f][0];
    scene.step({ arms: [{ q, arm: ARM }], grips: [grip] });
  }

  // Outcome check
  const outcome = rigidOutcome(task.rigid, scene);
  assert.equal(outcome.success, true, 'Auto-propagation outcome scored as success');
  assert.equal(outcome.placed, true, 'All 3 modules placed and stacked');
  assert.equal(outcome.resting, true, 'Assembly came to rest on table');
  assert.equal(outcome.held, false, 'Primary arm released all modules');

  const cBase = scene.objectCenter('arm_base');
  const cLink = scene.objectCenter('arm_link');
  const cTool = scene.objectCenter('arm_tool');
  const target = task.rigid.goal.position;

  // Stacking geometry
  assert.ok(Math.hypot(cBase[0] - target[0], cBase[1] - target[1]) < 8, 'Arm Base at target site');
  assert.ok(Math.abs(cBase[2] - 15) < 3, 'Base pedestal seated on table (z ~ 15 mm)');

  assert.ok(Math.hypot(cLink[0] - target[0], cLink[1] - target[1]) < 10, 'Arm Link aligned over base');
  assert.ok(Math.abs(cLink[2] - 43) < 4, 'Linkage stacked onto base (z ~ 43 mm)');

  assert.ok(Math.hypot(cTool[0] - target[0], cTool[1] - target[1]) < 12, 'Gripper tool aligned over link');
  assert.ok(Math.abs(cTool[2] - 67) < 4, 'Toolhead docked onto linkage (z ~ 67 mm)');
});

test('Stage rewards and telemetry score auto-propagate progression', () => {
  const recipe = policyRecipeFor(task);
  assert.equal(recipe.kind, 'rigid-assembly');
  assert.equal(recipe.stages.length, 4);

  const scene = new RigidScene(task.rigid, { arms: 1 });
  const initScore = scoreTaskStages(task, { scene, rigid: task.rigid });
  assert.equal(initScore.reward, 0, 'Initial reward is 0 before assembly starts');
  assert.equal(initScore.stage, '1/4 · Install shoulder turret onto podium');
  assert.equal(initScore.complete, false);

  // Run full trajectory
  const plan = planRigidTask(task.rigid, scene, { q: [...HOME_POSE], arm: ARM }, safety);
  for (let f = 0; f < plan.frames.length; f += 1) {
    const [q] = plan.frames[f];
    const grip = plan.grips[f][0];
    scene.step({ arms: [{ q, arm: ARM }], grips: [grip] });
  }

  const finalScore = scoreTaskStages(task, { scene, rigid: task.rigid });
  assert.equal(finalScore.reward, 1.0, 'Final reward reaches 100%');
  assert.equal(finalScore.complete, true, 'Auto-propagation completed');
});

test('Full 6-DOF sibling arm kinematic structure and ready posture', async () => {
  const { forwardKinematics } = await import('../src/core.js');
  const { SIBLING_ARM_READY_POSE } = await import('../src/auto-propagate.js');

  assert.equal(SIBLING_ARM_READY_POSE.length, 6, 'Sibling arm has 6 joint coordinates');
  const siblingArm = {
    id: 'Sibling',
    base: [440, 280],
    baseHeight: 90,
    lengths: [110, 100, 90, 80, 75, 70],
    axes: ['yaw', 'pitch', 'pitch', 'yaw', 'pitch', 'yaw'],
    jointLimit: 1.7,
    yawLimit: Math.PI,
    mirror: true,
  };
  const { points, forward, up } = forwardKinematics(SIBLING_ARM_READY_POSE, siblingArm);
  assert.equal(points.length, 7, '6-DOF arm has 7 kinematic points (base + 6 links)');
  assert.ok(points.at(-1)[2] > 50, 'End-effector rests above table');
  assert.equal(forward.length, 3);
  assert.equal(up.length, 3);
});
