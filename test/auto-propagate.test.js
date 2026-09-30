import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, ARM_B, clamp, evaluateCellSafety, forwardKinematics, HOME_POSE, jacobianSingularValues, linkBendAngles } from '../src/core.js';
import { RigidScene } from '../src/rigid.js';
import { goalCenter, planRigidTask, rigidOutcome } from '../src/rigid-tasks.js';
import {
  autoPropagateOutcome,
  planAutoPropagateTask,
  PROPAGATE_BUILD_WORKSPACE,
  PROPAGATE_MODULES,
  PROPAGATE_STEP_BUDGET,
  propagateSafety,
  scoreAutoPropagateStages,
  SIBLING_ARM,
  SIBLING_ARM_READY_POSE,
  SIBLING_PARK_POSE,
  siblingArmAt,
  siblingPoseAt,
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
  const { minX, maxX, minY, maxY } = PROPAGATE_BUILD_WORKSPACE;
  const testSites = [
    [minX, minY],
    [440, 280],
    [maxX, maxY],
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
    assert.equal(plan.stageEnds.length, 4, 'Marks a milestone for each module and for commissioning');

    // Every primary frame keeps the bend, self, pedestal, podium, and
    // module-stack limits of the stage it belongs to.
    let prevQ = HOME_POSE;
    let stage = 0;
    for (let f = 0; f < plan.frames.length; f += 1) {
      const [q] = plan.frames[f];
      assert.ok(
        q.every((val, joint) => Math.abs(val - prevQ[joint]) <= ARM.maxActionDelta + 1e-9),
        `Frame ${f} exceeded joint action delta limit`,
      );
      const safeCheck = evaluateCellSafety([{ q, arm: ARM }], propagateSafety(customRigid, safety, { stacked: Math.min(stage, 3) }));
      assert.equal(safeCheck.safe, true, `Frame ${f} violated cell safety: ${safeCheck.reason} ${safeCheck.obstacle ?? ''}`);
      prevQ = q;
      if (f === plan.stageEnds[stage]) stage += 1;
    }

    // Arm-to-arm: the sibling's calibration sweep against the parked primary.
    const primary = { q: plan.frames.at(-1)[0], arm: ARM };
    const sibling = siblingArmAt(site);
    assert.deepEqual(plan.sibling.frames[0], [...SIBLING_PARK_POSE]);
    plan.sibling.frames.at(-1).forEach((value, joint) => assert.ok(Math.abs(value - SIBLING_ARM_READY_POSE[joint]) < 1e-9, 'Sweep ends in the ready pose'));
    assert.equal(plan.sibling.startFrame + plan.sibling.frames.length, plan.frames.length);
    let prevS = plan.sibling.frames[0];
    for (const q of plan.sibling.frames) {
      assert.ok(q.every((val, joint) => Math.abs(val - prevS[joint]) <= ARM.maxActionDelta + 1e-9), 'Sibling joint step within rate cap');
      const cell = evaluateCellSafety([primary, { q, arm: sibling }], propagateSafety(customRigid, safety));
      assert.equal(cell.safe, true, `Sibling sweep violated ${cell.reason}`);
      assert.ok(cell.armClearance >= safety.arm_clearance_px, 'Sibling keeps arm-to-arm clearance');
      assert.ok(jacobianSingularValues(q, sibling)[0] >= task.rigid.constraints.singularity_margin, 'Sibling stays clear of singularities');
      prevS = q;
    }
  }
});

test('Physics simulation constructs secondary robot arm at user-picked location', () => {
  const scene = new RigidScene(task.rigid, { arms: 1 });
  const startCenters = task.rigid.objects.map((obj) => scene.objectCenter(obj.id));

  // Verify all 3 modules initially in supply depot
  for (const center of startCenters) {
    const depot = task.rigid.fixtures.find((fixture) => fixture.id === 'depot');
    assert.ok(Math.abs(center[0] - depot.position[0]) <= depot.inner[0] / 2, 'Module staged in parts depot');
    assert.ok(Math.abs(center[1] - depot.position[1]) <= depot.inner[1] / 2, 'Module staged in parts depot');
    assert.ok(center[2] >= 10 && center[2] < 20, 'Module rests on depot floor');
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
  assert.ok(Math.abs(cBase[2] - 105) < 3, 'Shoulder turret seated on the 90 mm podium top (z ~ 105 mm)');

  assert.ok(Math.hypot(cLink[0] - target[0], cLink[1] - target[1]) < 10, 'Arm Link aligned over base');
  assert.ok(Math.abs(cLink[2] - 133) < 4, 'Boom stacked onto turret (z ~ 133 mm)');

  assert.ok(Math.hypot(cTool[0] - target[0], cTool[1] - target[1]) < 12, 'Gripper tool aligned over link');
  assert.ok(Math.abs(cTool[2] - 157) < 4, 'Toolhead docked onto boom (z ~ 157 mm)');
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

test('Full 6-DOF sibling arm kinematic structure and ready posture', () => {
  assert.equal(SIBLING_ARM_READY_POSE.length, 6, 'Sibling arm has 6 joint coordinates');
  assert.deepEqual(SIBLING_ARM.lengths, ARM.lengths, 'Sibling is the same manipulator as the primary');
  assert.equal(SIBLING_ARM.baseHeight, ARM.baseHeight, 'Sibling stands on a column as tall as the primary pedestal');
  const { points, forward, up } = forwardKinematics(SIBLING_ARM_READY_POSE, siblingArmAt([440, 280]));
  assert.equal(points.length, 7, '6-DOF arm has 7 kinematic points (base + 6 links)');
  assert.ok(points.at(-1)[2] > 50, 'End-effector rests above table');
  assert.equal(forward.length, 3);
  assert.equal(up.length, 3);
  assert.ok(Math.max(...linkBendAngles(points)) <= task.rigid.constraints.bend_limit_rad, 'Ready pose inside the bend limit');
});

test('siblingPoseAt parks before the sweep, follows it, and holds ready after', () => {
  const sibling = { startFrame: 10, frames: [[...SIBLING_PARK_POSE], [0, 0, 0, 0, 0, 0], [...SIBLING_ARM_READY_POSE]] };
  assert.deepEqual(siblingPoseAt(sibling, 0), [...SIBLING_PARK_POSE]);
  assert.deepEqual(siblingPoseAt(sibling, 11), [0, 0, 0, 0, 0, 0]);
  assert.deepEqual(siblingPoseAt(sibling, 99), [...SIBLING_ARM_READY_POSE]);
  assert.deepEqual(siblingPoseAt(null, 5), [...SIBLING_ARM_READY_POSE]);
});

test('Task 18 constraints reject bent, self-colliding, and podium-piercing poses', () => {
  const rigid = task.rigid;
  const cell = propagateSafety(rigid, safety, { stacked: 0 });
  assert.equal(evaluateCellSafety([{ q: [...HOME_POSE], arm: ARM }], cell).safe, true, 'Home pose passes every constraint');

  // Bend: a joint past the limit (but inside the hard stop) is rejected.
  const bent = [...HOME_POSE];
  bent[2] = 1.69;
  assert.equal(evaluateCellSafety([{ q: bent, arm: ARM }], cell).reason, 'bend');
  assert.equal(evaluateCellSafety([{ q: bent, arm: ARM }], safety).safe, true, 'Plain cell safety has no bend limit');

  // Podium: a pose whose tool drops into the sibling podium column.
  const podiumProbe = { ...cell, bend_limit_rad: 9, self_clearance_px: 0 };
  const [px, py] = rigid.goal.position;
  const inPodium = evaluateCellSafety([{ q: [...HOME_POSE], arm: { ...ARM, base: [px - 400, py] } }], podiumProbe);
  assert.notEqual(inPodium.reason, 'obstacle', 'Distant arm is clear of the podium');
  const obstacles = [{ id: 'podium', center: [px, py], radius: 42, topRadius: 30, height: 90, clearance: 6 }];
  const pierce = { ...safety, obstacles };
  const tipAt = forwardKinematics(HOME_POSE, ARM).points.at(-1);
  const moved = { ...pierce, obstacles: [{ ...obstacles[0], center: [tipAt[0], tipAt[1]], height: tipAt[2] + 20 }] };
  const hit = evaluateCellSafety([{ q: [...HOME_POSE], arm: ARM }], moved);
  assert.equal(hit.reason, 'obstacle');
  assert.equal(hit.obstacle, 'podium');

  // Mounted column: an arm may stand on its own pedestal without tripping it.
  const own = evaluateCellSafety([{ q: [...HOME_POSE], arm: ARM }], { ...safety, obstacles: [{ id: 'pedestal', center: ARM.base, radius: 42, topRadius: 30, height: 90 }] });
  assert.equal(own.safe, true);

  // Self: folding the arm back on itself brings non-adjacent links together.
  const folded = [0, 1, 1.7, 0, 1.7, 0];
  const self = evaluateCellSafety([{ q: folded, arm: ARM }], { self_clearance_px: 40 });
  assert.equal(self.reason, 'self');
});

test('Jacobian singular values vanish at singular postures and match across mirrored arms', () => {
  // Fully stretched: elbow and wrist both lose a direction.
  const stretched = jacobianSingularValues([0, 0, 0, 0, 0, 0], ARM);
  assert.equal(stretched.length, 6);
  assert.ok(stretched[0] < 1e-6 && stretched[1] < 1e-6, 'straight arm is doubly singular');
  // Home is well-conditioned, and a mirror image has the same values.
  const home = jacobianSingularValues(HOME_POSE, ARM);
  assert.ok(home[0] > 0.05);
  jacobianSingularValues(HOME_POSE, ARM_B).forEach((value, i) => assert.ok(Math.abs(value - home[i]) < 1e-9));
  // Sorted smallest first.
  home.slice(1).forEach((value, i) => assert.ok(value >= home[i]));
});

test('Task 18 rejects near-singular poses and its whole plan stays clear of them', () => {
  const margin = task.rigid.constraints.singularity_margin;
  assert.ok(margin > 0);
  const cell = propagateSafety(task.rigid, safety);
  assert.equal(cell.singularity_margin, margin);
  // A nearly straight arm: floor/bend/self are fine, only the singularity trips.
  const nearStraight = [0.66, 0.05, 0.05, 0.05, 0.05, 0.05];
  const check = evaluateCellSafety([{ q: nearStraight, arm: ARM }], { ...cell, goal_workspace: undefined, obstacles: [] });
  assert.equal(check.reason, 'singular');
  assert.ok(check.singularity < margin);
  assert.equal(evaluateCellSafety([{ q: nearStraight, arm: ARM }], { singularity_margin: 0 }).safe, true);

  const scene = new RigidScene(task.rigid, { arms: 1 });
  const plan = planAutoPropagateTask(task.rigid, scene, { q: [...HOME_POSE], arm: ARM }, safety);
  const worst = Math.min(...plan.frames.map(([q]) => jacobianSingularValues(q, ARM)[0]));
  assert.ok(worst >= margin, `primary comes within ${worst} of a singularity`);
});
