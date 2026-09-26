import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, ARM_B, distance, evaluateCellSafety, forwardKinematics, HOME_POSE, projectToReachableWorkspace, solveInverseKinematics } from '../src/core.js';
import { ClothSimulator } from '../src/cloth.js';
import { createShirtMesh, getShirtFoldMetrics, getShirtMeshInfo, planShirtMultiFoldMotion, scoreShirtFold } from '../src/shirt-fold.js';
import { policyRecipeFor, scoreTaskStages } from '../src/task-policies.js';

const HOME = HOME_POSE;
const safety = compiled.environment.safety;
const task = compiled.workflows.find((workflow) => workflow.id === 'fold_shirt');

test('Task 17 (Shirt multi-fold) specification and bimanual setup', () => {
  assert.ok(task, 'Task 17 must exist in compiled.json');
  assert.equal(task.id, 'fold_shirt');
  assert.equal(task.number, '17');
  assert.equal(task.family, 'laundry');
  assert.equal(task.arms, 2, 'Shirt folding is a bimanual task requiring two arms');
  assert.equal(task.object, 'shirt');
  assert.ok(task.curriculum.length >= 3, 'declares a curriculum');
  assert.ok(task.failure_modes.length >= 2, 'declares failure modes');
  assert.ok(task.guardrail.length > 10, 'declares safety guardrails');
  assert.ok(task.summary.length > 20, 'declares task summary');
  assert.ok(task.success.length > 20, 'declares measurable success criteria');

  // Both goals must be reachable in free space
  const targetA = projectToReachableWorkspace([...task.goal, task.goal_height], safety, ARM);
  const targetB = projectToReachableWorkspace([...task.goal_b, task.goal_height], safety, ARM_B);
  const planA = solveInverseKinematics(targetA, ARM, safety);
  const planB = solveInverseKinematics(targetB, ARM_B, safety, [{ q: planA.q, arm: ARM }]);

  assert.ok(planA?.safety?.safe, 'Arm A goal must have a safe IK solution');
  assert.ok(planB?.safety?.safe, 'Arm B goal must have a safe IK solution');
  assert.ok(planA.distance < 2, 'Arm A plan reaches goal');
  assert.ok(planB.distance < 2, 'Arm B plan reaches goal');

  const cellCheck = evaluateCellSafety([{ q: planA.q, arm: ARM }, { q: planB.q, arm: ARM_B }], safety);
  assert.equal(cellCheck.safe, true, 'Both arms at their goal poses maintain inter-arm clearance');
});

test('Shirt mesh topology, regions, and landmarks', () => {
  const info = createShirtMesh({ columns: 16, rows: 12, width: 1.8, height: 1.2, tableZ: 0.018 });
  assert.ok(info.numVertices > 80, 'Mesh has sufficient resolution for garment dynamics');
  assert.ok(info.mesh.index.length > 100, 'Mesh is triangulated');
  assert.ok(info.quads.length > 50, 'Quad cells indexed');

  const { landmarks, regions } = info;
  assert.ok(Number.isInteger(landmarks.leftSleeveCuff), 'leftSleeveCuff landmark defined');
  assert.ok(Number.isInteger(landmarks.rightSleeveCuff), 'rightSleeveCuff landmark defined');
  assert.ok(Number.isInteger(landmarks.leftSleeveTarget), 'leftSleeveTarget landmark defined');
  assert.ok(Number.isInteger(landmarks.rightSleeveTarget), 'rightSleeveTarget landmark defined');
  assert.ok(Number.isInteger(landmarks.hemLeft), 'hemLeft landmark defined');
  assert.ok(Number.isInteger(landmarks.hemRight), 'hemRight landmark defined');

  // Cuff positions in rest mesh
  const p = info.mesh.positions;
  const leftCuffX = p[landmarks.leftSleeveCuff * 3];
  const rightCuffX = p[landmarks.rightSleeveCuff * 3];
  assert.ok(leftCuffX < -0.8, `left cuff extends to left sleeve tip: ${leftCuffX}`);
  assert.ok(rightCuffX > 0.8, `right cuff extends to right sleeve tip: ${rightCuffX}`);

  // Regional labeling
  assert.equal(regions[landmarks.leftSleeveCuff], 'sleeve_left');
  assert.equal(regions[landmarks.rightSleeveCuff], 'sleeve_right');
  assert.equal(regions[landmarks.hemLeft], 'torso_lower');
  assert.equal(regions[landmarks.hemRight], 'torso_lower');
  assert.equal(regions[landmarks.leftSleeveTarget], 'torso_upper');
  assert.equal(regions[landmarks.rightSleeveTarget], 'torso_upper');

  // Verify spring generation via ClothSimulator
  const sim = new ClothSimulator({
    columns: info.columns,
    rows: info.rows,
    width: info.width,
    height: info.height,
    tableZ: info.tableZ,
    mesh: info.mesh,
    foldType: 'shirt',
    regions: info.regions,
    shirtMeshInfo: info,
  });
  assert.equal(sim.numVertices, info.numVertices);
  assert.ok(sim.numSprings > 200, 'Constructed spring network');
  assert.ok(sim.structuralSprings.length > 0, 'Generated structural springs');
  assert.ok(sim.shearSprings.length > 0, 'Generated shear springs');
  assert.ok(sim.bendingSprings.length > 0, 'Generated bending springs');
});

test('Shirt rests stably under gravity without penetrating table', () => {
  const info = getShirtMeshInfo();
  const sim = new ClothSimulator({
    columns: info.columns,
    rows: info.rows,
    width: info.width,
    height: info.height,
    tableZ: info.tableZ,
    mesh: info.mesh,
    foldType: 'shirt',
    regions: info.regions,
    shirtMeshInfo: info,
  });

  for (let s = 0; s < 40; s += 1) sim.step();
  for (let i = 0; i < sim.numVertices; i += 1) {
    const z = sim.positions[i * 3 + 2];
    assert.ok(z >= sim.tableZ - 1e-4, `Vertex ${i} penetrated table: z = ${z}`);
    assert.ok(Math.abs(z - sim.tableZ) < 0.01, `Resting garment stayed flat on table: z = ${z}`);
  }
});

test('Task 17 bimanual multi-fold motion plan is collision-free and rate-capped', () => {
  const poses = [{ q: [...HOME], arm: ARM }, { q: [...HOME], arm: ARM_B }];
  const recipe = policyRecipeFor(task);
  const plan = planShirtMultiFoldMotion(poses, safety, recipe.profile);

  assert.ok(plan, 'planShirtMultiFoldMotion must return a valid plan');
  assert.ok(plan.frames.length > 100, `plan has substantial multi-step frames: ${plan.frames.length}`);
  assert.equal(plan.frames.length, plan.grips.length, '1:1 correspondence between frames and gripper commands');
  assert.ok(plan.stageEnds.length >= 4, 'plan marks stage milestones');

  let prevA = poses[0].q;
  let prevB = poses[1].q;
  for (let i = 0; i < plan.frames.length; i += 1) {
    const [qA, qB] = plan.frames[i];
    const check = evaluateCellSafety([{ q: qA, arm: ARM }, { q: qB, arm: ARM_B }], safety);
    assert.equal(check.safe, true, `Frame ${i} violated cell safety: ${check.reason}`);

    for (let j = 0; j < 6; j += 1) {
      assert.ok(Math.abs(qA[j] - prevA[j]) <= ARM.maxActionDelta + 1e-6, `Frame ${i} arm A joint ${j} exceeded delta cap`);
      assert.ok(Math.abs(qB[j] - prevB[j]) <= ARM_B.maxActionDelta + 1e-6, `Frame ${i} arm B joint ${j} exceeded delta cap`);
    }
    prevA = qA;
    prevB = qB;
  }
});

test('Realistic cloth physics executes the multi-fold and scores stages', () => {
  const info = getShirtMeshInfo();
  const sim = new ClothSimulator({
    columns: info.columns,
    rows: info.rows,
    width: info.width,
    height: info.height,
    tableZ: info.tableZ,
    mesh: info.mesh,
    foldType: 'shirt',
    regions: info.regions,
    shirtMeshInfo: info,
  });

  sim.setAnchors(info.landmarks.leftSleeveCuff, info.landmarks.rightSleeveCuff, {
    additionalAnchors: [info.landmarks.hemLeft, info.landmarks.hemRight],
    foldType: 'shirt',
    pinOnRelease: false,
  });

  // Flat shirt initially: not folded
  const initMetrics = getShirtFoldMetrics(sim);
  assert.equal(initMetrics.folded, false, 'Unfolded shirt is not marked folded');

  const initScore = scoreTaskStages(task, { cloth: sim });
  assert.ok(initScore.reward < 0.2, `Initial score is near zero: ${initScore.reward}`);
  assert.equal(initScore.complete, false);

  // Run the multi-fold trajectory through the simulator
  const poses = [{ q: [...HOME], arm: ARM }, { q: [...HOME], arm: ARM_B }];
  const recipe = policyRecipeFor(task);
  const plan = planShirtMultiFoldMotion(poses, safety, recipe.profile);
  assert.ok(plan);

  const clothOrigin = [380, 240];
  for (let f = 0; f < plan.frames.length; f += 1) {
    const [qA, qB] = plan.frames[f];
    const tipA = forwardKinematics(qA, ARM).points.at(-1);
    const tipB = forwardKinematics(qB, ARM_B).points.at(-1);
    const grips = plan.grips[f];

    const targetA = { x: (tipA[0] - clothOrigin[0]) / 100, y: (tipA[1] - clothOrigin[1]) / 100, z: tipA[2] / 100 };
    const targetB = { x: (tipB[0] - clothOrigin[0]) / 100, y: (tipB[1] - clothOrigin[1]) / 100, z: tipB[2] / 100 };

    sim.step({ targetA, targetB, grips });
  }

  // After multi-fold execution and settling:
  const finalMetrics = getShirtFoldMetrics(sim);
  assert.ok(finalMetrics.leftSleeveGap < 0.22, `Left sleeve tucked: gap = ${finalMetrics.leftSleeveGap}`);
  assert.ok(finalMetrics.rightSleeveGap < 0.22, `Right sleeve tucked: gap = ${finalMetrics.rightSleeveGap}`);
  assert.ok(finalMetrics.maxHemGap < 0.24, `Bottom hem lifted over waist: gap = ${finalMetrics.maxHemGap}`);
  assert.equal(finalMetrics.folded, true, 'Garment is fully multi-folded');
  assert.ok(finalMetrics.stretchError < 0.25, `Garment fabric preserved without overstretch: ${finalMetrics.stretchError}`);

  // Policy stage evaluation
  const finalScore = scoreTaskStages(task, { cloth: sim });
  assert.ok(finalScore.reward >= 0.9, `Stage reward reached near completion: ${finalScore.reward}`);
});
