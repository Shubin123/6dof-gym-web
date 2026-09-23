import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import registry from '../data/sources.json' with { type: 'json' };
import { ARM, ARM_B, buildEpisodeArtifact, distance, evaluateCellSafety, forwardKinematics, GOAL_Z, guidedStep, HALT, haltState, HOME_POSE, MAX_EPISODE_TRANSITIONS, nearestArm, planSafeCellMotion, planTowelFoldMotion, projectToReachableWorkspace, solveInverseKinematics } from '../src/core.js';

const HOME = HOME_POSE;
const armsOf = (workflow) => (workflow.arms === 2 ? [[ARM, workflow.goal], [ARM_B, workflow.goal_b]] : [[ARM, workflow.goal]]);
const targetOf = (workflow, arm, goal) => projectToReachableWorkspace([...goal, workflow.goal_height], compiled.environment.safety, arm);

test('compiled environment has the browser task contract', () => {
  assert.equal(compiled.environment.id, 'Arm6-Reach-v0');
  assert.equal(compiled.environment.observation_dim, 22);
  assert.equal(compiled.environment.action_dim, 7);
  assert.equal(compiled.environment.control_hz, 25);
});
test('every workflow goal is inside the table and the reach shell', () => {
  const [minX, maxX, minY, maxY] = compiled.environment.safety.goal_workspace;
  for (const workflow of compiled.workflows) {
    for (const [arm, goal] of armsOf(workflow)) {
      assert.ok(goal[0] >= minX && goal[0] <= maxX && goal[1] >= minY && goal[1] <= maxY, `${workflow.id}/${arm.id} is off the table`);
      assert.ok(workflow.goal_height >= GOAL_Z.min && workflow.goal_height <= GOAL_Z.max, `${workflow.id} height is outside the band`);
      const target = [...goal, workflow.goal_height];
      assert.deepEqual(
        targetOf(workflow, arm, goal).map((value) => Math.round(value)),
        target.map((value) => Math.round(value)),
        `${workflow.id}/${arm.id} is displaced by the reach projection`,
      );
    }
  }
});

test('model and dataset cards retain a primary URL', () => {
  for (const item of [...compiled.models, ...compiled.datasets]) assert.match(item.url, /^https:\/\//);
});

test('guidance reaches every task goal in three dimensions, on either arm', () => {
  for (const workflow of compiled.workflows) {
    for (const [arm, goal] of armsOf(workflow)) {
      const target = targetOf(workflow, arm, goal);
      const plan = solveInverseKinematics(target, arm);
      assert.ok(plan.distance < 2, `${workflow.id}/${arm.id} should have a valid IK plan`);
      let q = [...HOME];
      const before = distance(forwardKinematics(q, arm).points.at(-1), target);
      for (let step = 0; step < workflow.horizon_steps; step += 1) {
        const update = guidedStep(q, target, arm, plan);
        assert.ok(update.action.every((delta) => Math.abs(delta) <= arm.maxActionDelta + 1e-9));
        q = update.q;
      }
      const after = distance(forwardKinematics(q, arm).points.at(-1), target);
      assert.ok(after < 8, `${workflow.id}/${arm.id} finished ${after.toFixed(1)}px from target`);
      assert.ok(after < before, 'guidance should reduce target distance');
    }
  }
});

test('the two arms are mirror images that share one joint vector', () => {
  const pose = [0.4, 0.9, -1.2, 0.3, -0.7, 0.2];
  const left = forwardKinematics(pose, ARM).points;
  const right = forwardKinematics(pose, ARM_B).points;
  left.forEach(([x, y, z], index) => {
    const [mx, my, mz] = right[index];
    assert.ok(Math.abs((x - ARM.base[0]) + (mx - ARM_B.base[0])) < 1e-9, 'mirrored across the base column');
    assert.ok(Math.abs(y - my) < 1e-9 && Math.abs(z - mz) < 1e-9, 'depth and height are shared');
  });
  assert.equal(nearestArm([540, 200], [ARM, ARM_B]).id, 'B');
  assert.equal(nearestArm([220, 200], [ARM, ARM_B]).id, 'A');
});

test('floor, workspace, and inter-arm collisions are rejected', () => {
  const safety = compiled.environment.safety;
  assert.equal(evaluateCellSafety([{ q: HOME, arm: ARM }, { q: HOME, arm: ARM_B }], safety).safe, true, 'home is a safe bimanual pose');

  const belowFloor = evaluateCellSafety([{ q: [0, 0.6, -1.1, 0, -0.6, 0], arm: ARM }], safety);
  assert.equal(belowFloor.safe, false);
  assert.equal(belowFloor.reason, 'floor');

  const beyondEdge = evaluateCellSafety([{ q: [0, 0, 0, 0, 0, 0], arm: ARM }], safety);
  assert.equal(beyondEdge.safe, false);
  assert.equal(beyondEdge.reason, 'workspace');

  const formerCrossedHome = [1.284, 1.61, -1.688, -0.865, -1.522, -1.004];
  const crossed = evaluateCellSafety([{ q: formerCrossedHome, arm: ARM }, { q: formerCrossedHome, arm: ARM_B }], safety);
  assert.equal(crossed.safe, false);
  assert.equal(crossed.reason, 'collision');
  assert.ok(crossed.armClearance < safety.arm_clearance_px);
});

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
    let previous = poses.map(({ q }) => q);
    for (const frame of motion.frames) {
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

test('a pose occupies real height, not a single plane', () => {
  const { points } = forwardKinematics(HOME, ARM);
  const heights = points.map((point) => point[2]);
  assert.ok(Math.max(...heights) - Math.min(...heights) > 60, 'the home pose should span a real height range');
  assert.ok(points.some((point, index) => index > 1 && Math.abs(point[1] - points[1][1]) > 20), 'the chain should leave its initial plane');
});

test('guidance recovers from representative manual poses across the workspace', () => {
  const starts = [HOME, [0, 0, 0, 0, 0, 0], [1.2, 1.1, -0.8, 0.6, -0.4, 0.2], [-1.2, 0.5, -1.5, -0.6, -0.9, -0.2]];
  for (const arm of [ARM, ARM_B]) {
    for (let x = 100; x <= 620; x += 130) for (let y = 110; y <= 390; y += 90) for (const z of [10, 80, 150]) {
      const target = projectToReachableWorkspace([x, y, z], compiled.environment.safety, arm);
      const plan = solveInverseKinematics(target, arm);
      // Guidance converges on the plan; whether the plan reaches the target is
      // the solver's business, and the task-goal test above covers that.
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

test('an episode always ends in a named halt state', () => {
  assert.equal(haltState({ error: 3, steps: 10, budget: 200 }), HALT.REACHED);
  assert.equal(haltState({ error: 40, steps: 200, budget: 200 }), HALT.BUDGET);
  assert.equal(haltState({ error: 40, steps: 10, budget: 200 }), HALT.RUNNING);
});

test('a dragged target is projected into the reach shell from either side', () => {
  const safety = compiled.environment.safety;
  const shoulder = [...ARM.base, ARM.baseHeight];
  const far = projectToReachableWorkspace([760, 0, 160], safety);
  assert.ok(distance(far, shoulder) <= safety.max_reach_px + 1e-9, 'clamped to the outer wall');
  const onTopOfTheBase = projectToReachableWorkspace([...ARM.base, 150], safety);
  assert.ok(distance(onTopOfTheBase, shoulder) >= safety.min_reach_px - 1e-6, 'pushed out of the inner wall');
  const [minX, maxX, minY, maxY] = safety.goal_workspace;
  for (const projected of [far, onTopOfTheBase]) {
    assert.ok(projected[0] >= minX - 1e-6 && projected[0] <= maxX + 1e-6);
    assert.ok(projected[1] >= minY - 1e-6 && projected[1] <= maxY + 1e-6);
    assert.ok(projected[2] >= GOAL_Z.min && projected[2] <= GOAL_Z.max);
  }
});

test('episode exports retain optional replayable voice fields', () => {
  const artifact = buildEpisodeArtifact({ environment: 'Arm6-Reach-v0', task: compiled.workflows[0], transitions: [], voice: { transcript: 'place the cube', mimeType: 'audio/webm', dataUrl: 'data:audio/webm;base64,AA==' } });
  assert.equal(artifact.schema, 'armlab-episode-preview/v0.4');
  assert.equal(artifact.voice.transcript, 'place the cube');
  assert.match(artifact.voice.audio_data_url, /^data:audio\/webm;base64,/);
});

test('browser episode buffer has a finite task-aligned bound', () => {
  assert.equal(MAX_EPISODE_TRANSITIONS, compiled.environment.max_steps);
});

test('every task example carries a complete, teachable specification', () => {
  const families = new Set(compiled.families.map((family) => family.id));
  const numbers = new Set();
  for (const workflow of compiled.workflows) {
    assert.ok(families.has(workflow.family), `${workflow.id} references an unknown family`);
    assert.ok(!numbers.has(workflow.number), `${workflow.number} is used twice`);
    numbers.add(workflow.number);
    for (const field of ['name', 'instruction', 'metric', 'summary', 'success', 'sensors', 'guardrail']) {
      assert.equal(typeof workflow[field], 'string', `${workflow.id} is missing ${field}`);
      assert.ok(workflow[field].length > 8, `${workflow.id}.${field} is too thin to teach from`);
    }
    assert.ok(workflow.baseline?.length, `${workflow.id} should name a suggested baseline policy`);
    assert.ok(workflow.curriculum.length >= 3, `${workflow.id} needs a curriculum with progression`);
    assert.ok(workflow.failure_modes.length >= 2, `${workflow.id} needs its known failure modes`);
    assert.ok(workflow.difficulty >= 1 && workflow.difficulty <= 5);
    assert.ok(workflow.horizon_steps > 0 && workflow.horizon_steps <= compiled.environment.max_steps);
  }
});

test('bimanual examples declare a second reachable goal', () => {
  for (const workflow of compiled.workflows) {
    assert.ok(workflow.arms === 1 || workflow.arms === 2, `${workflow.id} must declare its arm count`);
    if (workflow.arms === 2) assert.ok(Array.isArray(workflow.goal_b), `${workflow.id} is bimanual but has no second goal`);
    else assert.equal(workflow.goal_b, undefined, `${workflow.id} is single-armed but carries a second goal`);
  }
  assert.ok(compiled.workflows.some((workflow) => workflow.arms === 2), 'some tasks need two arms');
  assert.equal(compiled.environment.bimanual.observation_dim, compiled.environment.observation_dim * 2);
  assert.equal(compiled.environment.bimanual.action_dim, compiled.environment.action_dim * 2);
});

test('every task family is represented by at least one example', () => {
  for (const family of compiled.families) {
    assert.ok(compiled.workflows.some((workflow) => workflow.family === family.id), `${family.id} has no example`);
  }
});

test('the study path and source registry keep resolvable references', () => {
  for (const entry of registry.study_path) {
    assert.match(entry.url, /^https:\/\//);
    assert.ok(entry.stage && entry.title && entry.blurb);
  }
  assert.ok(registry.sources.every((source) => /^https:\/\//.test(source.url)));
  assert.ok(registry.sources.every((source) => typeof source.used_for === 'string' && source.used_for.length > 0));
});

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
