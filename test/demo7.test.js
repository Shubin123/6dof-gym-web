import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import {
  ARM,
  ARM_B,
  HOME_POSE,
  distance,
  evaluateCellSafety,
  forwardKinematics,
  planTowelFoldMotion,
  computePolicyProgress,
} from '../src/core.js';
import { ClothSimulator } from '../src/cloth.js';

test('Laundry folding demo 7 (Towel fold) specification and bimanual setup', () => {
  const foldTask = compiled.workflows.find((w) => w.id === 'fold' || w.number === '07');
  assert.ok(foldTask, 'Demo 7 exists in workflows');
  assert.equal(foldTask.family, 'laundry');
  assert.equal(foldTask.name, 'Towel fold');
  assert.equal(foldTask.arms, 2, 'Requires bimanual two-arm setup');
  assert.deepEqual(foldTask.goal, [250, 180], 'Arm A pinning target is front-left');
  assert.deepEqual(foldTask.goal_b, [400, 180], 'Arm B grasping target is front-right');
});

test('Demo 7 bimanual folding motion plans safely and reaches folded target', () => {
  const safety = compiled.environment.safety;
  const poses = [
    { q: [...HOME_POSE], arm: ARM },
    { q: [...HOME_POSE], arm: ARM_B },
  ];
  const motion = planTowelFoldMotion(poses, safety);
  assert.ok(motion, 'Folding motion should plan successfully');
  assert.ok(motion.frames.length >= 100, 'Trajectory has enough control steps');

  let prev = poses.map((p) => p.q);
  for (let i = 0; i < motion.frames.length; i += 1) {
    const frame = motion.frames[i];
    const cellAssessment = evaluateCellSafety(
      [
        { q: frame[0], arm: ARM },
        { q: frame[1], arm: ARM_B },
      ],
      safety,
    );
    assert.equal(cellAssessment.safe, true, `Frame ${i} violated safety: ${cellAssessment.reason}`);

    for (let armIdx = 0; armIdx < 2; armIdx += 1) {
      for (let j = 0; j < 6; j += 1) {
        const delta = Math.abs(frame[armIdx][j] - prev[armIdx][j]);
        assert.ok(delta <= ARM.maxActionDelta + 1e-9, `Frame ${i} arm ${armIdx} joint ${j} exceeded delta: ${delta}`);
      }
    }
    prev = frame;
  }

  // Check end kinematics
  const finalA = forwardKinematics(prev[0], ARM).points.at(-1);
  const finalB = forwardKinematics(prev[1], ARM_B).points.at(-1);

  // Arm A pins first, then retracts so Arm B can place onto the fold line.
  assert.ok(distance(finalA, forwardKinematics(HOME_POSE, ARM).points.at(-1)) < 2.5, 'Arm A retracts clear after pinning');
  // Arm B should fold over the center line (x=325) toward Arm A
  assert.ok(finalB[0] < 325, `Arm B crossed midline to ${finalB[0]}`);
  assert.ok(finalB[2] < 40, `Arm B placed down on table: z=${finalB[2]}`);
});

test('Realistic cloth physics interacts with Demo 7 folding trajectory', () => {
  const safety = compiled.environment.safety;
  const motion = planTowelFoldMotion(
    [
      { q: [...HOME_POSE], arm: ARM },
      { q: [...HOME_POSE], arm: ARM_B },
    ],
    safety,
  );

  // Matches the production grid (src/main.js's cloth2d, src/viewport3d.js's
  // makeCloth) rather than an arbitrary resolution: a coarser grid converged
  // fine here while the shipped 14x11 mesh was still visibly unstable, which
  // is exactly how the premature-halt/no-settle-time regression this test
  // now guards against went unnoticed.
  const cloth = new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2 });
  assert.equal(cloth.captured[0], false);
  assert.equal(cloth.captured[1], false);

  // Step through the planned folding frames and feed arm tool positions
  const stepInterval = 1;
  for (let f = 0; f < motion.frames.length; f += stepInterval) {
    const frame = motion.frames[f];
    const tipA = forwardKinematics(frame[0], ARM).points.at(-1);
    const tipB = forwardKinematics(frame[1], ARM_B).points.at(-1);

    // Convert scene coordinates to cloth local coords (center at [325, 240], scale=100)
    const localTargetA = {
      x: (tipA[0] - 325) / 100,
      y: (tipA[1] - 240) / 100,
      z: tipA[2] / 100,
    };
    const localTargetB = {
      x: (tipB[0] - 325) / 100,
      y: (tipB[1] - 240) / 100,
      z: tipB[2] / 100,
    };

    cloth.step({ targetA: localTargetA, targetB: localTargetB, grips: motion.grips[f] });

    // A gripper that closes must close on its corner, not snap it across.
    motion.grips[f].forEach((closed, arm) => {
      if (!closed || (f > 0 && motion.grips[f - 1][arm])) return;
      const corner = arm ? 14 : 0;
      const tip = arm ? tipB : tipA;
      const cornerScene = [325 + cloth.positions[corner * 3] * 100, 240 + cloth.positions[corner * 3 + 1] * 100];
      assert.ok(Math.hypot(cornerScene[0] - tip[0], cornerScene[1] - tip[1]) < 3, `Arm ${arm ? 'B' : 'A'} closes on its corner`);
    });
    // The towel is folded where it lies, never dragged around the table.
    for (let i = 0; i < cloth.numVertices; i += 1) {
      const x = 325 + cloth.positions[i * 3] * 100;
      const y = 240 + cloth.positions[i * 3 + 1] * 100;
      assert.ok(x > 240 && x < 410 && y > 170 && y < 310, `frame ${f}: towel point ${i} dragged off its footprint to (${x.toFixed(0)}, ${y.toFixed(0)})`);
    }
  }

  // Verify corners were captured
  assert.equal(cloth.wasCaptured[0], true, 'Left corner captured by Arm A before its planned release');
  assert.equal(cloth.captured[0], false, 'Arm A releases the pinned edge before Arm B crosses the fold line');
  assert.equal(cloth.captured[1], true, 'Right corner captured by Arm B');

  // A's corner is pinned where A held it, and B's corner is laid on it.
  assert.ok(Math.hypot(cloth.positions[0] + 0.75, cloth.positions[1] + 0.6) < 0.02, 'pinned corner stays at the front-left corner');
  assert.ok(Math.hypot(cloth.tablePinA.x + 0.75, cloth.tablePinA.y + 0.6) < 0.02, 'table pin is where Arm A released the corner');

  // Verify folding metrics
  const metrics = cloth.getFoldMetrics();
  assert.equal(metrics.folded, true, 'Cloth is folded');
  assert.ok(metrics.frontDistance < 0.65, `Fold distance is tight: ${metrics.frontDistance.toFixed(3)}`);
  assert.ok(metrics.stretchError < 0.25, `Fabric stretch error bounded: ${metrics.stretchError.toFixed(3)}`);
});

test('Loading sliderbar supports scrubbing and loading simulation history', () => {
  const history = [];
  const cloth = new ClothSimulator({ columns: 8, rows: 6 });

  // Simulate policy steps recording into loading slider history
  for (let s = 0; s < 20; s += 1) {
    const targetA = { x: -0.75, y: -0.6, z: 0.02 + s * 0.01 };
    cloth.step({ targetA });
    history[s] = {
      step: s,
      arms: [
        { q: [0.1 * s, 0, 0, 0, 0, 0] },
        { q: [-0.1 * s, 0, 0, 0, 0, 0] },
      ],
      clothSnapshot: cloth.snapshot(),
    };
  }

  assert.equal(history.length, 20);

  // Test loading arbitrary step via sliderbar scrub
  const scrubStep = 7;
  const loadedFrame = history[scrubStep];
  assert.ok(loadedFrame, 'Frame exists in history');
  assert.ok(Math.abs(loadedFrame.arms[0].q[0] - 0.7) < 1e-6);
  assert.equal(loadedFrame.arms[0].q.length, 6);

  // Restore cloth state from slider snapshot
  cloth.restore(loadedFrame.clothSnapshot);
  assert.ok(cloth.positions[0] !== undefined);
  assert.equal(cloth.captured[0], true);
});

test('Policy progress bar tracks demo policy execution smoothly from 0% to 100%', () => {
  // Test computePolicyProgress math across boundary and step conditions
  assert.equal(computePolicyProgress(0, 100), 0);
  assert.equal(computePolicyProgress(50, 100), 50);
  assert.equal(computePolicyProgress(100, 100), 100);
  assert.equal(computePolicyProgress(150, 100), 100, 'Clamps at 100%');
  assert.equal(computePolicyProgress(-5, 100), 0, 'Clamps at 0%');
  assert.equal(computePolicyProgress(0, 0), 0, 'Handles 0 total steps gracefully');
  assert.equal(computePolicyProgress(27, 272), 10);
  assert.equal(computePolicyProgress(136, 272), 50);
  assert.equal(computePolicyProgress(272, 272), 100);

  // Test simulation policy run with Demo 7 folding path
  const safety = compiled.environment.safety;
  const motion = planTowelFoldMotion(
    [
      { q: [...HOME_POSE], arm: ARM },
      { q: [...HOME_POSE], arm: ARM_B },
    ],
    safety,
  );
  assert.ok(motion?.frames?.length > 0);
  const totalSteps = motion.frames.length;

  let lastProgress = -1;
  for (let step = 0; step <= totalSteps; step += 15) {
    const progress = computePolicyProgress(step, totalSteps);
    assert.ok(progress >= lastProgress, 'Progress increases monotonically');
    assert.ok(progress >= 0 && progress <= 100, 'Progress within 0-100%');
    lastProgress = progress;
  }
  assert.equal(computePolicyProgress(totalSteps, totalSteps), 100, 'Reaches 100% on completion');
});
