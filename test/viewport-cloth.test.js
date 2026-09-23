import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, ARM_B, HOME_POSE, forwardKinematics, planTowelFoldMotion } from '../src/core.js';
import { ClothSimulator } from '../src/cloth.js';
import { makeCloth, syncClothGeometry, toWorld } from '../src/viewport3d.js';

// The 3-D towel is a view of the same simulator state the 2-D scene draws.
// A rotated group once mirrored it front-to-back, so the corners the grippers
// held were drawn on the far edge of the towel, away from the tools.
test('3-D towel corners sit in the grippers throughout the Task 7 fold', () => {
  const motion = planTowelFoldMotion([{ q: [...HOME_POSE], arm: ARM }, { q: [...HOME_POSE], arm: ARM_B }], compiled.environment.safety);
  const sim = new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2 });
  const view = makeCloth();
  const worldOf = (index) => {
    view.group.updateMatrixWorld(true);
    return new THREE.Vector3().fromBufferAttribute(view.geometry.attributes.position, index).applyMatrix4(view.group.matrixWorld);
  };
  let checked = 0;

  motion.frames.forEach((frame, f) => {
    const tips = [forwardKinematics(frame[0], ARM).points.at(-1), forwardKinematics(frame[1], ARM_B).points.at(-1)];
    const local = tips.map(([x, y, z]) => ({ x: (x - 325) / 100, y: (y - 240) / 100, z: z / 100 }));
    sim.step({ targetA: local[0], targetB: local[1], grips: motion.grips[f] });
    view.simulator.restore(sim.snapshot());
    syncClothGeometry(view);
    sim.captured.forEach((held, arm) => {
      if (!held) return;
      const corner = arm ? sim.anchorsB[0] : sim.anchorsA[0];
      const gap = worldOf(corner).distanceTo(toWorld(tips[arm]));
      assert.ok(gap < 0.03, `frame ${f}: Arm ${arm ? 'B' : 'A'}'s corner drawn ${(gap * 100).toFixed(1)} px from its tool`);
      checked += 1;
    });
  });
  assert.ok(checked > 200, 'the held corners were checked through the fold');

  // The resting towel lies flat on the table under its scene footprint.
  sim.reset();
  view.simulator.restore(sim.snapshot());
  syncClothGeometry(view);
  const nearLeft = worldOf(0);
  assert.ok(nearLeft.distanceTo(toWorld([250, 180, sim.tableZ * 100])) < 1e-3, 'front-left corner under scene (250, 180)');
  assert.ok(worldOf(view.simulator.numVertices - 1).distanceTo(toWorld([400, 300, sim.tableZ * 100])) < 1e-3, 'back-right corner under scene (400, 300)');
});
