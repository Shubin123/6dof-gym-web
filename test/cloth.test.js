import test from 'node:test';
import assert from 'node:assert/strict';
import { ClothSimulator, DEFAULT_CLOTH_CONFIG } from '../src/cloth.js';

test('ClothSimulator initializes with correct grid topology and constraints', () => {
  const sim = new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2 });
  const expectedVertices = (14 + 1) * (11 + 1);
  assert.equal(sim.numVertices, expectedVertices);
  assert.equal(sim.positions.length, expectedVertices * 3);
  assert.equal(sim.restPositions.length, expectedVertices * 3);

  // Structural springs: horizontal (columns * (rows + 1)) + vertical (rows * (columns + 1))
  const expectedStructural = 14 * 12 + 11 * 15; // 168 + 165 = 333
  assert.equal(sim.structuralSprings.length / 3, expectedStructural);

  // Shear springs: 2 per quad cell (2 * columns * rows)
  const expectedShear = 2 * 14 * 11; // 308
  assert.equal(sim.shearSprings.length / 3, expectedShear);

  // Bending springs: 2-step horizontal ((columns - 1) * (rows + 1)) + 2-step vertical ((rows - 1) * (columns + 1))
  const expectedBending = 13 * 12 + 10 * 15; // 156 + 150 = 306
  assert.equal(sim.bendingSprings.length / 3, expectedBending);

  // Anchors initialized
  assert.ok(sim.anchorsA.length > 0, 'Arm A pinning anchors exist');
  assert.ok(sim.anchorsB.length > 0, 'Arm B grasping anchors exist');
});

test('Cloth rests stably on table under gravity without penetrating floor', () => {
  const sim = new ClothSimulator({ columns: 10, rows: 8, tableZ: 0.018 });
  for (let s = 0; s < 50; s += 1) {
    sim.step();
  }

  for (let i = 0; i < sim.numVertices; i += 1) {
    const z = sim.positions[i * 3 + 2];
    assert.ok(!isNaN(z), `Vertex ${i} has NaN z`);
    assert.ok(z >= sim.tableZ - 1e-5, `Vertex ${i} penetrated table: z = ${z}`);
  }
});

test('Table Coulomb friction prevents resting cloth from drifting', () => {
  const sim = new ClothSimulator({ columns: 8, rows: 6 });
  for (let s = 0; s < 20; s += 1) sim.step();

  const xBefore = Float32Array.from(sim.positions);
  // Run 30 more idle steps
  for (let s = 0; s < 30; s += 1) sim.step();

  for (let i = 0; i < sim.numVertices; i += 1) {
    const dx = Math.abs(sim.positions[i * 3] - xBefore[i * 3]);
    const dy = Math.abs(sim.positions[i * 3 + 1] - xBefore[i * 3 + 1]);
    assert.ok(dx < 0.001, `Vertex ${i} drifted horizontally: dx = ${dx}`);
    assert.ok(dy < 0.001, `Vertex ${i} drifted vertically: dy = ${dy}`);
  }
});

test('Gripper grasps corners within contact radius and tracks trajectory', () => {
  const sim = new ClothSimulator({ columns: 10, rows: 8 });
  assert.equal(sim.captured[0], false);
  assert.equal(sim.captured[1], false);

  // Far away target should not capture
  sim.step({ targetA: { x: 5, y: 5, z: 1 } });
  assert.equal(sim.captured[0], false);

  // Approaching front-left and front-right corners captures them
  const targetA = { x: -0.75, y: -0.6, z: 0.02 };
  const targetB = { x: 0.75, y: -0.6, z: 0.02 };
  sim.step({ targetA, targetB });
  assert.equal(sim.captured[0], true, 'Corner A captured');
  assert.equal(sim.captured[1], true, 'Corner B captured');

  // Gripper moves up: grasped corner tracks target height
  const liftTargetB = { x: 0.75, y: -0.6, z: 0.45 };
  for (let s = 0; s < 10; s += 1) {
    sim.step({ targetA, targetB: liftTargetB });
  }
  const cornerBIdx = sim.columns;
  assert.ok(Math.abs(sim.positions[cornerBIdx * 3 + 2] - 0.45) < 0.02, 'Corner B tracked lift target');
});

test('Folding motion folds cloth over midline with layer thickness separation', () => {
  const sim = new ClothSimulator({ columns: 12, rows: 10, thickness: 0.035 });
  const targetA = { x: -0.75, y: -0.6, z: 0.02 };
  const targetB = { x: 0.75, y: -0.6, z: 0.02 };
  sim.step({ targetA, targetB });

  // Simulate folding sequence: lift, travel across midline to x = -0.3, lower down
  const steps = 60;
  for (let s = 0; s <= steps; s += 1) {
    const t = s / steps;
    const x = 0.75 - t * 1.05; // from +0.75 across center to -0.30
    const z = 0.02 + Math.sin(t * Math.PI) * 0.35 + (1 - t) * 0.02;
    sim.step({ targetA, targetB: { x, y: -0.6, z } });
  }

  // Front corner fold distance should be significantly reduced from initial 1.5
  const frontDist = sim.getFrontFoldDistance();
  assert.ok(frontDist < 0.65, `Front corner folded over: distance = ${frontDist.toFixed(3)}`);

  const metrics = sim.getFoldMetrics();
  assert.equal(metrics.folded, true, 'Fold metrics report folded state');
  assert.ok(metrics.stretchError < 0.25, `Max stretch error bounded: ${metrics.stretchError.toFixed(3)}`);

  // Verify thickness barrier: upper folded layer does not sink below lower layer
  const halfCol = sim.columns / 2;
  for (let r = 0; r <= sim.rows; r += 1) {
    for (let c1 = 0; c1 <= halfCol; c1 += 1) {
      const i1 = r * (sim.columns + 1) + c1;
      const x1 = sim.positions[i1 * 3];
      const y1 = sim.positions[i1 * 3 + 1];
      const z1 = sim.positions[i1 * 3 + 2];

      for (let c2 = sim.columns; c2 > halfCol; c2 -= 1) {
        const i2 = r * (sim.columns + 1) + c2;
        const x2 = sim.positions[i2 * 3];
        const y2 = sim.positions[i2 * 3 + 1];
        const z2 = sim.positions[i2 * 3 + 2];

        if (sim.invMass[i2] > 0 && Math.hypot(x2 - x1, y2 - y1) < 0.1) {
          assert.ok(z2 >= z1 + sim.thickness - 1e-4, 'Layer thickness separation preserved');
        }
      }
    }
  }
});

test('Snapshot and restore support instant timeline frame loading', () => {
  const sim = new ClothSimulator({ columns: 8, rows: 6 });
  const targetA = { x: -0.75, y: -0.6, z: 0.02 };
  sim.step({ targetA });

  const snap1 = sim.snapshot();
  assert.ok(snap1.positions instanceof Float32Array);
  assert.equal(snap1.captured[0], true);

  // Deform cloth
  for (let s = 0; s < 25; s += 1) {
    sim.step({ targetA: { x: -0.4, y: -0.3, z: 0.3 } });
  }
  const deformedPos = sim.positions[0];
  assert.notEqual(deformedPos, snap1.positions[0]);

  // Restore snapshot
  sim.restore(snap1);
  assert.equal(sim.positions[0], snap1.positions[0]);
  assert.deepEqual(sim.captured, snap1.captured);
});

test('2D projection polygon outputs valid perimeter and crease points', () => {
  const sim = new ClothSimulator({ columns: 8, rows: 6 });
  const { basePoints, creasePoints } = sim.get2DPolygons([325, 240], 100);

  assert.ok(Array.isArray(basePoints));
  assert.ok(basePoints.length > 20, 'Base perimeter has enough boundary vertices');
  assert.ok(Array.isArray(creasePoints));
  assert.equal(creasePoints.length, sim.rows + 1, 'Crease spans all rows');

  basePoints.forEach(([x, y]) => {
    assert.ok(typeof x === 'number' && typeof y === 'number');
    assert.ok(!isNaN(x) && !isNaN(y));
  });
});
