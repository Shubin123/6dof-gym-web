import test from 'node:test';
import assert from 'node:assert/strict';
import { ClothSimulator, buildClothTopology } from '../src/cloth.js';

test('ClothSimulator initializes with correct grid topology and constraints', () => {
  const sim = new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2 });
  const expectedVertices = (14 + 1) * (11 + 1);
  assert.equal(sim.numVertices, expectedVertices);
  assert.equal(sim.positions.length, expectedVertices * 3);
  assert.equal(sim.restPositions.length, expectedVertices * 3);

  // Stretch springs are the non-diagonal triangle edges:
  // horizontal (columns * (rows + 1)) + vertical (rows * (columns + 1))
  const expectedStructural = 14 * 12 + 11 * 15; // 168 + 165 = 333
  assert.equal(sim.structuralSprings.length / 3, expectedStructural);

  // Shear springs: the one triangulation diagonal per quad cell
  const expectedShear = 14 * 11; // 154
  assert.equal(sim.shearSprings.length / 3, expectedShear);

  // Bending springs: one per interior edge, joining the far corners of the
  // two triangles that share it (all edges minus the perimeter)
  const expectedBending = expectedStructural + expectedShear - 2 * (14 + 11); // 437
  assert.equal(sim.bendingSprings.length / 3, expectedBending);

  // Graspable corners painted per arm
  assert.deepEqual(sim.anchorsA, [0], 'Arm A grasps the front-left corner');
  assert.deepEqual(sim.anchorsB, [14], 'Arm B grasps the front-right corner');
});

test('buildClothTopology merges seam duplicates and indexes every spring per point', () => {
  // Two triangles of a unit quad whose shared edge is duplicated, as an
  // exporter does across a UV seam: 6 vertices, 4 spatial points.
  const positions = [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0];
  const index = [0, 1, 2, 3, 4, 5];
  const topo = buildClothTopology(positions, index);

  assert.equal(topo.positions.length / 3, 4, 'coincident vertices collapse to one point');
  assert.deepEqual(Array.from(topo.vertexToPoint), [0, 1, 2, 1, 3, 2]);
  // 4 perimeter edges + 1 shared diagonal, plus the opposite-corner bend spring
  assert.equal(topo.springKinds.length, 6);
  assert.equal(topo.springKinds.filter((kind) => kind === 1).length, 1, 'the shared hypotenuse is the shear spring');

  // CSR: [count, ids...] per point; every spring appears under both its ends
  const seen = new Array(topo.springKinds.length).fill(0);
  for (let p = 0; p < 4; p += 1) {
    const start = topo.springPointer[p];
    const count = topo.springsPerPoint[start];
    for (let k = start + 1; k <= start + count; k += 1) {
      const s = topo.springsPerPoint[k];
      assert.ok(topo.springs[s * 2] === p || topo.springs[s * 2 + 1] === p, `spring ${s} listed under point ${p} touches it`);
      seen[s] += 1;
    }
  }
  assert.ok(seen.every((n) => n === 2), 'each spring is reachable from exactly its two points');
});

test('A flat towel at rest stays flat: weave neighbours across the midline are not fold layers', () => {
  // Midline neighbours are closer than the layer-overlap radius; treating
  // them as stacked layers lifted the flat towel before anything touched it.
  const sim = new ClothSimulator({ columns: 14, rows: 11 });
  for (let s = 0; s < 40; s += 1) sim.step();
  for (let i = 0; i < sim.numVertices; i += 1) {
    assert.ok(Math.abs(sim.positions[i * 3 + 2] - sim.tableZ) < 1e-4, `vertex ${i} left the table: z = ${sim.positions[i * 3 + 2]}`);
  }
});

test('A grasp binds the nearest painted corner as a magnet and a release frees it', () => {
  const sim = new ClothSimulator({ columns: 10, rows: 8 });
  // Hovering over the middle of the towel grabs nothing: only painted
  // corners are graspable.
  sim.step({ targetA: { x: 0, y: 0, z: 0.05 } });
  assert.equal(sim.captured[0], false);
  assert.ok(sim.pointMagnet.every((m) => m === 0));

  const corner = { x: -0.75, y: -0.6, z: 0.02 };
  sim.step({ targetA: corner });
  assert.equal(sim.pointMagnet[0], 1, 'corner 0 is bound to magnet A');
  assert.equal(sim.invMass[0], 0, 'a magnet-held point is kinematic');

  // Retracting past releaseRadius releases the arm; the corner is kept on
  // the table by the pin instead.
  sim.step({ targetA: { x: -0.75, y: -0.6, z: 0.8 } });
  assert.equal(sim.captured[0], false);
  assert.equal(sim.released[0], true);
  assert.ok(sim.tablePinA, 'released corner is pinned to the table');
  for (let s = 0; s < 10; s += 1) sim.step({ targetA: { x: -0.75, y: -0.6, z: 0.8 } });
  assert.ok(Math.abs(sim.positions[2] - sim.tableZ) < 1e-4, 'pinned corner stays on the table');
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

  // Deform cloth. The target stays within releaseRadius of the grasp point
  // so the arm keeps hold and actually drags the towel.
  for (let s = 0; s < 25; s += 1) {
    sim.step({ targetA: { x: -0.6, y: -0.45, z: 0.3 } });
  }
  // The captured corner is intentionally kinematic; inspect its neighbouring
  // free vertex to prove that the saved cloth state actually differs.
  const freeVertexX = 3;
  const deformedPos = sim.positions[freeVertexX];
  assert.notEqual(deformedPos, snap1.positions[freeVertexX]);

  // Restore snapshot
  sim.restore(snap1);
  assert.equal(sim.positions[freeVertexX], snap1.positions[freeVertexX]);
  assert.deepEqual(sim.captured, snap1.captured);
});

/** Max relative stretch error for an arbitrary snapshot, not just sim.positions. */
function maxStretchErrorOf(sim, positions) {
  let maxError = 0;
  for (let s = 0; s < sim.structuralSprings.length; s += 3) {
    const i1 = sim.structuralSprings[s];
    const i2 = sim.structuralSprings[s + 1];
    const restLen = sim.structuralSprings[s + 2];
    const dx = positions[i2 * 3] - positions[i1 * 3];
    const dy = positions[i2 * 3 + 1] - positions[i1 * 3 + 1];
    const dz = positions[i2 * 3 + 2] - positions[i1 * 3 + 2];
    const len = Math.hypot(dx, dy, dz);
    maxError = Math.max(maxError, Math.abs(len - restLen) / restLen);
  }
  return maxError;
}

test('Frame buffer: every recorded step of a full fold trajectory stays physically valid', () => {
  // history is recorded for the settling detector, but it is also a ready-made
  // fixture: replaying a real trajectory once and then checking every frame it
  // produced catches mid-trajectory instability (e.g. the fabric overstretching
  // partway through a fold before settling back down) that an end-state-only
  // check would miss entirely.
  const sim = new ClothSimulator({ columns: 12, rows: 10, thickness: 0.035 });
  const targetA = { x: -0.75, y: -0.6, z: 0.02 };
  const targetB = { x: 0.75, y: -0.6, z: 0.02 };
  sim.step({ targetA, targetB });

  const steps = 60;
  for (let s = 0; s <= steps; s += 1) {
    const t = s / steps;
    const x = 0.75 - t * 1.05;
    const z = 0.02 + Math.sin(t * Math.PI) * 0.35 + (1 - t) * 0.02;
    sim.step({ targetA, targetB: { x, y: -0.6, z } });
  }

  assert.ok(sim.history.length > 50, 'the trajectory recorded enough frames to be a meaningful fixture');
  assert.ok(Array.from(sim.history.at(-1)).every((v, i) => v === sim.positions[i]), 'the newest buffered frame matches the live state');

  sim.history.forEach((frame, index) => {
    for (let i = 0; i < sim.numVertices; i += 1) {
      const z = frame[i * 3 + 2];
      assert.ok(!Number.isNaN(z), `frame ${index} vertex ${i} went NaN`);
      assert.ok(z >= sim.tableZ - 1e-4, `frame ${index} vertex ${i} penetrated the table: z = ${z}`);
    }
    const stretch = maxStretchErrorOf(sim, frame);
    assert.ok(stretch < 0.6, `frame ${index} stretched past a physically plausible bound: ${stretch.toFixed(3)}`);
  });
});

test('Frame buffer stays bounded to historyLimit and keeps only the most recent frames', () => {
  const sim = new ClothSimulator({ columns: 6, rows: 5, historyLimit: 20 });
  for (let s = 0; s < 45; s += 1) sim.step({ targetA: { x: -0.75, y: -0.6, z: 0.02 } });
  assert.equal(sim.history.length, 20, 'the buffer stops growing at historyLimit');
  assert.ok(Array.from(sim.history.at(-1)).every((v, i) => v === sim.positions[i]), 'the last buffered frame is the current state');
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
