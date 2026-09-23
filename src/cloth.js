/**
 * Cloth simulation for robotic manipulation, structured after
 * bandinopla/three-simplecloth (MIT license,
 * https://github.com/bandinopla/three-simplecloth) and its write-up
 * "Simple Cloth Simulation with Three.js and Compute Shaders on skeletal
 * animated meshes".
 *
 * The cloth is data, not objects: flat typed-array "stores" indexed by point
 * or spring id, advanced by kernel passes that each read one consistent store
 * and write only their own index. Every pass is therefore a 1:1 stand-in for
 * a compute dispatch (`Fn(...)().compute(count)`), while running on the CPU
 * keeps the solver deterministic and synchronous for scoring, timeline
 * snapshots and `node --test`.
 *
 *  - Topology is read from a triangle mesh: coincident vertices are merged
 *    by a position hash, every unique triangle edge becomes a spring, and a
 *    CSR list ([count, id, id, ..., count, id, ...]) gives each point its
 *    springs.
 *  - Bending springs join the opposite vertices of every pair of triangles
 *    sharing an edge (the reference has no bending term; a fold needs one).
 *  - Per sub-step: computeSpringForces (per spring) -> computeVertexForces
 *    (per point: damped force accumulator, spring sum, gravity, table
 *    collider with Coulomb friction, maxForce clamp, magnet override) ->
 *    layer separation for the folded half.
 *  - Grippers are magnets: a grasp binds the nearest graspable vertex (the
 *    reference's vertex-paint mask) and drives it to the gripper target.
 */

export const DEFAULT_CLOTH_CONFIG = Object.freeze({
  columns: 14,
  rows: 11,
  width: 1.5,
  height: 1.2,
  thickness: 0.035,
  tableZ: 0.018,
  // Per-step quantities. They are converted to per-sub-step values so that
  // changing `substeps` changes accuracy, not the physical behaviour.
  // Roughly half of real gravity for 10 cm units at 50 steps/s. Paired with
  // air damping this plays the role of the reference's heavy `dampening`: a
  // yanked corner cannot fling the towel upward for dozens of steps.
  gravity: -0.02,
  damping: 0.1,
  // Coulomb coefficients against the table: a contact sticks while its
  // lateral motion is within staticFriction x the normal push, otherwise
  // it slides and loses kineticFriction x the normal push.
  staticFriction: 1.2,
  kineticFriction: 0.9,
  // Fixed sub-steps per step() call, the analogue of the reference's
  // steps-per-second accumulator.
  substeps: 24,
  // The reference's single `stiffness` uniform, split per spring kind. Each
  // spring moves its ends by stiffness/2 of its error per sub-step, so a
  // point's summed stiffness must stay under the explicit-integration limit
  // (~4); a towel is limp, hence the small bend term.
  stretchStiffness: 0.4,
  shearStiffness: 0.1,
  bendStiffness: 0.01,
  // Per-sub-step displacement cap ("so it doesn't act too crazy").
  maxForce: 0.02,
  // Dashpot on each spring: the share of the two ends' relative velocity
  // along the spring removed per sub-step. Unlike the reference's global
  // `dampening` it does not slow free fall, only internal oscillation.
  springDamping: 0.25,
  graspRadius: 0.36,
  releaseRadius: 0.52,
  historyLimit: 120,
  settleDisplacementLimit: 0.003,
});

/** Parameters that may change on a live simulator; the rest fix the mesh. */
export const CLOTH_PHYSICS_KEYS = Object.freeze([
  'thickness', 'gravity', 'damping', 'staticFriction', 'kineticFriction', 'substeps',
  'stretchStiffness', 'shearStiffness', 'bendStiffness', 'maxForce', 'springDamping',
  'graspRadius', 'releaseRadius',
]);

const SPRING_STRETCH = 0;
const SPRING_SHEAR = 1;
const SPRING_BEND = 2;

/**
 * Build point/spring stores from an indexed triangle mesh.
 *
 * @param {ArrayLike<number>} meshPositions - xyz triplets, possibly with
 *   duplicated positions (split UV / normal seams).
 * @param {ArrayLike<number>} meshIndex - triangle vertex indices.
 */
export function buildClothTopology(meshPositions, meshIndex) {
  const vertexCount = meshPositions.length / 3;
  const pointOf = new Map();
  const vertexToPoint = new Uint32Array(vertexCount);
  const points = [];

  for (let v = 0; v < vertexCount; v += 1) {
    const x = meshPositions[v * 3];
    const y = meshPositions[v * 3 + 1];
    const z = meshPositions[v * 3 + 2];
    const key = `${x},${y},${z}`;
    if (!pointOf.has(key)) {
      pointOf.set(key, points.length / 3);
      points.push(x, y, z);
    }
    vertexToPoint[v] = pointOf.get(key);
  }

  const pointCount = points.length / 3;
  const distance = (a, b) => Math.hypot(
    points[a * 3] - points[b * 3],
    points[a * 3 + 1] - points[b * 3 + 1],
    points[a * 3 + 2] - points[b * 3 + 2],
  );

  const springs = [];
  const kinds = [];
  const springOf = new Map();
  const edgeOpposites = new Map();
  const addSpring = (a, b, kind) => {
    if (a === b) return;
    const key = a < b ? `${a}-${b}` : `${b}-${a}`;
    if (springOf.has(key)) return;
    springOf.set(key, kinds.length);
    springs.push(a, b);
    kinds.push(kind);
  };

  const faces = [];
  for (let t = 0; t < meshIndex.length; t += 3) {
    const tri = [vertexToPoint[meshIndex[t]], vertexToPoint[meshIndex[t + 1]], vertexToPoint[meshIndex[t + 2]]];
    faces.push(...tri);
    const edges = [[tri[0], tri[1], tri[2]], [tri[1], tri[2], tri[0]], [tri[2], tri[0], tri[1]]];
    const lengths = edges.map(([a, b]) => distance(a, b));
    const longest = Math.max(...lengths);
    const shorter = lengths.filter((length) => length < longest - 1e-9).length;

    edges.forEach(([a, b, opposite], e) => {
      // A triangle's hypotenuse is the quad diagonal: it resists shear,
      // the other two edges resist stretch along the weave.
      addSpring(a, b, shorter === 2 && lengths[e] === longest ? SPRING_SHEAR : SPRING_STRETCH);
      const key = a < b ? `${a}-${b}` : `${b}-${a}`;
      if (!edgeOpposites.has(key)) edgeOpposites.set(key, []);
      edgeOpposites.get(key).push(opposite);
    });
  }

  // Once every edge is typed, join the far corners of each triangle pair.
  for (const opposites of edgeOpposites.values()) {
    for (let i = 0; i < opposites.length; i += 1) {
      for (let j = i + 1; j < opposites.length; j += 1) addSpring(opposites[i], opposites[j], SPRING_BEND);
    }
  }

  const perPoint = Array.from({ length: pointCount }, () => []);
  for (let s = 0; s < kinds.length; s += 1) {
    perPoint[springs[s * 2]].push(s);
    perPoint[springs[s * 2 + 1]].push(s);
  }
  const springPointer = new Uint32Array(pointCount);
  const springsPerPoint = [];
  perPoint.forEach((ids, p) => {
    springPointer[p] = springsPerPoint.length;
    springsPerPoint.push(ids.length, ...ids);
  });

  return {
    positions: new Float32Array(points),
    vertexToPoint,
    faces: new Uint32Array(faces),
    springs: new Uint32Array(springs),
    springKinds: new Uint8Array(kinds),
    springPointer,
    springsPerPoint: new Uint32Array(springsPerPoint),
  };
}

/** Row-major grid in the same vertex order and triangulation as THREE.PlaneGeometry. */
function gridMesh(columns, rows, width, height, z) {
  const positions = [];
  const index = [];
  for (let r = 0; r <= rows; r += 1) {
    for (let c = 0; c <= columns; c += 1) {
      positions.push((c * width) / columns - width / 2, (r * height) / rows - height / 2, z);
    }
  }
  for (let r = 0; r < rows; r += 1) {
    for (let c = 0; c < columns; c += 1) {
      const a = r * (columns + 1) + c;
      const b = a + columns + 1;
      index.push(a, b, a + 1, b, b + 1, a + 1);
    }
  }
  return { positions, index };
}

export class ClothSimulator {
  constructor(options = {}) {
    this.config = { ...DEFAULT_CLOTH_CONFIG, ...options };
    const { columns, rows, width, height, tableZ, historyLimit, settleDisplacementLimit } = this.config;
    Object.assign(this, { columns, rows, width, height, tableZ, historyLimit, settleDisplacementLimit });

    const mesh = gridMesh(this.columns, this.rows, this.width, this.height, this.tableZ);
    const topology = buildClothTopology(mesh.positions, mesh.index);
    this.topology = topology;
    this.numVertices = topology.positions.length / 3;
    this.restPositions = topology.positions;
    this.springs = topology.springs;
    this.springKinds = topology.springKinds;
    this.numSprings = topology.springKinds.length;

    // --- stores (one element per point / per spring) ---
    this.positions = new Float32Array(this.numVertices * 3);
    this.forces = new Float32Array(this.numVertices * 3);
    this.springForces = new Float32Array(this.numSprings * 3);
    this.restLengths = new Float32Array(this.numSprings);
    this.springStiffness = new Float32Array(this.numSprings);
    // 1 = kinematic (held by a magnet), 0 = simulated: the reference's
    // vertex-paint `w` channel, kept as inverse mass for callers.
    this.invMass = new Float32Array(this.numVertices);
    // Magnet id + 1 per point; 0 = free.
    this.pointMagnet = new Uint8Array(this.numVertices);
    // xyz target + strength per gripper; `magnetFrom` is where the target was
    // at the start of the step, for interpolation across sub-steps.
    this.magnets = new Float32Array(2 * 4);
    this.magnetFrom = new Float32Array(2 * 3);

    // Graspable paint: the two front corners, one per arm.
    this.anchorsA = [0];
    this.anchorsB = [this.columns];
    this.graspable = new Uint8Array(this.numVertices);
    for (const idx of [...this.anchorsA, ...this.anchorsB]) this.graspable[idx] = 1;

    this._calculateRestLengths();
    this.configure(this.config);

    // Flat [i, j, restLength] views by kind, for metrics and inspection.
    this.structuralSprings = this._springList(SPRING_STRETCH);
    this.shearSprings = this._springList(SPRING_SHEAR);
    this.bendingSprings = this._springList(SPRING_BEND);

    this.reset();
  }

  /**
   * Apply physical parameters (any subset of CLOTH_PHYSICS_KEYS) without
   * disturbing the cloth's state, so settings can be tuned mid-episode.
   * Mesh size and resolution are fixed at construction.
   */
  configure(params = {}) {
    for (const key of CLOTH_PHYSICS_KEYS) {
      if (Number.isFinite(params[key])) {
        this[key] = params[key];
        this.config[key] = params[key];
      }
    }
    this.substeps = Math.max(1, Math.round(this.substeps));
    const stiffnessOf = [this.stretchStiffness, this.shearStiffness, this.bendStiffness];
    for (let s = 0; s < this.numSprings; s += 1) this.springStiffness[s] = stiffnessOf[this.springKinds[s]];
    return this;
  }

  /**
   * Largest summed spring stiffness at any point. The explicit update is
   * stable while this stays under roughly 4; past it the cloth jitters and
   * the maxForce clamp is all that holds it together.
   */
  getStiffnessLoad() {
    const { springPointer, springsPerPoint } = this.topology;
    let load = 0;
    for (let i = 0; i < this.numVertices; i += 1) {
      const start = springPointer[i];
      let sum = 0;
      for (let k = start + 1; k <= start + springsPerPoint[start]; k += 1) sum += this.springStiffness[springsPerPoint[k]];
      load = Math.max(load, sum);
    }
    return load;
  }

  _calculateRestLengths() {
    const p = this.restPositions;
    for (let s = 0; s < this.numSprings; s += 1) {
      const a = this.springs[s * 2] * 3;
      const b = this.springs[s * 2 + 1] * 3;
      this.restLengths[s] = Math.max(Math.hypot(p[b] - p[a], p[b + 1] - p[a + 1], p[b + 2] - p[a + 2]), 1e-6);
    }
  }

  _springList(kind) {
    const list = [];
    for (let s = 0; s < this.numSprings; s += 1) {
      if (this.springKinds[s] === kind) list.push(this.springs[s * 2], this.springs[s * 2 + 1], this.restLengths[s]);
    }
    return list;
  }

  reset() {
    this.positions.set(this.restPositions);
    this.forces.fill(0);
    this.springForces.fill(0);
    this.invMass.fill(1);
    this.pointMagnet.fill(0);
    this.magnets.fill(0);
    this.magnetFrom.fill(0);
    this.captured = [false, false];
    this.wasCaptured = [false, false];
    this.graspOrigins = [null, null];
    this.released = [false, false];
    this.tablePinA = null;
    this.history = [];
    this.settled = false;
  }

  /** Nearest free graspable point within graspRadius of `target`, or -1. */
  _nearestGraspable(target) {
    let best = -1;
    let bestDist = this.graspRadius;
    for (let i = 0; i < this.numVertices; i += 1) {
      if (!this.graspable[i] || this.pointMagnet[i]) continue;
      const d = Math.hypot(
        this.positions[i * 3] - target.x,
        this.positions[i * 3 + 1] - target.y,
        this.positions[i * 3 + 2] - target.z,
      );
      if (d < bestDist) {
        bestDist = d;
        best = i;
      }
    }
    return best;
  }

  /** Bind point `idx` to magnet `m`; the target starts at the point so it is eased in. */
  _activateMagnet(m, idx) {
    this.pointMagnet[idx] = m + 1;
    this.magnetFrom.set(this.positions.subarray(idx * 3, idx * 3 + 3), m * 3);
    this.magnets.set(this.positions.subarray(idx * 3, idx * 3 + 3), m * 4);
    this.magnets[m * 4 + 3] = 1;
  }

  _deactivateMagnet(m) {
    for (let i = 0; i < this.numVertices; i += 1) if (this.pointMagnet[i] === m + 1) this.pointMagnet[i] = 0;
    this.magnets.fill(0, m * 4, m * 4 + 4);
  }

  /** Grasp / release state machine; returns the target each magnet should reach this step. */
  _updateGrasps(targetA, targetB) {
    const targets = [targetA, targetB];
    // A captured corner keeps its original contact point. Re-testing capture
    // each frame would overwrite that point as the arm retracts, turning a
    // deliberate release into an unrealistic "air-bending" tow.
    targets.forEach((target, m) => {
      if (!target || this.released[m] || this.captured[m]) return;
      const idx = this._nearestGraspable(target);
      if (idx < 0) return;
      this.captured[m] = true;
      this.wasCaptured[m] = true;
      this.graspOrigins[m] = { x: target.x, y: target.y, z: target.z };
      this._activateMagnet(m, idx);
    });

    // The fold plan retracts Arm A after it has pressed the left edge to the
    // table. Treat a departing gripper as an explicit release, allowing the
    // table to hold that edge while Arm B places the fold over it.
    if (this.captured[0] && targetA) {
      const origin = this.graspOrigins[0];
      if (Math.hypot(origin.x - targetA.x, origin.y - targetA.y, origin.z - targetA.z) > this.releaseRadius) {
        this.captured[0] = false;
        this.released[0] = true;
        this.tablePinA = { x: origin.x, y: origin.y, z: this.tableZ };
      }
    }

    const goals = [
      this.captured[0] ? targetA : this.tablePinA,
      this.captured[1] ? targetB : null,
    ];
    goals.forEach((goal, m) => {
      if (!goal) {
        if (this.magnets[m * 4 + 3] > 0) this._deactivateMagnet(m);
        return;
      }
      this.magnetFrom.set(this.magnets.subarray(m * 4, m * 4 + 3), m * 3);
    });
    return goals;
  }

  /**
   * Advance one simulation step under current arm gripper targets.
   *
   * @param {Object} targets - Gripper positions in cloth-local coordinates:
   *                           { targetA: {x, y, z}, targetB: {x, y, z} }
   */
  step(targets = {}) {
    const goals = this._updateGrasps(targets.targetA, targets.targetB);
    for (let i = 0; i < this.numVertices; i += 1) this.invMass[i] = this.pointMagnet[i] ? 0 : 1;

    for (let sub = 1; sub <= this.substeps; sub += 1) {
      const t = sub / this.substeps;
      goals.forEach((goal, m) => {
        if (!goal) return;
        const from = this.magnetFrom;
        this.magnets[m * 4] = from[m * 3] + (goal.x - from[m * 3]) * t;
        this.magnets[m * 4 + 1] = from[m * 3 + 1] + (goal.y - from[m * 3 + 1]) * t;
        this.magnets[m * 4 + 2] = Math.max(this.tableZ, from[m * 3 + 2] + (goal.z - from[m * 3 + 2]) * t);
      });
      this._computeSpringForces();
      this._computeVertexForces();
      this._solveLayerSeparation();
    }

    this._recordHistory();
  }

  /**
   * Kernel, one invocation per spring: half the length error along the
   * spring, plus a dashpot on the ends' relative velocity (the previous
   * sub-step's force store) along it. Positive pulls end A toward end B.
   */
  _computeSpringForces() {
    const p = this.positions;
    const f = this.forces;
    for (let s = 0; s < this.numSprings; s += 1) {
      const a = this.springs[s * 2] * 3;
      const b = this.springs[s * 2 + 1] * 3;
      const dx = p[b] - p[a];
      const dy = p[b + 1] - p[a + 1];
      const dz = p[b + 2] - p[a + 2];
      const dist = Math.max(Math.hypot(dx, dy, dz), 1e-6);
      const separating = ((f[b] - f[a]) * dx + (f[b + 1] - f[a + 1]) * dy + (f[b + 2] - f[a + 2]) * dz) / dist;
      const k = ((dist - this.restLengths[s]) * this.springStiffness[s] + separating * this.springDamping) * 0.5 / dist;
      this.springForces[s * 3] = dx * k;
      this.springForces[s * 3 + 1] = dy * k;
      this.springForces[s * 3 + 2] = dz * k;
    }
  }

  /**
   * Kernel, one invocation per point: accumulate forces into the damped
   * per-point force (a displacement per sub-step), resolve the table, clamp,
   * then move the point - or snap it to its magnet.
   */
  _computeVertexForces() {
    const n = this.substeps;
    const p = this.positions;
    const f = this.forces;
    const airDamping = (1 - this.damping) ** (1 / n);
    const gravity = this.gravity / (n * n);
    const { springPointer, springsPerPoint } = this.topology;

    for (let i = 0; i < this.numVertices; i += 1) {
      const o = i * 3;
      const magnet = this.pointMagnet[i];
      if (magnet) {
        const m = (magnet - 1) * 4;
        const strength = this.magnets[m + 3];
        for (let axis = 0; axis < 3; axis += 1) {
          const move = (this.magnets[m + axis] - p[o + axis]) * strength;
          f[o + axis] = move;
          p[o + axis] += move;
        }
        continue;
      }

      let sx = 0;
      let sy = 0;
      let sz = 0;
      const start = springPointer[i] + 1;
      const end = start + springsPerPoint[springPointer[i]];
      for (let k = start; k < end; k += 1) {
        const s = springsPerPoint[k];
        const sign = this.springs[s * 2] === i ? 1 : -1;
        sx += this.springForces[s * 3] * sign;
        sy += this.springForces[s * 3 + 1] * sign;
        sz += this.springForces[s * 3 + 2] * sign;
      }

      let fx = f[o] * airDamping + sx;
      let fy = f[o + 1] * airDamping + sy;
      let fz = f[o + 2] * airDamping + sz + gravity;

      const length = Math.hypot(fx, fy, fz);
      if (length > this.maxForce) {
        const scale = this.maxForce / length;
        fx *= scale;
        fy *= scale;
        fz *= scale;
      }

      // Table plane collider with impulse-based Coulomb friction: the normal
      // push is how much downward motion the table cancels this sub-step.
      if (p[o + 2] + fz <= this.tableZ + 1e-4) {
        const normal = Math.max(0, this.tableZ - p[o + 2] - fz);
        fz = Math.max(fz, this.tableZ - p[o + 2]);
        const lateral = Math.hypot(fx, fy);
        if (lateral <= this.staticFriction * normal) {
          fx = 0;
          fy = 0;
        } else {
          const slide = 1 - (this.kineticFriction * normal) / lateral;
          fx *= slide;
          fy *= slide;
        }
      }

      f[o] = fx;
      f[o + 1] = fy;
      f[o + 2] = fz;
      p[o] += fx;
      p[o + 1] += fy;
      p[o + 2] = Math.max(this.tableZ, p[o + 2] + fz);
    }
  }

  /**
   * Fold layer separation: a right-half point (c > columns / 2) over a
   * left-half point of the same row must stay one thickness above it. Pairs
   * within the bending stencil (two columns) are neighbours in the weave,
   * not stacked layers, and are skipped - at rest they are closer than the
   * overlap radius and would otherwise lift the flat towel's midline. The
   * correction per sub-step is capped (the reference's maxForce idea applied
   * to a position) so it converges alongside the springs instead of
   * fighting them with a full snap every pass.
   */
  _solveLayerSeparation() {
    const p = this.positions;
    const halfCol = this.columns / 2;
    const maxStep = 0.008 / Math.sqrt(this.substeps);
    for (let r = 0; r <= this.rows; r += 1) {
      for (let c2 = this.columns; c2 > halfCol; c2 -= 1) {
        const i2 = r * (this.columns + 1) + c2;
        if (this.invMass[i2] === 0) continue;
        const o2 = i2 * 3;
        for (let c1 = 0; c1 <= halfCol && c1 < c2 - 2; c1 += 1) {
          const o1 = (r * (this.columns + 1) + c1) * 3;
          if (Math.hypot(p[o2] - p[o1], p[o2 + 1] - p[o1 + 1]) >= 0.12) continue;
          const minZ = p[o1 + 2] + this.thickness;
          if (p[o2 + 2] < minZ) {
            const lift = Math.min(minZ - p[o2 + 2], maxStep);
            p[o2 + 2] += lift;
            if (this.forces[o2 + 2] < 0) this.forces[o2 + 2] = 0;
          }
        }
      }
    }
  }

  /** Rolling history & settling detection. */
  _recordHistory() {
    this.history.push(Float32Array.from(this.positions));
    if (this.history.length > this.historyLimit) this.history.shift();
    if (this.history.length < this.historyLimit) return;
    const oldest = this.history[0];
    const newest = this.history.at(-1);
    let totalDisp = 0;
    for (let k = 0; k < newest.length; k += 3) {
      totalDisp += Math.hypot(newest[k] - oldest[k], newest[k + 1] - oldest[k + 1], newest[k + 2] - oldest[k + 2]);
    }
    this.settled = totalDisp / this.numVertices < this.settleDisplacementLimit;
  }

  /** Snapshot of current state for loading and scrubbing frames. */
  snapshot() {
    return {
      positions: Float32Array.from(this.positions),
      forces: Float32Array.from(this.forces),
      pointMagnet: Uint8Array.from(this.pointMagnet),
      magnets: Float32Array.from(this.magnets),
      captured: [...this.captured],
      wasCaptured: [...this.wasCaptured],
      graspOrigins: this.graspOrigins.map((origin) => origin && { ...origin }),
      released: [...this.released],
      tablePinA: this.tablePinA && { ...this.tablePinA },
      settled: this.settled,
    };
  }

  /** Restore a past snapshot instantly. */
  restore(snap) {
    if (!snap) return;
    this.positions.set(snap.positions);
    if (snap.forces) this.forces.set(snap.forces);
    else this.forces.fill(0);
    this.captured = [...snap.captured];
    this.wasCaptured = [...(snap.wasCaptured || snap.captured)];
    this.graspOrigins = (snap.graspOrigins || [null, null]).map((origin) => origin && { ...origin });
    this.released = [...(snap.released || [false, false])];
    this.tablePinA = snap.tablePinA && { ...snap.tablePinA };
    this.settled = snap.settled;
    if (snap.pointMagnet) {
      this.pointMagnet.set(snap.pointMagnet);
      this.magnets.set(snap.magnets);
    } else {
      // Snapshots from before magnets existed: rebuild from the grasp flags.
      this.pointMagnet.fill(0);
      this.magnets.fill(0);
      if (this.captured[0] || this.tablePinA) this._activateMagnet(0, this.anchorsA[0]);
      if (this.captured[1]) this._activateMagnet(1, this.anchorsB[0]);
    }
    for (let i = 0; i < this.numVertices; i += 1) this.invMass[i] = this.pointMagnet[i] ? 0 : 1;
  }

  /** Distance between the pinned left edge (c=0) and the right edge (c=columns). */
  getFoldAlignment() {
    let sumDist = 0;
    for (let r = 0; r <= this.rows; r += 1) {
      const iLeft = r * (this.columns + 1);
      const iRight = iLeft + this.columns;
      sumDist += Math.hypot(
        this.positions[iRight * 3] - this.positions[iLeft * 3],
        this.positions[iRight * 3 + 1] - this.positions[iLeft * 3 + 1],
      );
    }
    return sumDist / (this.rows + 1);
  }

  /** Front corner alignment distance (where the gripper holds). */
  getFrontFoldDistance() {
    const cornerRight = this.columns;
    return Math.hypot(
      this.positions[cornerRight * 3] - this.positions[0],
      this.positions[cornerRight * 3 + 1] - this.positions[1],
    );
  }

  /** Max relative stretch error across all structural springs. */
  getMaxStretchError() {
    let maxError = 0;
    for (let s = 0; s < this.structuralSprings.length; s += 3) {
      const i1 = this.structuralSprings[s] * 3;
      const i2 = this.structuralSprings[s + 1] * 3;
      const restLen = this.structuralSprings[s + 2];
      const len = Math.hypot(
        this.positions[i2] - this.positions[i1],
        this.positions[i2 + 1] - this.positions[i1 + 1],
        this.positions[i2 + 2] - this.positions[i1 + 2],
      );
      maxError = Math.max(maxError, Math.abs(len - restLen) / restLen);
    }
    return maxError;
  }

  /** Comprehensive evaluation of fold metrics. */
  getFoldMetrics() {
    const frontDist = this.getFrontFoldDistance();
    // In local units, rest width is 1.5. A fold in half brings the edge within ~0.6 or less.
    return {
      folded: frontDist < 0.65,
      frontDistance: frontDist,
      averageDistance: this.getFoldAlignment(),
      stretchError: this.getMaxStretchError(),
      settled: this.settled,
    };
  }

  /**
   * 2D polygon projection in scene coordinates for SVG rendering.
   *
   * @param {Array<number>} origin - Cloth center in scene pixels, e.g. [325, 240]
   * @param {number} pxPerUnit - Scaling factor (default 100)
   */
  get2DPolygons(origin = [325, 240], pxPerUnit = 100) {
    const toScene = (i) => [origin[0] + this.positions[i * 3] * pxPerUnit, origin[1] + this.positions[i * 3 + 1] * pxPerUnit];
    const stride = this.columns + 1;

    // Base boundary polygon (counter-clockwise around perimeter)
    const basePoints = [];
    for (let c = 0; c <= this.columns; c += 1) basePoints.push(toScene(c));
    for (let r = 1; r <= this.rows; r += 1) basePoints.push(toScene(r * stride + this.columns));
    for (let c = this.columns - 1; c >= 0; c -= 1) basePoints.push(toScene(this.rows * stride + c));
    for (let r = this.rows - 1; r >= 1; r -= 1) basePoints.push(toScene(r * stride));

    // Fold crease line (along midline c = columns / 2)
    const midC = Math.round(this.columns / 2);
    const creasePoints = [];
    for (let r = 0; r <= this.rows; r += 1) creasePoints.push(toScene(r * stride + midC));

    return { basePoints, creasePoints };
  }
}
