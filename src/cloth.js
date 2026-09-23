/**
 * Physics-based cloth simulation engine for robotic manipulation.
 *
 * Implements Position-Based Dynamics (PBD) with Verlet integration, modeling
 * textile mechanics:
 *  - Structural springs (warp and weft tensile resistance)
 *  - Shear springs (in-plane diagonal shear resistance)
 *  - Bending / flexion springs (out-of-plane curvature resistance)
 *  - Table contact with Coulomb friction (static stick & dynamic slide)
 *  - Fabric thickness barrier and self-collision layer separation
 *  - Multi-vertex kinematic gripper grasping and edge pinning
 *  - Rolling history settling detector and fold alignment metrics
 *  - Frame snapshot / restore for timeline scrubbing and loading sliderbar
 */

export const DEFAULT_CLOTH_CONFIG = Object.freeze({
  columns: 14,
  rows: 11,
  width: 1.5,
  height: 1.2,
  thickness: 0.035,
  tableZ: 0.018,
  gravity: -0.0032,
  damping: 0.02,
  tableFriction: 0.35,
  staticFriction: 0.003,
  // Two extra PBD passes keep the compressed fold within the tensile bound
  // while it is lowered onto the pinned layer.
  solverIterations: 8,
  stretchStiffness: 0.95,
  shearStiffness: 0.75,
  bendStiffness: 0.45,
  graspRadius: 0.36,
  releaseRadius: 0.52,
  historyLimit: 120,
  settleDisplacementLimit: 0.003,
});

export class ClothSimulator {
  constructor(options = {}) {
    this.config = { ...DEFAULT_CLOTH_CONFIG, ...options };
    this.columns = this.config.columns;
    this.rows = this.config.rows;
    this.width = this.config.width;
    this.height = this.config.height;
    this.thickness = this.config.thickness;
    this.tableZ = this.config.tableZ;
    this.gravity = this.config.gravity;
    this.damping = this.config.damping;
    this.tableFriction = this.config.tableFriction;
    this.staticFriction = this.config.staticFriction;
    this.solverIterations = this.config.solverIterations;
    this.stretchStiffness = this.config.stretchStiffness;
    this.shearStiffness = this.config.shearStiffness;
    this.bendStiffness = this.config.bendStiffness;
    this.graspRadius = this.config.graspRadius;
    this.releaseRadius = this.config.releaseRadius;
    this.historyLimit = this.config.historyLimit;
    this.settleDisplacementLimit = this.config.settleDisplacementLimit;

    this.numVertices = (this.columns + 1) * (this.rows + 1);
    this.positions = new Float32Array(this.numVertices * 3);
    this.prevPositions = new Float32Array(this.numVertices * 3);
    this.restPositions = new Float32Array(this.numVertices * 3);
    this.velocities = new Float32Array(this.numVertices * 3);
    this.invMass = new Float32Array(this.numVertices);

    this.captured = [false, false];
    this.wasCaptured = [false, false];
    this.graspOrigins = [null, null];
    this.released = [false, false];
    this.tablePinA = null;
    this.anchorsA = [];
    this.anchorsB = [];

    this.history = [];
    this.settled = false;

    this._initMesh();
    this._initConstraints();
    this.reset();
  }

  _initMesh() {
    const dx = this.width / this.columns;
    const dy = this.height / this.rows;
    const halfW = this.width / 2;
    const halfH = this.height / 2;

    this.anchorsA = [];
    this.anchorsB = [];

    for (let r = 0; r <= this.rows; r += 1) {
      for (let c = 0; c <= this.columns; c += 1) {
        const i = r * (this.columns + 1) + c;
        const x = c * dx - halfW;
        const y = r * dy - halfH;
        const z = this.tableZ;

        this.restPositions[i * 3] = x;
        this.restPositions[i * 3 + 1] = y;
        this.restPositions[i * 3 + 2] = z;

        // Front-left corner anchor (Arm A gripper pinch)
        if (c === 0 && r === 0) {
          this.anchorsA.push(i);
        }
        // Front-right corner anchor (Arm B gripper pinch)
        if (c === this.columns && r === 0) {
          this.anchorsB.push(i);
        }
      }
    }
  }

  _initConstraints() {
    this.structuralSprings = [];
    this.shearSprings = [];
    this.bendingSprings = [];

    const getIdx = (r, c) => r * (this.columns + 1) + c;
    const restDist = (i1, i2) => {
      const dx = this.restPositions[i1 * 3] - this.restPositions[i2 * 3];
      const dy = this.restPositions[i1 * 3 + 1] - this.restPositions[i2 * 3 + 1];
      const dz = this.restPositions[i1 * 3 + 2] - this.restPositions[i2 * 3 + 2];
      return Math.hypot(dx, dy, dz);
    };

    for (let r = 0; r <= this.rows; r += 1) {
      for (let c = 0; c <= this.columns; c += 1) {
        const i = getIdx(r, c);

        // 1. Structural constraints (horizontal and vertical neighbors)
        if (c < this.columns) {
          const right = getIdx(r, c + 1);
          this.structuralSprings.push(i, right, restDist(i, right));
        }
        if (r < this.rows) {
          const down = getIdx(r + 1, c);
          this.structuralSprings.push(i, down, restDist(i, down));
        }

        // 2. Shear constraints (quad diagonals)
        if (c < this.columns && r < this.rows) {
          const diag1 = getIdx(r + 1, c + 1);
          const diag2 = getIdx(r + 1, c);
          const right = getIdx(r, c + 1);
          this.shearSprings.push(i, diag1, restDist(i, diag1));
          this.shearSprings.push(right, diag2, restDist(right, diag2));
        }

        // 3. Bending / Flexion constraints (2-step neighbors across yarns)
        if (c < this.columns - 1) {
          const right2 = getIdx(r, c + 2);
          this.bendingSprings.push(i, right2, restDist(i, right2));
        }
        if (r < this.rows - 1) {
          const down2 = getIdx(r + 2, c);
          this.bendingSprings.push(i, down2, restDist(i, down2));
        }
      }
    }
  }

  reset() {
    this.positions.set(this.restPositions);
    this.prevPositions.set(this.restPositions);
    this.velocities.fill(0);
    this.invMass.fill(1.0);
    this.captured = [false, false];
    this.wasCaptured = [false, false];
    this.graspOrigins = [null, null];
    this.released = [false, false];
    this.tablePinA = null;
    this.history = [];
    this.settled = false;
  }

  /**
   * Advance one simulation step under current arm gripper targets.
   *
   * @param {Object} targets - Gripper positions in cloth-local coordinates:
   *                           { targetA: {x, y, z}, targetB: {x, y, z} }
   */
  step(targets = {}) {
    const { targetA, targetB } = targets;

    // 1. Gripper contact / capture check
    // A captured corner keeps its original contact point. Re-testing capture
    // each frame would overwrite that point as the arm retracts, turning a
    // deliberate release into an unrealistic "air-bending" tow.
    if (targetA && !this.released[0] && !this.captured[0]) {
      const cornerA = 0;
      const d = Math.hypot(
        this.positions[cornerA * 3] - targetA.x,
        this.positions[cornerA * 3 + 1] - targetA.y,
        this.positions[cornerA * 3 + 2] - targetA.z,
      );
      if (d < this.graspRadius) {
        this.captured[0] = true;
        this.wasCaptured[0] = true;
        this.graspOrigins[0] = { x: targetA.x, y: targetA.y, z: targetA.z };
      }
    }
    if (targetB && !this.released[1] && !this.captured[1]) {
      const cornerB = this.columns;
      const d = Math.hypot(
        this.positions[cornerB * 3] - targetB.x,
        this.positions[cornerB * 3 + 1] - targetB.y,
        this.positions[cornerB * 3 + 2] - targetB.z,
      );
      if (d < this.graspRadius) {
        this.captured[1] = true;
        this.wasCaptured[1] = true;
        this.graspOrigins[1] = { x: targetB.x, y: targetB.y, z: targetB.z };
      }
    }
    // The fold plan retracts Arm A after it has pressed the left edge to the
    // table. Treat a departing gripper as an explicit release, allowing table
    // friction to hold that edge while Arm B places the fold over it.
    if (this.captured[0] && targetA) {
      const origin = this.graspOrigins[0];
      const d = origin ? Math.hypot(origin.x - targetA.x, origin.y - targetA.y, origin.z - targetA.z) : 0;
      if (d > this.releaseRadius) {
        this.captured[0] = false;
        this.released[0] = true;
        // The left edge was pressed onto the table before release. Preserve
        // that contact as a table pin while the other gripper crosses over it.
        this.tablePinA = { x: origin.x, y: origin.y, z: this.tableZ };
      }
    }

    // Set inverse masses
    this.invMass.fill(1.0);
    if (this.captured[0] || this.tablePinA) {
      for (const idx of this.anchorsA) this.invMass[idx] = 0;
    }
    if (this.captured[1]) {
      for (const idx of this.anchorsB) this.invMass[idx] = 0;
    }

    // 2. Symplectic / Verlet integration with gravity, air damping, and table friction
    const dampingFactor = 1 - this.damping;
    for (let i = 0; i < this.numVertices; i += 1) {
      if (this.invMass[i] === 0) continue;

      const pIdx = i * 3;
      const x = this.positions[pIdx];
      const y = this.positions[pIdx + 1];
      const z = this.positions[pIdx + 2];

      const px = this.prevPositions[pIdx];
      const py = this.prevPositions[pIdx + 1];
      const pz = this.prevPositions[pIdx + 2];

      let vx = (x - px) * dampingFactor;
      let vy = (y - py) * dampingFactor;
      let vz = (z - pz) * dampingFactor + this.gravity;

      // Table contact & Coulomb friction
      if (z <= this.tableZ + 1e-4) {
        vz = Math.max(0, vz);
        const lateralDisp = Math.hypot(vx, vy);
        if (lateralDisp < this.staticFriction) {
          vx = 0;
          vy = 0;
        } else {
          const frictionFactor = Math.max(0, 1 - this.tableFriction);
          vx *= frictionFactor;
          vy *= frictionFactor;
        }
      }

      this.prevPositions[pIdx] = x;
      this.prevPositions[pIdx + 1] = y;
      this.prevPositions[pIdx + 2] = z;

      this.positions[pIdx] = x + vx;
      this.positions[pIdx + 1] = y + vy;
      this.positions[pIdx + 2] = Math.max(this.tableZ, z + vz);
    }

    // 3. Apply Gripper Kinematic Anchors
    const applyGripperAnchor = (anchors, target) => {
      if (!target) return;
      const refIdx = anchors[0];
      const rx = this.restPositions[refIdx * 3];
      const ry = this.restPositions[refIdx * 3 + 1];
      for (const idx of anchors) {
        const offX = this.restPositions[idx * 3] - rx;
        const offY = this.restPositions[idx * 3 + 1] - ry;
        this.positions[idx * 3] = target.x + offX;
        this.positions[idx * 3 + 1] = target.y + offY;
        this.positions[idx * 3 + 2] = Math.max(this.tableZ, target.z);
        this.prevPositions[idx * 3] = this.positions[idx * 3];
        this.prevPositions[idx * 3 + 1] = this.positions[idx * 3 + 1];
        this.prevPositions[idx * 3 + 2] = this.positions[idx * 3 + 2];
      }
    };

    if (this.captured[0]) applyGripperAnchor(this.anchorsA, targetA);
    if (this.tablePinA) applyGripperAnchor(this.anchorsA, this.tablePinA);
    if (this.captured[1]) applyGripperAnchor(this.anchorsB, targetB);

    // 4. Position-Based Dynamics (PBD) Constraint Relaxation
    const solveSpringList = (list, stiffness) => {
      for (let s = 0; s < list.length; s += 3) {
        const i1 = list[s];
        const i2 = list[s + 1];
        const restLen = list[s + 2];

        const w1 = this.invMass[i1];
        const w2 = this.invMass[i2];
        const wTotal = w1 + w2;
        if (wTotal === 0) continue;

        const p1 = i1 * 3;
        const p2 = i2 * 3;

        const dx = this.positions[p2] - this.positions[p1];
        const dy = this.positions[p2 + 1] - this.positions[p1 + 1];
        const dz = this.positions[p2 + 2] - this.positions[p1 + 2];

        const currentLen = Math.hypot(dx, dy, dz) || 1e-6;
        const delta = ((currentLen - restLen) / currentLen) * stiffness;

        if (w1 > 0) {
          const factor1 = (w1 / wTotal) * delta;
          this.positions[p1] += dx * factor1;
          this.positions[p1 + 1] += dy * factor1;
          this.positions[p1 + 2] += dz * factor1;
        }
        if (w2 > 0) {
          const factor2 = (w2 / wTotal) * delta;
          this.positions[p2] -= dx * factor2;
          this.positions[p2 + 1] -= dy * factor2;
          this.positions[p2 + 2] -= dz * factor2;
        }
      }
    };

    for (let iter = 0; iter < this.solverIterations; iter += 1) {
      solveSpringList(this.structuralSprings, this.stretchStiffness);
      solveSpringList(this.shearSprings, this.shearStiffness);
      solveSpringList(this.bendingSprings, this.bendStiffness);

      // 5. Floor constraint & Layer Thickness / Fold Volume Constraint
      for (let i = 0; i < this.numVertices; i += 1) {
        if (this.invMass[i] > 0) {
          if (this.positions[i * 3 + 2] < this.tableZ) {
            this.positions[i * 3 + 2] = this.tableZ;
          }
        }
      }

      // Self-collision / fold layer separation:
      // When right-hand particles (c > columns / 2) fold over left-hand particles (c <= columns / 2),
      // enforce thickness separation z_right >= z_left + thickness. Snapping
      // straight to minZ on every iteration fights the structural springs at
      // the fold line - each full snap overshoots, the spring pulls the
      // vertex back, and the pair never reaches a joint equilibrium (this
      // measured as stretch-error spikes past 3x rest length during a fast
      // cross-over, never settling). Capping how far a single iteration may
      // move a vertex - rather than snapping it exactly - is the same fix
      // bandinopla/three-simplecloth (MIT license,
      // https://github.com/bandinopla/three-simplecloth) uses for its spring
      // and collision forces (a per-step maxForce clamp); applied here to a
      // position correction instead of a force, it lets the constraint
      // converge over a few iterations instead of fighting the springs in one.
      const halfCol = this.columns / 2;
      const selfCollisionMaxStep = 0.008;
      for (let r = 0; r <= this.rows; r += 1) {
        for (let c1 = 0; c1 <= halfCol; c1 += 1) {
          const i1 = r * (this.columns + 1) + c1;
          const p1 = i1 * 3;
          const x1 = this.positions[p1];
          const y1 = this.positions[p1 + 1];
          const z1 = this.positions[p1 + 2];

          for (let c2 = this.columns; c2 > halfCol; c2 -= 1) {
            const i2 = r * (this.columns + 1) + c2;
            const p2 = i2 * 3;
            const x2 = this.positions[p2];
            const y2 = this.positions[p2 + 1];
            const z2 = this.positions[p2 + 2];

            const horizDist = Math.hypot(x2 - x1, y2 - y1);
            if (horizDist < 0.12) {
              const minZ = z1 + this.thickness;
              if (z2 < minZ && this.invMass[i2] > 0) {
                this.positions[p2 + 2] += Math.min(minZ - z2, selfCollisionMaxStep);
              }
            }
          }
        }
      }

      // Re-assert gripper anchors
      if (this.captured[0]) applyGripperAnchor(this.anchorsA, targetA);
      if (this.tablePinA) applyGripperAnchor(this.anchorsA, this.tablePinA);
      if (this.captured[1]) applyGripperAnchor(this.anchorsB, targetB);
    }

    // 6. Rolling history & Settling detection
    const frameSnapshot = Float32Array.from(this.positions);
    this.history.push(frameSnapshot);
    if (this.history.length > this.historyLimit) this.history.shift();

    if (this.history.length === this.historyLimit) {
      const oldest = this.history[0];
      const newest = this.history.at(-1);
      let totalDisp = 0;
      for (let k = 0; k < newest.length; k += 3) {
        totalDisp += Math.hypot(
          newest[k] - oldest[k],
          newest[k + 1] - oldest[k + 1],
          newest[k + 2] - oldest[k + 2],
        );
      }
      const meanDisp = totalDisp / this.numVertices;
      this.settled = meanDisp < this.settleDisplacementLimit;
    }
  }

  /** Snapshot of current state for loading and scrubbing frames. */
  snapshot() {
    return {
      positions: Float32Array.from(this.positions),
      prevPositions: Float32Array.from(this.prevPositions),
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
    this.prevPositions.set(snap.prevPositions);
    this.captured = [...snap.captured];
    this.wasCaptured = [...(snap.wasCaptured || snap.captured)];
    this.graspOrigins = (snap.graspOrigins || [null, null]).map((origin) => origin && { ...origin });
    this.released = [...(snap.released || [false, false])];
    this.tablePinA = snap.tablePinA && { ...snap.tablePinA };
    this.settled = snap.settled;
  }

  /** Distance between the pinned left edge (c=0) and the right edge (c=columns). */
  getFoldAlignment() {
    let sumDist = 0;
    for (let r = 0; r <= this.rows; r += 1) {
      const iLeft = r * (this.columns + 1);
      const iRight = r * (this.columns + 1) + this.columns;
      const dx = this.positions[iRight * 3] - this.positions[iLeft * 3];
      const dy = this.positions[iRight * 3 + 1] - this.positions[iLeft * 3 + 1];
      sumDist += Math.hypot(dx, dy);
    }
    return sumDist / (this.rows + 1);
  }

  /** Front corner alignment distance (where the gripper holds). */
  getFrontFoldDistance() {
    const cornerLeft = 0;
    const cornerRight = this.columns;
    const dx = this.positions[cornerRight * 3] - this.positions[cornerLeft * 3];
    const dy = this.positions[cornerRight * 3 + 1] - this.positions[cornerLeft * 3 + 1];
    return Math.hypot(dx, dy);
  }

  /** Max relative stretch error across all structural springs. */
  getMaxStretchError() {
    let maxError = 0;
    for (let s = 0; s < this.structuralSprings.length; s += 3) {
      const i1 = this.structuralSprings[s];
      const i2 = this.structuralSprings[s + 1];
      const restLen = this.structuralSprings[s + 2];
      const dx = this.positions[i2 * 3] - this.positions[i1 * 3];
      const dy = this.positions[i2 * 3 + 1] - this.positions[i1 * 3 + 1];
      const dz = this.positions[i2 * 3 + 2] - this.positions[i1 * 3 + 2];
      const len = Math.hypot(dx, dy, dz);
      maxError = Math.max(maxError, Math.abs(len - restLen) / restLen);
    }
    return maxError;
  }

  /** Comprehensive evaluation of fold metrics. */
  getFoldMetrics() {
    const frontDist = this.getFrontFoldDistance();
    const avgDist = this.getFoldAlignment();
    const stretchError = this.getMaxStretchError();
    // In local units, rest width is 1.5. A fold in half brings the edge within ~0.6 or less.
    const folded = frontDist < 0.65;
    return {
      folded,
      frontDistance: frontDist,
      averageDistance: avgDist,
      stretchError,
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
    const toScene = (lx, ly) => [origin[0] + lx * pxPerUnit, origin[1] + ly * pxPerUnit];

    // Base boundary polygon (counter-clockwise around perimeter)
    const basePoints = [];
    // Bottom edge (r=0, c=0..columns)
    for (let c = 0; c <= this.columns; c += 1) {
      const i = c;
      basePoints.push(toScene(this.positions[i * 3], this.positions[i * 3 + 1]));
    }
    // Right edge (c=columns, r=1..rows)
    for (let r = 1; r <= this.rows; r += 1) {
      const i = r * (this.columns + 1) + this.columns;
      basePoints.push(toScene(this.positions[i * 3], this.positions[i * 3 + 1]));
    }
    // Top edge (r=rows, c=columns-1..0)
    for (let c = this.columns - 1; c >= 0; c -= 1) {
      const i = this.rows * (this.columns + 1) + c;
      basePoints.push(toScene(this.positions[i * 3], this.positions[i * 3 + 1]));
    }
    // Left edge (c=0, r=rows-1..1)
    for (let r = this.rows - 1; r >= 1; r -= 1) {
      const i = r * (this.columns + 1);
      basePoints.push(toScene(this.positions[i * 3], this.positions[i * 3 + 1]));
    }

    // Fold crease line (along midline c = columns / 2)
    const midC = Math.round(this.columns / 2);
    const creasePoints = [];
    for (let r = 0; r <= this.rows; r += 1) {
      const i = r * (this.columns + 1) + midC;
      creasePoints.push(toScene(this.positions[i * 3], this.positions[i * 3 + 1]));
    }

    return { basePoints, creasePoints };
  }
}
