/**
 * Rigid-body scene for the pick-and-place tasks: real colliders for the
 * objects, the table, fixed fixtures such as a tray, and each arm's gripper
 * fingers, stepped by cannon-es.
 *
 * Frames: the arm lives in scene pixels (x right, y toward the front edge, z
 * up; 1 px = 1 mm). The physics world uses metres in the 3-D viewport's axes
 * - x right, y up, z toward the front - so the viewport can copy a body's
 * quaternion straight onto its mesh. `toPhysics`/`toScene` convert points.
 *
 * Grasping is contact-gated rather than a magnet: fingers are kinematic box
 * colliders that push whatever they touch, and closing them only takes hold
 * of an object that is physically between the jaws at that moment. A held
 * object follows the tool as a kinematic body - still a collider, so a cube
 * set on another cube rests on it - and becomes dynamic again the moment the
 * jaws open, keeping the tool's velocity.
 *
 * DOM-free on purpose, like cloth.js: it runs under `node --test`.
 */
import * as CANNON from 'cannon-es';
import { forwardKinematics } from './core.js';

const MM = 0.001;
/** Physics substep. 500 Hz matches environment.physics_hz; 20 per 25 Hz control frame. */
export const PHYSICS_DT = 1 / 500;
export const CONTROL_DT = 1 / 25;

/**
 * Finger geometry, scene mm. Each finger is a slab running back from the
 * tool tip along the tool; `openHalfGap` is the tip-to-finger-centre
 * distance with the jaws open, wide enough to straddle a 30 mm cube at any
 * yaw (its half-diagonal is 21 mm).
 */
export const FINGER = Object.freeze({ length: 22, width: 5, thickness: 4, openHalfGap: 26, minHalfGap: 3 });

export const toPhysics = ([x, y, z = 0]) => new CANNON.Vec3(x * MM, z * MM, y * MM);
export const toScene = (v) => [v.x / MM, v.z / MM, v.y / MM];

/**
 * The tool frame in world axes (x right, y up, z front), as three unit
 * vectors: `forward` along the tool, `lateral` the axis the jaws open along
 * (the kinematic frame's own lateral axis, forward × up), and `normal`
 * completing a right-handed frame. Shared with the 3-D viewport so the drawn
 * fingers are exactly the colliders.
 */
export function toolBasis(forward, up) {
  // Scene -> world swaps y and z, a reflection, so a cross product has to be
  // taken in one frame: take it in scene axes, then map every vector.
  const lateralScene = [
    forward[1] * up[2] - forward[2] * up[1],
    forward[2] * up[0] - forward[0] * up[2],
    forward[0] * up[1] - forward[1] * up[0],
  ];
  const w = ([x, y, z]) => { const l = Math.hypot(x, y, z) || 1; return [x / l, z / l, y / l]; };
  const f = w(forward);
  const l = w(lateralScene);
  // normal = lateral × forward, so (forward, normal, lateral) is right-handed.
  const n = [l[1] * f[2] - l[2] * f[1], l[2] * f[0] - l[0] * f[2], l[0] * f[1] - l[1] * f[0]];
  return { forward: f, normal: n, lateral: l };
}

/** Quaternion whose local x/y/z axes are the basis' forward/normal/lateral. */
function basisQuaternion({ forward: x, normal: y, lateral: z }) {
  // Rotation matrix columns are the basis vectors; standard matrix -> quaternion.
  const m00 = x[0]; const m01 = y[0]; const m02 = z[0];
  const m10 = x[1]; const m11 = y[1]; const m12 = z[1];
  const m20 = x[2]; const m21 = y[2]; const m22 = z[2];
  const trace = m00 + m11 + m22;
  let qw; let qx; let qy; let qz;
  if (trace > 0) {
    const s = 0.5 / Math.sqrt(trace + 1);
    qw = 0.25 / s; qx = (m21 - m12) * s; qy = (m02 - m20) * s; qz = (m10 - m01) * s;
  } else if (m00 > m11 && m00 > m22) {
    const s = 2 * Math.sqrt(1 + m00 - m11 - m22);
    qw = (m21 - m12) / s; qx = 0.25 * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = 2 * Math.sqrt(1 + m11 - m00 - m22);
    qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25 * s; qz = (m12 + m21) / s;
  } else {
    const s = 2 * Math.sqrt(1 + m22 - m00 - m11);
    qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25 * s;
  }
  return new CANNON.Quaternion(qx, qy, qz, qw).normalize();
}

/** Tool pose from an arm's joints: tip (world, m), orientation, and basis. */
export function toolPose(q, arm) {
  const { points, forward, up } = forwardKinematics(q, arm);
  const basis = toolBasis(forward, up);
  return { tip: toPhysics(points.at(-1)), quaternion: basisQuaternion(basis), basis };
}

const vec = (a) => new CANNON.Vec3(a[0], a[1], a[2]);
const dotArr = (v, a) => v.x * a[0] + v.y * a[1] + v.z * a[2];

/** Half-extent of a body's collider along world direction `dir` (unit array), metres. */
function supportHalf(body, dir) {
  const shape = body.shapes[0];
  if (shape instanceof CANNON.Sphere) return shape.radius;
  const h = shape.halfExtents;
  const axes = [new CANNON.Vec3(1, 0, 0), new CANNON.Vec3(0, 1, 0), new CANNON.Vec3(0, 0, 1)].map((axis) => body.quaternion.vmult(axis));
  return Math.abs(dotArr(axes[0], dir)) * h.x + Math.abs(dotArr(axes[1], dir)) * h.y + Math.abs(dotArr(axes[2], dir)) * h.z;
}

/**
 * A rigid-body scene built from a task's `rigid` block:
 *
 *   objects:  [{ id, shape: 'box' | 'sphere', size (mm edge or diameter),
 *                position: [x, y] scene px on the table, yaw (rad), mass (kg), color }]
 *   fixtures: [{ id, type: 'tray', position: [x, y], inner: [w, d], wall, height }]
 */
export class RigidScene {
  constructor(spec = {}, { arms = 2 } = {}) {
    this.spec = spec;
    this.world = new CANNON.World({ gravity: new CANNON.Vec3(0, -9.81, 0), allowSleep: false });
    this.world.solver.iterations = 20;
    this.world.defaultContactMaterial.friction = 0.6;
    this.world.defaultContactMaterial.restitution = 0.05;
    this.world.defaultContactMaterial.contactEquationStiffness = 1e7;
    this.world.defaultContactMaterial.contactEquationRelaxation = 3;

    const table = new CANNON.Body({ type: CANNON.Body.STATIC, shape: new CANNON.Plane() });
    table.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    this.world.addBody(table);

    this.fixtures = [];
    for (const fixture of spec.fixtures || []) {
      if (fixture.type !== 'tray') continue;
      const [cx, cy] = fixture.position;
      const [w, d] = fixture.inner;
      const t = fixture.wall ?? 4;
      const hgt = fixture.height ?? 18;
      // Four walls around the inner footprint; the table is the tray floor.
      const walls = [
        { at: [cx, cy - d / 2 - t / 2], half: [w / 2 + t, t / 2] },
        { at: [cx, cy + d / 2 + t / 2], half: [w / 2 + t, t / 2] },
        { at: [cx - w / 2 - t / 2, cy], half: [t / 2, d / 2] },
        { at: [cx + w / 2 + t / 2, cy], half: [t / 2, d / 2] },
      ];
      for (const wall of walls) {
        const body = new CANNON.Body({ type: CANNON.Body.STATIC, shape: new CANNON.Box(new CANNON.Vec3(wall.half[0] * MM, hgt / 2 * MM, wall.half[1] * MM)) });
        body.position.copy(toPhysics([...wall.at, hgt / 2]));
        this.world.addBody(body);
        this.fixtures.push({ id: fixture.id, body, size: [wall.half[0] * 2, wall.half[1] * 2, hgt] });
      }
    }

    this.objects = (spec.objects || []).map((object) => {
      const size = object.size * MM;
      const shape = object.shape === 'sphere' ? new CANNON.Sphere(size / 2) : new CANNON.Box(new CANNON.Vec3(size / 2, size / 2, size / 2));
      const body = new CANNON.Body({ mass: object.mass ?? 0.05, shape });
      // Stand-in for rolling resistance, which cannon does not model: without
      // it a ball set down with any residual spin rolls off the table.
      body.angularDamping = object.shape === 'sphere' ? 0.4 : 0.1;
      body.linearDamping = 0.05;
      this.world.addBody(body);
      return { ...object, body };
    });

    this.grippers = Array.from({ length: arms }, () => {
      const fingers = [-1, 1].map((side) => {
        const body = new CANNON.Body({
          type: CANNON.Body.KINEMATIC,
          mass: 0,
          shape: new CANNON.Box(new CANNON.Vec3(FINGER.length / 2 * MM, FINGER.width / 2 * MM, FINGER.thickness / 2 * MM)),
        });
        body.position.set(0, -10, 0); // parked below the table until an arm drives it
        this.world.addBody(body);
        return { side, body };
      });
      return { fingers, closed: false, halfGap: FINGER.openHalfGap, held: null, pose: null };
    });
    this.reset();
  }

  /** Put every object back at its declared start pose, at rest, and open the jaws. */
  reset() {
    for (const object of this.objects) {
      const { body } = object;
      body.type = CANNON.Body.DYNAMIC;
      body.updateMassProperties();
      const half = object.size / 2;
      body.position.copy(toPhysics([...object.position, half]));
      body.quaternion.setFromEuler(0, -(object.yaw || 0), 0);
      body.velocity.setZero();
      body.angularVelocity.setZero();
      body.force.setZero();
      body.torque.setZero();
    }
    for (const gripper of this.grippers) {
      gripper.closed = false;
      gripper.halfGap = FINGER.openHalfGap;
      gripper.held = null;
      gripper.pose = null;
      for (const { body } of gripper.fingers) { body.position.set(0, -10, 0); body.velocity.setZero(); }
    }
    this.time = 0;
  }

  object(id) { return this.objects.find((object) => object.id === id); }

  /** Scene-space center (px) of an object. */
  objectCenter(id) { return toScene(this.object(id).body.position); }

  /**
   * Try to take hold of an object between gripper `index`'s jaws. The object
   * qualifies only if it physically sits in the jaw volume: centred between
   * the fingers closely enough to fit inside the open gap, overlapping the
   * finger slabs along the tool and across it. The closest such object is
   * held, pulled to the jaw centre (a parallel gripper centres what it
   * closes on), and the jaws stop at its surface.
   */
  grasp(index) {
    const gripper = this.grippers[index];
    const { tip, quaternion, basis } = gripper.pose;
    let best = null;
    for (const object of this.objects) {
      const { body } = object;
      const d = body.position.vsub(tip);
      const along = dotArr(d, basis.forward);
      const across = dotArr(d, basis.lateral);
      const side = dotArr(d, basis.normal);
      const halfAcross = supportHalf(body, basis.lateral);
      const halfAlong = supportHalf(body, basis.forward);
      const halfSide = supportHalf(body, basis.normal);
      const innerGap = (FINGER.openHalfGap - FINGER.thickness / 2) * MM;
      const fingerSpan = [-FINGER.length * MM, 0];
      const fits = Math.abs(across) + halfAcross <= innerGap + 1e-3;
      const overlapsAlong = along + halfAlong >= fingerSpan[0] && along - halfAlong <= fingerSpan[1];
      const overlapsSide = Math.abs(side) <= halfSide + FINGER.width / 2 * MM;
      if (!fits || !overlapsAlong || !overlapsSide) continue;
      const score = Math.abs(across) + Math.abs(side);
      if (!best || score < best.score) best = { object, along, side, halfAcross, score };
    }
    if (!best) {
      gripper.halfGap = FINGER.minHalfGap;
      return null;
    }
    const { object, along, side, halfAcross } = best;
    const { body } = object;
    body.type = CANNON.Body.KINEMATIC;
    body.velocity.setZero();
    body.angularVelocity.setZero();
    gripper.held = {
      id: object.id,
      // Offset in the tool frame (forward, normal, lateral), lateral zeroed.
      offset: [along, side, 0],
      rotation: quaternion.conjugate().mult(body.quaternion),
    };
    gripper.halfGap = halfAcross / MM + FINGER.thickness / 2;
    return object.id;
  }

  release(index) {
    const gripper = this.grippers[index];
    gripper.halfGap = FINGER.openHalfGap;
    if (!gripper.held) return;
    const { body } = this.object(gripper.held.id);
    body.type = CANNON.Body.DYNAMIC;
    body.updateMassProperties();
    body.angularVelocity.setZero();
    gripper.held = null;
  }

  /** Place gripper `index`'s fingers (and anything held) for tool pose `pose`, with velocity over `dt`. */
  #placeGripper(index, pose, dt) {
    const gripper = this.grippers[index];
    const { tip, quaternion, basis } = pose;
    const f = vec(basis.forward);
    const l = vec(basis.lateral);
    for (const { side, body } of gripper.fingers) {
      const target = tip.vsub(f.scale(FINGER.length / 2 * MM)).vadd(l.scale(side * gripper.halfGap * MM));
      body.velocity.copy(target.vsub(body.position).scale(1 / dt));
      body.quaternion.copy(quaternion);
    }
    if (gripper.held) {
      const { body } = this.object(gripper.held.id);
      const [a, s, c] = gripper.held.offset;
      const target = tip.vadd(f.scale(a)).vadd(vec(basis.normal).scale(s)).vadd(l.scale(c));
      body.velocity.copy(target.vsub(body.position).scale(1 / dt));
      body.quaternion.copy(quaternion.mult(gripper.held.rotation));
    }
  }

  /** Teleport fingers and held objects onto the current tool poses, with zero velocity. */
  #snapGrippers() {
    this.grippers.forEach((gripper, index) => {
      if (!gripper.pose) return;
      this.#placeGripper(index, gripper.pose, 1);
      for (const { body } of gripper.fingers) { body.position.vadd(body.velocity, body.position); body.velocity.setZero(); }
      if (gripper.held) {
        const { body } = this.object(gripper.held.id);
        body.position.vadd(body.velocity, body.position);
        body.velocity.setZero();
      }
    });
  }

  /**
   * Advance one control frame. `arms` is [{ q, arm }] for each active arm,
   * `grips` the gripper command per arm (true = closed) or undefined to leave
   * the jaws as they are. A close command on open jaws attempts a grasp; an
   * open command releases. Kinematic bodies are swept between the previous
   * and new tool pose over the frame so contacts see real velocities.
   */
  step({ arms = [], grips } = {}, dt = CONTROL_DT) {
    const substeps = Math.max(1, Math.round(dt / PHYSICS_DT));
    const h = dt / substeps;
    const poses = arms.map(({ q, arm }) => toolPose(q, arm));
    poses.forEach((pose, index) => {
      const gripper = this.grippers[index];
      if (!gripper.pose) { gripper.pose = pose; this.#snapGrippers(); }
    });
    // Gripper commands act at the start of the frame, at the current pose.
    poses.forEach((_, index) => {
      const gripper = this.grippers[index];
      const command = grips?.[index];
      if (command === undefined || command === gripper.closed) return;
      gripper.closed = command;
      if (command) this.grasp(index); else this.release(index);
      this.#snapGrippers();
    });
    for (let s = 1; s <= substeps; s += 1) {
      const t = s / substeps;
      poses.forEach((pose, index) => {
        const from = this.grippers[index].pose;
        const tip = from.tip.vadd(pose.tip.vsub(from.tip).scale(t));
        const quaternion = new CANNON.Quaternion();
        from.quaternion.slerp(pose.quaternion, t, quaternion);
        // The basis is only used for directions; the end pose's is close
        // enough within one rate-capped frame.
        this.#placeGripper(index, { tip, quaternion, basis: pose.basis }, h);
      });
      this.world.step(h);
    }
    poses.forEach((pose, index) => { this.grippers[index].pose = pose; });
    this.time += dt;
  }

  /** Whether every object is essentially at rest (m/s and rad/s). */
  atRest(linear = 0.01, angular = 0.2) {
    return this.objects.every(({ body }) => body.velocity.length() < linear && body.angularVelocity.length() < angular);
  }

  /** Serializable state for the timeline and the renderers. Positions are scene px. */
  snapshot() {
    return {
      time: this.time,
      objects: this.objects.map(({ id, body }) => ({
        id,
        type: body.type,
        position: [body.position.x, body.position.y, body.position.z],
        quaternion: [body.quaternion.x, body.quaternion.y, body.quaternion.z, body.quaternion.w],
        velocity: [body.velocity.x, body.velocity.y, body.velocity.z],
        angularVelocity: [body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z],
      })),
      grippers: this.grippers.map((gripper) => ({
        closed: gripper.closed,
        halfGap: gripper.halfGap,
        held: gripper.held ? { id: gripper.held.id, offset: [...gripper.held.offset], rotation: [gripper.held.rotation.x, gripper.held.rotation.y, gripper.held.rotation.z, gripper.held.rotation.w] } : null,
        pose: gripper.pose ? { tip: [gripper.pose.tip.x, gripper.pose.tip.y, gripper.pose.tip.z], quaternion: [gripper.pose.quaternion.x, gripper.pose.quaternion.y, gripper.pose.quaternion.z, gripper.pose.quaternion.w], basis: gripper.pose.basis } : null,
      })),
    };
  }

  restore(snapshot) {
    if (!snapshot) return;
    this.time = snapshot.time;
    snapshot.objects.forEach((saved) => {
      const object = this.object(saved.id);
      if (!object) return;
      const { body } = object;
      body.type = saved.type;
      if (saved.type === CANNON.Body.DYNAMIC) body.updateMassProperties();
      body.position.set(...saved.position);
      body.quaternion.set(...saved.quaternion);
      body.velocity.set(...saved.velocity);
      body.angularVelocity.set(...saved.angularVelocity);
    });
    snapshot.grippers.forEach((saved, index) => {
      const gripper = this.grippers[index];
      if (!gripper) return;
      gripper.closed = saved.closed;
      gripper.halfGap = saved.halfGap;
      gripper.held = saved.held ? { id: saved.held.id, offset: [...saved.held.offset], rotation: new CANNON.Quaternion(...saved.held.rotation) } : null;
      gripper.pose = saved.pose ? { tip: vec(saved.pose.tip), quaternion: new CANNON.Quaternion(...saved.pose.quaternion), basis: saved.pose.basis } : null;
    });
    this.#snapGrippers();
  }
}

/** A snapshot's object as scene-space render data: center (px) and world quaternion. */
export function snapshotObject(snapshot, id) {
  const saved = snapshot?.objects.find((object) => object.id === id);
  if (!saved) return null;
  const [x, y, z] = saved.position;
  return { center: [x / MM, z / MM, y / MM], quaternion: saved.quaternion };
}

/** Yaw (rad, scene frame) of a world quaternion, for the top-down drawing. */
export function sceneYaw([x, y, z, w]) {
  // Local +x in world axes, then projected onto the table (world x, z) = scene (x, y).
  const ax = 1 - 2 * (y * y + z * z);
  const az = 2 * (x * z - w * y);
  return Math.atan2(az, ax);
}
