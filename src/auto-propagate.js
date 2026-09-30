/**
 * Task 18: Auto-propagate Arm.
 *
 * Mechanical robotic arm installation: an operating 6-DOF arm retrieves
 * modular sub-assemblies from a parts depot (Shoulder Turret, Articulated Arm
 * Boom, and Wrist/Gripper Toolhead) and mechanically installs a full 6-DOF
 * sibling robotic arm onto the user-positioned base podium.
 *
 * The podium is a real column (the same size as the primary arm's own base),
 * so the modules are seated on its top face and every frame of both arms is
 * checked against the task's physical constraints:
 *
 *   bend        no joint folds its links past `bend_limit_rad`
 *   self        no arm folds back into its own non-adjacent links
 *   podium      no link enters the sibling podium, the growing module stack
 *               on it, or the primary arm's pedestal
 *   arm-to-arm  once commissioned, the sibling's calibration sweep keeps
 *               `arm_clearance_px` from the parked primary arm
 */
import { ARM, clamp, evaluateCellSafety, HOME_POSE, planSafeMotion } from './core.js';
import { planPickPlaceMotion } from './rigid-plan.js';

export const PROPAGATE_STEP_BUDGET = 1200;

/** Bounding box of verified safe, collision-free deployment coordinates. */
export const PROPAGATE_BUILD_WORKSPACE = Object.freeze({
  minX: 430,
  maxX: 460,
  // The podium's 42 mm floor flange must clear the depot tray's front wall.
  minY: 250,
  maxY: 305,
});

/** Mounting podium: the same column the primary arm stands on. */
export const PODIUM = Object.freeze({ radius: 42, top_radius: 30, height: ARM.baseHeight });

/** Physical constraints the task enforces when its spec declares none. */
export const PROPAGATE_CONSTRAINTS = Object.freeze({
  bend_limit_rad: 1.65,
  self_clearance_px: 40,
  obstacle_clearance_px: 6,
});

/** The sibling arm: the primary's own manipulator, mirrored, bolted onto the podium. */
export const SIBLING_ARM = Object.freeze({ ...ARM, id: 'S', mirror: true });
export const siblingArmAt = (position) => ({ ...SIBLING_ARM, base: [position[0], position[1]] });

/**
 * Posture the sibling is assembled in: folded over its podium, clear of the
 * table edge and the primary arm from anywhere in the build workspace.
 */
export const SIBLING_PARK_POSE = Object.freeze([-1.62, 1.25, 1.42, -0.31, 1.31, -0.74]);
/** Articulated ready posture: raised, inside every bend and clearance limit at any podium site. */
export const SIBLING_ARM_READY_POSE = Object.freeze([0.59, 1.28, 0.3, 1.41, 1.02, 0.05]);
/** Power-on self-test waypoints: J1 azimuth checkout either side of park, then unpark into ready. */
export const SIBLING_CALIBRATION = Object.freeze([
  Object.freeze([-1.42, 1.25, 1.42, -0.31, 1.31, -0.74]),
  Object.freeze([-1.82, 1.25, 1.42, -0.31, 1.31, -0.74]),
  SIBLING_ARM_READY_POSE,
]);

/** Modular robot arm sub-assemblies staged in the depot, bottom of the stack first. */
export const PROPAGATE_MODULES = Object.freeze([
  // hoverZ carries each module's bottom face clear over the stack it joins.
  Object.freeze({ id: 'arm_base', name: 'Shoulder Turret Assembly', size: 30, hoverZ: 115 }),
  Object.freeze({ id: 'arm_link', name: 'Articulated Arm Boom', size: 26, hoverZ: 145 }),
  Object.freeze({ id: 'arm_tool', name: 'Wrist & Gripper Toolhead', size: 22, hoverZ: 172 }),
]);

/** Tip height above a module's bottom at the grasp, plus the drop gap on release. */
const GRASP_Z = 8;
const RELEASE_GAP = 2;

const flat = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

/** Height of the module stack's bottom face for each module, resting on the podium top. */
export function moduleSeatZ(index) {
  return PODIUM.height + PROPAGATE_MODULES.slice(0, index).reduce((sum, mod) => sum + mod.size, 0);
}

/**
 * The arm with its joint range narrowed to the bend limit. Every articulated
 * joint turns about an axis perpendicular to its link, so the angle between
 * consecutive links is exactly |q|: the solvers then search only bends the
 * safety check will accept.
 */
export function bendLimitedArm(arm, rigid) {
  const bend = rigid.constraints?.bend_limit_rad ?? PROPAGATE_CONSTRAINTS.bend_limit_rad;
  return { ...arm, jointLimit: Math.min(arm.jointLimit, bend) };
}

/**
 * The cell's safety block with this task's constraints layered on.
 * `stacked` modules already sit on the podium and extend it as an obstacle.
 */
export function propagateSafety(rigid, safety = {}, { stacked = 0 } = {}) {
  const constraints = { ...PROPAGATE_CONSTRAINTS, ...(rigid.constraints || {}) };
  const podium = rigid.goal.podium || PODIUM;
  const clearance = constraints.obstacle_clearance_px;
  const center = rigid.goal.position;
  const obstacles = [
    { id: 'pedestal', center: ARM.base, radius: PODIUM.radius, topRadius: PODIUM.top_radius, height: ARM.baseHeight, clearance },
    { id: 'podium', center, radius: podium.radius, topRadius: podium.top_radius, height: podium.height, clearance },
  ];
  if (stacked > 0) {
    // A module's footprint is its half-diagonal; the widest placed one bounds the stack.
    const radius = Math.max(...PROPAGATE_MODULES.slice(0, stacked).map((mod) => mod.size / Math.SQRT2));
    obstacles.push({ id: 'assembly', center, radius, height: moduleSeatZ(stacked), clearance });
  }
  return {
    ...safety,
    bend_limit_rad: constraints.bend_limit_rad,
    self_clearance_px: constraints.self_clearance_px,
    obstacles: [...(safety.obstacles || []), ...obstacles],
  };
}

/**
 * Plan the sibling's power-on calibration sweep, from its assembled park
 * pose through SIBLING_CALIBRATION, against the parked primary arm. Every
 * frame keeps both arms apart, off both columns, and inside the bend limit.
 */
export function planSiblingCommissioning(rigid, safety, primary) {
  const arm = bendLimitedArm(siblingArmAt(rigid.goal.position), rigid);
  const cellSafety = propagateSafety(rigid, safety, { stacked: 0 });
  let q = [...SIBLING_PARK_POSE];
  const frames = [q];
  for (const waypoint of SIBLING_CALIBRATION) {
    const leg = planSafeMotion(q, [...waypoint], arm, cellSafety, [primary]);
    if (!leg) return null;
    frames.push(...leg);
    q = leg.at(-1);
  }
  return { arm, frames };
}

/**
 * Plans the complete mechanical installation trajectory to install a
 * secondary robotic arm onto the base podium at user-defined coordinates.
 *
 *  1. Pick and dock Shoulder Turret onto the podium top flange
 *  2. Pick and couple Articulated Arm Boom into shoulder clevis
 *  3. Pick and lock Wrist & Gripper Toolhead onto forearm tool flange
 *  4. Primary holds at HOME while the sibling runs its calibration sweep
 */
export function planAutoPropagateTask(rigid, scene, pose, safety = {}) {
  const target = rigid.goal.position;
  const arm = bendLimitedArm(pose.arm || ARM, rigid);
  let curQ = [...pose.q];
  const frames = [];
  const grips = [];
  const stageEnds = [];

  for (const [index, mod] of PROPAGATE_MODULES.entries()) {
    const objCenter = scene.objectCenter(mod.id);
    const pick = [objCenter[0], objCenter[1], objCenter[2] - mod.size / 2 + GRASP_Z];
    const place = [target[0], target[1], moduleSeatZ(index) + RELEASE_GAP + GRASP_Z];
    const plan = planPickPlaceMotion(
      { q: curQ, arm },
      { pick, place, hoverZ: mod.hoverZ, settleFrames: 25 },
      propagateSafety(rigid, safety, { stacked: index }),
    );
    if (!plan) return null;

    for (let i = 0; i < plan.frames.length; i += 1) {
      frames.push(plan.frames[i]);
      grips.push(plan.grips[i]);
    }
    curQ = plan.frames.at(-1)[0];
    stageEnds.push(frames.length - 1);
  }

  const sibling = planSiblingCommissioning(rigid, safety, { q: curQ, arm });
  if (!sibling) return null;
  const startFrame = frames.length;
  for (let i = 0; i < sibling.frames.length; i += 1) {
    frames.push([[...curQ]]);
    grips.push([false]);
  }
  stageEnds.push(frames.length - 1);

  return {
    frames,
    grips,
    stageEnds,
    sourceFrames: frames.length,
    reducedBy: 0,
    sibling: { arm: sibling.arm, frames: sibling.frames, startFrame },
  };
}

/** The sibling's joint pose at plan frame `index` (park before the sweep, ready after it). */
export function siblingPoseAt(sibling, index) {
  if (!sibling) return [...SIBLING_ARM_READY_POSE];
  const at = clamp(index - sibling.startFrame, 0, sibling.frames.length - 1);
  return [...sibling.frames[at]];
}

/**
 * Live cell check for the telemetry: the primary pose against the task's
 * constraints and, once it is assembled, the sibling arm at `siblingQ`.
 */
export function evaluatePropagateCell(rigid, safety, primary, { siblingQ = null, stacked = 0 } = {}) {
  const poses = [primary];
  if (siblingQ) poses.push({ q: siblingQ, arm: siblingArmAt(rigid.goal.position) });
  return evaluateCellSafety(poses, propagateSafety(rigid, safety, { stacked: siblingQ ? 0 : stacked }));
}

/**
 * Scores the physical outcome of the self-replication assembly.
 */
export function autoPropagateOutcome(rigid, scene) {
  const { goal } = rigid;
  const target = goal.position;
  const tolerance = goal.tolerance ?? 15;

  const base = scene.object('arm_base');
  const link = scene.object('arm_link');
  const tool = scene.object('arm_tool');
  if (!base || !link || !tool) {
    return { success: false, placed: false, resting: false, held: false, error: 99 };
  }

  const centers = PROPAGATE_MODULES.map((mod) => scene.objectCenter(mod.id));
  const [dBase, dLink, dTool] = centers.map((center) => flat(center, target));
  // Each module is seated when it sits over the podium at its stack height.
  const seated = (index, slack) => Math.abs(centers[index][2] - (moduleSeatZ(index) + PROPAGATE_MODULES[index].size / 2)) < slack;

  const basePlaced = dBase <= tolerance && seated(0, 6);
  const linkPlaced = dLink <= tolerance + 4 && seated(1, 8);
  const toolPlaced = dTool <= tolerance + 6 && seated(2, 10);
  const placed = basePlaced && linkPlaced && toolPlaced;

  const held = scene.grippers.some((g) => g.held);
  const resting = !held && scene.atRest();
  const error = Math.max(dBase, dLink, dTool);

  return {
    success: placed && resting,
    placed,
    resting,
    held,
    error,
    modules: { base: basePlaced, link: linkPlaced, tool: toolPlaced },
  };
}

/**
 * Dense stage rewards for auto-propagation progress.
 */
export function scoreAutoPropagateStages(recipe, scene, rigid) {
  if (!scene || !rigid) {
    return { reward: 0, stage: recipe.stages[0].label, complete: false };
  }

  const outcome = autoPropagateOutcome(rigid, scene);
  const { modules, resting, placed } = outcome;

  const scores = {
    base: modules?.base ? 1 : 0,
    link: modules?.link ? 1 : 0,
    tool: modules?.tool ? 1 : 0,
    init: placed && resting ? 1 : 0,
  };

  const stage = recipe.stages.find((entry) => scores[entry.id] < 0.999) || recipe.stages.at(-1);
  const reward = Number(recipe.stages.reduce((sum, entry) => sum + entry.weight * scores[entry.id], 0).toFixed(4));

  return {
    reward,
    stage: stage.label,
    complete: outcome.success,
    outcome,
  };
}
