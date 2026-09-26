/**
 * Task 18: Auto-propagate Arm.
 *
 * Mechanical robotic arm installation: an operating 6-DOF arm retrieves
 * modular sub-assemblies from a parts depot (Shoulder Turret, Articulated Arm
 * Boom, and Wrist/Gripper Toolhead) and mechanically installs a full 6-DOF
 * sibling robotic arm onto the user-positioned base podium.
 */
import { ARM, clamp, HOME_POSE } from './core.js';
import { planPickPlaceMotion } from './rigid-plan.js';

export const PROPAGATE_STEP_BUDGET = 1200;

/** Bounding box of verified safe, collision-free deployment coordinates. */
export const PROPAGATE_BUILD_WORKSPACE = Object.freeze({
  minX: 390,
  maxX: 480,
  minY: 230,
  maxY: 310,
});

/** Articulated ready posture for the constructed full 6-DOF sibling arm. */
export const SIBLING_ARM_READY_POSE = Object.freeze([0.35, 0.95, 1.15, -0.95, 1.1, -0.6]);

/** Modular robot arm sub-assemblies staged in the depot. */
export const PROPAGATE_MODULES = Object.freeze([
  Object.freeze({ id: 'arm_base', name: 'Shoulder Turret Assembly', size: 30, placeZ: 10, hoverZ: 95 }),
  Object.freeze({ id: 'arm_link', name: 'Articulated Arm Boom', size: 26, placeZ: 40, hoverZ: 100 }),
  Object.freeze({ id: 'arm_tool', name: 'Wrist & Gripper Toolhead', size: 22, placeZ: 66, hoverZ: 105 }),
]);

const flat = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

/**
 * Plans the complete 3-stage mechanical installation trajectory to install
 * a secondary robotic arm onto the base podium at user-defined coordinates.
 *
 *  1. Pick and dock Shoulder Turret onto the base podium
 *  2. Pick and couple Articulated Arm Boom into shoulder clevis
 *  3. Pick and lock Wrist & Gripper Toolhead onto forearm tool flange
 *  4. Retract primary arm to HOME and let the new arm commission & settle
 */
export function planAutoPropagateTask(rigid, scene, pose, safety = {}) {
  const target = rigid.goal.position;
  const arm = pose.arm || ARM;
  let curQ = [...pose.q];
  const frames = [];
  const grips = [];
  const stageEnds = [];

  for (const mod of PROPAGATE_MODULES) {
    const objCenter = scene.objectCenter(mod.id);
    const pick = [objCenter[0], objCenter[1], 8];
    const place = [target[0], target[1], mod.placeZ];
    const plan = planPickPlaceMotion(
      { q: curQ, arm },
      { pick, place, hoverZ: mod.hoverZ, settleFrames: 25 },
      safety,
    );
    if (!plan) return null;

    for (let i = 0; i < plan.frames.length; i += 1) {
      frames.push(plan.frames[i]);
      grips.push(plan.grips[i]);
    }
    curQ = plan.frames.at(-1)[0];
    stageEnds.push(frames.length - 1);
  }

  return {
    frames,
    grips,
    stageEnds,
    sourceFrames: frames.length,
    reducedBy: 0,
  };
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

  const cBase = scene.objectCenter('arm_base');
  const cLink = scene.objectCenter('arm_link');
  const cTool = scene.objectCenter('arm_tool');

  const dBase = flat(cBase, target);
  const dLink = flat(cLink, target);
  const dTool = flat(cTool, target);

  const basePlaced = dBase <= tolerance && Math.abs(cBase[2] - 15) < 6;
  const linkPlaced = dLink <= tolerance + 4 && Math.abs(cLink[2] - 43) < 8;
  const toolPlaced = dTool <= tolerance + 6 && Math.abs(cTool[2] - 67) < 10;
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
