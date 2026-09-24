/**
 * Declarative task-policy registry.
 *
 * This is the browser's pseudo-training seam: a task supplies a small,
 * inspectable recipe and a deterministic warm-start derives its policy
 * parameters from the task contract. It is intentionally not presented as a
 * trained production model. Real policies can replace `bootstrapPolicy` with
 * a checkpoint loader while retaining the same safety-gated interface.
 */
export const TASK_POLICY_RECIPES = Object.freeze({
  default: Object.freeze({
    label: 'Geometric policy',
    kind: 'safe-IK',
    training: 'Safe geometric warm-start',
  }),
  fold: Object.freeze({
    label: 'Cloth-aware warm-start',
    kind: 'cloth-fold',
    training: 'Synthetic cloth calibration',
    profile: Object.freeze({ liftHeight: 45, crossHeight: 38, placeHeight: 6, settleFrames: 50 }),
    stages: Object.freeze([
      Object.freeze({ id: 'contact', label: 'secure both corners', weight: 0.18 }),
      Object.freeze({ id: 'pin', label: 'pin and release left edge', weight: 0.18 }),
      Object.freeze({ id: 'cross', label: 'lift and cross fold line', weight: 0.24 }),
      Object.freeze({ id: 'place', label: 'place on folded target', weight: 0.28 }),
      Object.freeze({ id: 'settle', label: 'settle without overstretch', weight: 0.12 }),
    ]),
  }),
  // Same recipe as `fold`: the grasp vertices vary (see main.js's fold-vertex
  // selection), but the stage rewards read cloth.getFoldMetrics() and the
  // capture flags, neither of which cares which vertex was actually held.
  fold_custom: Object.freeze({
    label: 'Cloth-aware warm-start (configurable grasp)',
    kind: 'cloth-fold',
    training: 'Synthetic cloth calibration',
    profile: Object.freeze({ liftHeight: 45, crossHeight: 38, placeHeight: 6, settleFrames: 50 }),
    stages: Object.freeze([
      Object.freeze({ id: 'contact', label: 'secure both corners', weight: 0.18 }),
      Object.freeze({ id: 'pin', label: 'pin and release left edge', weight: 0.18 }),
      Object.freeze({ id: 'cross', label: 'lift and cross fold line', weight: 0.24 }),
      Object.freeze({ id: 'place', label: 'place on folded target', weight: 0.28 }),
      Object.freeze({ id: 'settle', label: 'settle without overstretch', weight: 0.12 }),
    ]),
  }),
});

export function policyRecipeFor(task) {
  return TASK_POLICY_RECIPES[task?.id] || TASK_POLICY_RECIPES.default;
}

/**
 * Produce a policy configuration from task metadata without bypassing safety.
 * This is repeatable pseudo-training: a fresh task can be added by declaring
 * one recipe, and every returned trajectory still goes through IK, per-frame
 * collision checking, and the cloth score after execution.
 */
export function bootstrapPolicy(task) {
  const recipe = policyRecipeFor(task);
  return {
    ...recipe,
    profile: recipe.profile ? { ...recipe.profile } : null,
    provenance: `browser ${recipe.training.toLowerCase()}`,
  };
}

/**
 * Dense, observable rewards for the fold. They make optimisation meaningful
 * before the final folded state and keep the policy/task boundary declarative.
 */
export function scoreTaskStages(task, { cloth, tips = [] } = {}) {
  const recipe = policyRecipeFor(task);
  if (recipe.kind !== 'cloth-fold' || !cloth) return { reward: 0, stage: 'route', complete: false };
  const metrics = cloth.getFoldMetrics();
  const rightCrossed = (tips[1]?.[0] ?? Infinity) < 325;
  const scores = {
    contact: cloth.wasCaptured?.[0] && cloth.wasCaptured?.[1] ? 1 : 0,
    pin: cloth.released?.[0] && cloth.tablePinA ? 1 : 0,
    cross: rightCrossed ? 1 : 0,
    place: metrics.frontDistance < 0.42 ? 1 : Math.max(0, 1 - metrics.frontDistance / 1.5),
    settle: metrics.stretchError < 0.25 ? 1 : Math.max(0, 1 - metrics.stretchError),
  };
  const stage = recipe.stages.find((entry) => scores[entry.id] < 0.999) || recipe.stages.at(-1);
  const reward = recipe.stages.reduce((sum, entry) => sum + entry.weight * scores[entry.id], 0);
  return { reward, stage: stage.label, complete: metrics.folded && metrics.stretchError < 0.25 && scores.place >= 0.999 };
}
