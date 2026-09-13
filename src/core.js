export const ARM = Object.freeze({
  base: [200, 345],
  lengths: [120, 105, 90, 80, 70, 60],
  jointLimit: 1.7,
  maxActionDelta: 0.05,
});
export const MAX_EPISODE_TRANSITIONS = 200;

export const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

export function forwardKinematics(q, arm = ARM) {
  let [x, y] = arm.base;
  let angle = -Math.PI / 2;
  const points = [[x, y]];
  for (let i = 0; i < arm.lengths.length; i += 1) {
    angle += q[i];
    x += Math.cos(angle) * arm.lengths[i];
    y += Math.sin(angle) * arm.lengths[i];
    points.push([x, y]);
  }
  return { points, angle };
}

export const distance = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
const angleDelta = (to, from) => Math.atan2(Math.sin(to - from), Math.cos(to - from));

/**
 * Find a bounded joint-space target with deterministic multi-start CCD.
 *
 * A single CCD run can settle into a joint-limit local minimum after an
 * operator has moved the sliders. Trying a small, fixed set of postures makes
 * guidance reproducible and lets the browser choose the closest valid plan.
 */
export function solveInverseKinematics(target, arm = ARM) {
  const seeds = [
    [0, 0, 0, 0, 0, 0],
    [0.6, -0.6, 0.6, -0.6, 0.4, -0.4],
    [-0.6, 0.6, -0.6, 0.6, -0.4, 0.4],
    [1, -0.5, 0.8, -0.5, 0.6, -0.4],
    [-1, 0.5, -0.8, 0.5, -0.6, 0.4],
    [0.2, 0.5, -0.5, 0.5, -0.5, 0.3],
    [-0.2, -0.5, 0.5, -0.5, 0.5, -0.3],
  ];
  let best = null;
  for (const seed of seeds) {
    const q = seed.map((value) => clamp(value, -arm.jointLimit, arm.jointLimit));
    for (let iteration = 0; iteration < 250; iteration += 1) {
      for (let joint = q.length - 1; joint >= 0; joint -= 1) {
        const { points } = forwardKinematics(q, arm);
        const pivot = points[joint];
        const endEffector = points.at(-1);
        const targetAngle = Math.atan2(target[1] - pivot[1], target[0] - pivot[0]);
        const effectorAngle = Math.atan2(endEffector[1] - pivot[1], endEffector[0] - pivot[0]);
        q[joint] = clamp(q[joint] + clamp(angleDelta(targetAngle, effectorAngle), -0.2, 0.2), -arm.jointLimit, arm.jointLimit);
      }
      if (distance(forwardKinematics(q, arm).points.at(-1), target) < 1) break;
    }
    const candidate = { q, distance: distance(forwardKinematics(q, arm).points.at(-1), target) };
    if (!best || candidate.distance < best.distance) best = candidate;
  }
  return best;
}

/** One safety-capped tracking update toward an already validated IK plan. */
export function guidedStep(q, target, arm = ARM, plan = solveInverseKinematics(target, arm)) {
  const next = q.map((value, index) => clamp(
    value + clamp(plan.q[index] - value, -arm.maxActionDelta, arm.maxActionDelta),
    -arm.jointLimit,
    arm.jointLimit,
  ));
  const action = next.map((value, index) => value - q[index]);
  return { q: next, action, distance: distance(forwardKinematics(next, arm).points.at(-1), target) };
}

export function projectToReachableWorkspace(point, workspace, arm = ARM) {
  const [minX, maxX, minY, maxY] = workspace.goal_workspace;
  const candidate = [clamp(point[0], minX, maxX), clamp(point[1], minY, maxY)];
  const maxRadius = workspace.max_reach_px;
  const fromBase = [candidate[0] - arm.base[0], candidate[1] - arm.base[1]];
  const radius = Math.hypot(...fromBase);
  if (radius <= maxRadius) return candidate;
  return [arm.base[0] + fromBase[0] / radius * maxRadius, arm.base[1] + fromBase[1] / radius * maxRadius];
}

/** Portable browser episode envelope; voice is optional and self-contained. */
export function buildEpisodeArtifact({ environment, task, transitions, voice }) {
  return {
    schema: 'armlab-episode-preview/v0.2',
    environment,
    task,
    source: 'browser-simulation',
    transitions,
    voice: {
      transcript: voice?.transcript || '',
      mime_type: voice?.mimeType || null,
      audio_data_url: voice?.dataUrl || null,
    },
  };
}
