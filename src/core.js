export const ARM = Object.freeze({
  base: [200, 345],
  lengths: [120, 105, 90, 80, 70, 60],
  jointLimit: 1.7,
  maxActionDelta: 0.05,
});

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

/** One safety-capped cyclic-coordinate-descent update. */
export function guidedStep(q, target, arm = ARM, passes = 3) {
  const next = [...q];
  const previous = [...q];
  const perPassLimit = arm.maxActionDelta / passes;
  for (let pass = 0; pass < passes; pass += 1) {
    for (let joint = next.length - 1; joint >= 0; joint -= 1) {
      const { points } = forwardKinematics(next, arm);
      const pivot = points[joint];
      const endEffector = points.at(-1);
      const targetAngle = Math.atan2(target[1] - pivot[1], target[0] - pivot[0]);
      const effectorAngle = Math.atan2(endEffector[1] - pivot[1], endEffector[0] - pivot[0]);
      const requested = angleDelta(targetAngle, effectorAngle);
      const permitted = clamp(requested, -perPassLimit, perPassLimit);
      next[joint] = clamp(next[joint] + permitted, -arm.jointLimit, arm.jointLimit);
    }
  }
  const action = next.map((value, index) => value - previous[index]);
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
