import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, evaluateCellSafety, forwardKinematics, HOME_POSE, planSafeMotion } from '../src/core.js';
import { derivativeProfile, KINEMATIC_WEIGHTS, LimbController, limbDragTarget, smoothJointPath } from '../src/limb-dynamics.js';
import { RigidScene } from '../src/rigid.js';
import { planAutoPropagateTask, propagateSafety } from '../src/auto-propagate.js';

const safety = compiled.environment.safety;
const task = compiled.workflows.find((w) => w.id === 'auto_propagate');
const maxStep = (frames) => Math.max(...frames.slice(1).map((q, i) => Math.max(...q.map((v, j) => Math.abs(v - frames[i][j])))));

/** A rate-capped joint path with a hard corner: yaw, then shoulder. */
function cornerPath() {
  const a = planSafeMotion([...HOME_POSE], [0.2, ...HOME_POSE.slice(1)], ARM, {});
  const b = planSafeMotion(a.at(-1), [0.2, 0.9, ...HOME_POSE.slice(2)], ARM, {});
  return [[...HOME_POSE], ...a, ...b];
}

test('weights cover jerk, snap, crackle and pop, falling with order', () => {
  assert.deepEqual(Object.keys(KINEMATIC_WEIGHTS), ['jerk', 'snap', 'crackle', 'pop']);
  const values = Object.values(KINEMATIC_WEIGHTS);
  values.slice(1).forEach((value, i) => assert.ok(value < values[i]));
});

test('derivativeProfile measures a known cubic exactly', () => {
  // q = t^3 / 6 has unit jerk and zero snap and above.
  const frames = Array.from({ length: 20 }, (_, t) => [t ** 3 / 6, 0]);
  const profile = derivativeProfile(frames);
  assert.ok(Math.abs(profile.jerk.peak - 1) < 1e-9);
  assert.ok(profile.snap.peak < 1e-9 && profile.pop.peak < 1e-9);
});

test('smoothJointPath lowers every weighted derivative, keeps endpoints, rate cap and limits', () => {
  const raw = cornerPath();
  const result = smoothJointPath(raw, { arm: ARM });
  assert.equal(result.smoothedSegments, 1);
  const before = derivativeProfile(raw);
  const after = derivativeProfile(result.frames);
  for (const name of Object.keys(KINEMATIC_WEIGHTS)) {
    assert.ok(after[name].peak < before[name].peak / 3, `${name} peak ${after[name].peak} vs ${before[name].peak}`);
  }
  assert.deepEqual(result.frames[0], raw[0]);
  result.frames.at(-1).forEach((value, joint) => assert.ok(Math.abs(value - raw.at(-1)[joint]) < 1e-9));
  assert.ok(maxStep(result.frames) <= ARM.maxActionDelta + 1e-9);
  assert.equal(result.source[0], 0);
  assert.equal(result.source.at(-1), raw.length - 1);
});

test('smoothJointPath pins anchors and keeps a segment its validator rejects', () => {
  const raw = cornerPath();
  const anchor = Math.floor(raw.length / 2);
  const pinned = smoothJointPath(raw, { arm: ARM, anchors: [anchor] });
  const at = pinned.source.indexOf(anchor);
  assert.ok(at > 0, 'anchor survives re-timing as an exact source frame');
  assert.deepEqual(pinned.frames[at], raw[anchor]);

  const refused = smoothJointPath(raw, { arm: ARM, validate: () => false });
  assert.equal(refused.smoothedSegments, 0);
  assert.deepEqual(refused.frames, raw);
});

test('LimbController eases in: continuous, rate-capped, and settles on target', () => {
  const controller = new LimbController(HOME_POSE, ARM);
  const target = [...HOME_POSE];
  target[1] -= 0.6;
  controller.setTarget(target);
  const frames = [[...HOME_POSE]];
  for (let tick = 0; tick < 400 && !controller.settled; tick += 1) frames.push(controller.step());
  assert.ok(controller.settled, 'reaches the target');
  assert.ok(maxStep(frames) <= ARM.maxActionDelta + 1e-9);
  // First steps are small: no velocity step at the start of motion.
  assert.ok(Math.abs(frames[1][1] - frames[0][1]) < 0.01, 'eases out of rest');
  // The same move as a bare rate-capped step, from rest.
  const naive = [[...HOME_POSE], [...HOME_POSE], [...HOME_POSE]];
  for (let t = 1; t <= 12; t += 1) naive.push(HOME_POSE.map((v, j) => (j === 1 ? v - 0.05 * t : v)));
  assert.ok(derivativeProfile(frames).jerk.peak < derivativeProfile(naive).jerk.peak / 4, 'far lower jerk than a rate-capped step');
});

test('LimbController stops against a constraint instead of passing it', () => {
  const cell = propagateSafety(task.rigid, safety);
  const controller = new LimbController(HOME_POSE, ARM);
  const target = [...HOME_POSE];
  target[2] = 1.7; // past joint 2's bend limit
  controller.setTarget(target);
  let blocked = false;
  for (let tick = 0; tick < 400; tick += 1) {
    const q = controller.step((pose) => evaluateCellSafety([{ q: pose, arm: ARM }], cell).safe);
    if (!q) { blocked = true; break; }
  }
  assert.ok(blocked, 'the bend limit halts the limb');
  assert.equal(evaluateCellSafety([{ q: controller.q, arm: ARM }], cell).safe, true, 'the arm is left on a safe pose');
  assert.ok(controller.q[2] <= cell.bend_limit_rad[1] + 1e-9);
});

test('limbDragTarget moves only the joints up to the grabbed limb', () => {
  const limb = 2;
  const before = forwardKinematics(HOME_POSE, ARM).points[limb + 1];
  const goal = [before[0] + 20, before[1] - 10, before[2] + 15];
  let q = [...HOME_POSE];
  for (let i = 0; i < 60; i += 1) q = limbDragTarget(q, ARM, limb, goal);
  q.slice(limb + 1).forEach((value, joint) => assert.equal(value, HOME_POSE[limb + 1 + joint], 'distal joints ride along'));
  const after = forwardKinematics(q, ARM).points[limb + 1];
  assert.ok(Math.hypot(...after.map((v, axis) => v - goal[axis])) < 1, 'grabbed limb end reaches the pointer');
});

test('Task 18 plan is derivative-smoothed and still fits its budget', () => {
  const scene = new RigidScene(task.rigid, { arms: 1 });
  const plan = planAutoPropagateTask(task.rigid, scene, { q: [...HOME_POSE], arm: ARM }, safety);
  assert.ok(plan.weights, 'plan records the derivative weights it used');
  const profile = derivativeProfile(plan.frames.map(([q]) => q));
  // Unsmoothed, this plan peaks at ~0.081 rad/step^3 jerk and ~0.58 pop.
  assert.ok(profile.jerk.peak < 0.05, `jerk peak ${profile.jerk.peak}`);
  assert.ok(profile.pop.peak < 0.25, `pop peak ${profile.pop.peak}`);
  const sibling = derivativeProfile(plan.sibling.frames);
  assert.ok(sibling.jerk.peak < 0.01, `sibling jerk peak ${sibling.jerk.peak}`);
});
