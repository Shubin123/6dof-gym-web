/**
 * Closed-loop stacking for Task 16 (Stack under fire).
 *
 * Every other task runs one plan to its end. Here the table changes under
 * the arm - the player is shooting balls at it - so the arm re-reads the
 * physics state and re-decides, the way a real cell has to:
 *
 *  - pick the nearest resting, reachable cube that is not already in the
 *    tower, and plan a tool-down pick and place onto the tower's top;
 *  - before the grasp, abandon the plan if that cube has been knocked away;
 *  - right after closing, confirm the gripper actually holds it;
 *  - while carrying, notice a hit that knocked the cube out of the jaws;
 *  - before lowering, re-read the tower and re-aim at its real height -
 *    shot down to fewer cubes, or still the height the plan assumed;
 *  - after placing, go back to the start and do it all again, forever,
 *    rebuilding whenever the tower is knocked down.
 *
 * Every frame it hands back still goes through main.js's per-frame joint-
 * rate and cell-safety check; this module only decides what to try next.
 */
import { forwardKinematics, HOME_POSE, planSafeMotion } from './core.js';
import { planPickPlaceMotion, planToolDownMotion, toolDownError, TOOL_DOWN_TOLERANCE } from './rigid-plan.js';
import { GRASP_HEIGHT, towerStack } from './rigid-tasks.js';
import { insideFence } from './rigid.js';

const RELEASE_GAP = 2;
/** A cube that moves this far (px) from where its grasp was planned is re-planned for. */
const MOVED_PX = 6;
/** Cubes slower than this (m/s) count as resting. */
const REST_SPEED = 0.03;
/** Control frames every cube must stay resting before the arm decides its next move. */
const SETTLE_FRAMES = 6;
const DWELL = 4;

const tipOf = (q, arm) => forwardKinematics(q, arm).points.at(-1);
const flat = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

export class StackController {
  constructor({ rigid, arm, safety }) {
    this.rigid = rigid;
    this.arm = arm;
    this.safety = safety;
    this.reset();
  }

  reset() {
    this.queue = []; // [{ q, grip, check? }] frames still to send
    this.task = null; // the cube being moved and where it goes
    this.grip = false;
    this.cooldown = 0;
    this.stillFor = 0;
    this.unreachable = new Map(); // cube id -> rounded position it was unreachable at
    this.stats = { placed: 0, best: 0, recoveries: 0, drops: 0, misses: 0 };
    this.phase = 'starting';
  }

  /** Drop whatever plan is in flight (the arm may have been moved by hand); keep the stats. */
  interrupt() {
    this.queue = [];
    this.task = null;
    this.cooldown = 0;
  }

  /** Where the tool tip releases a cube onto a tower `level` cubes tall. */
  placePoint(level) {
    const [x, y] = this.rigid.goal.position;
    return [x, y, level * 30 + RELEASE_GAP + GRASP_HEIGHT];
  }

  hoverFor(placeZ) { return Math.max(this.rigid.hover_z ?? 90, placeZ + 45); }

  /** The next control frame: { q, grip } (q unchanged to hold still). */
  next(scene, q) {
    const knocked = scene.takeKnock() === 0;
    const towerIds = towerStack(this.rigid, scene);
    const held = scene.grippers[0].held?.id;
    const still = scene.objects.every((object) => object.shape !== 'box' || object.id === held || object.body.velocity.length() < REST_SPEED);
    this.stillFor = still ? this.stillFor + 1 : 0;
    this.stats.best = Math.max(this.stats.best, towerIds.length);
    // Which cubes are reachable depends on the height they go to.
    if (towerIds.length !== this.towerHeight) this.unreachable.clear();
    this.towerHeight = towerIds.length;

    if (knocked && this.task?.stage === 'carry') {
      this.stats.drops += 1;
      this.#recover(q, 'hit - cube knocked out of the gripper');
    }
    if (this.queue.length) this.#monitor(scene, q, towerIds);
    if (!this.queue.length) this.#decide(scene, q, towerIds);

    const frame = this.queue.shift();
    if (!frame) return { q, grip: this.grip };
    this.grip = frame.grip;
    frame.onSend?.();
    return { q: frame.q, grip: frame.grip };
  }

  /** Checks that run before a frame is sent, at the points where the world may have changed under the plan. */
  #monitor(scene, q, towerIds) {
    const { task } = this;
    if (!task) return;
    const upcoming = this.queue[0];
    if (task.stage === 'approach') {
      const moved = flat(scene.objectCenter(task.id), task.pickAt);
      const heightMoved = Math.abs(scene.objectCenter(task.id)[2] - task.pickAt[2]);
      if (moved > MOVED_PX || heightMoved > MOVED_PX) this.#recover(q, 'target cube moved - re-planning');
      return;
    }
    if (upcoming?.mark === 'grasped') {
      if (scene.grippers[0].held?.id !== task.id) {
        this.stats.misses += 1;
        this.#recover(q, 'grasp missed - re-planning');
      }
      return;
    }
    if (upcoming?.mark === 'lower' && task.level !== towerIds.length) {
      // The tower changed while the cube was on its way: aim at its real top.
      this.#replanPlacement(q, towerIds.length);
    }
  }

  #decide(scene, q, towerIds) {
    this.task = null;
    if (this.cooldown > 0) { this.cooldown -= 1; return; }
    const { goal } = this.rigid;
    // Decide only on a settled table: a cube just released, or one still
    // tumbling from a hit, is neither a safe target nor a finished level.
    if (this.stillFor < SETTLE_FRAMES) {
      this.phase = 'waiting for the table to settle';
      return;
    }
    if (towerIds.length >= goal.height) {
      this.phase = `tower complete (${towerIds.length} high) - shoot it down`;
      return;
    }
    const inTower = new Set(towerIds);
    // Only cubes inside the pen are in play (a stray is being returned).
    const candidates = scene.objects
      .filter((object) => object.shape === 'box' && !inTower.has(object.id) && insideFence(this.rigid.fence, scene.objectCenter(object.id)))
      .map((object) => ({ object, center: scene.objectCenter(object.id) }))
      .filter(({ object, center }) => this.unreachable.get(object.id) !== key(center))
      .sort((a, b) => flat(a.center, goal.position) - flat(b.center, goal.position));
    for (const { object, center } of candidates) {
      const pick = [center[0], center[1], center[2] - object.size / 2 + GRASP_HEIGHT];
      const place = this.placePoint(towerIds.length);
      // A cube sitting in the tower's footprint (rubble) is lifted clear and
      // put back as the next level; one right on the axis already is that level.
      const plan = planPickPlaceMotion({ q, arm: this.arm }, { pick, place, hoverZ: this.hoverFor(place[2]), settleFrames: 8, dwellFrames: DWELL }, this.safety);
      if (!plan) { this.unreachable.set(object.id, key(center)); continue; }
      this.#load(plan, { id: object.id, pickAt: center, level: towerIds.length });
      this.phase = `stacking level ${towerIds.length + 1} of ${goal.height}`;
      return;
    }
    this.phase = 'no reachable cube - waiting';
    this.cooldown = 12;
  }

  /** Queue a pick-and-place plan, tagging the frames the monitor cares about. */
  #load(plan, task) {
    this.task = { ...task, stage: 'approach' };
    const closeAt = plan.grips.findIndex(([closed]) => closed);
    const [, graspEnd, carryEnd] = plan.stageEnds;
    const openAt = plan.grips.findIndex(([closed], index) => index > closeAt && !closed);
    this.queue = plan.frames.map(([frameQ], index) => {
      const frame = { q: frameQ, grip: plan.grips[index][0] };
      if (index === closeAt) frame.onSend = () => { this.task.stage = 'grasp'; };
      if (index === graspEnd + 1) { frame.mark = 'grasped'; frame.onSend = () => { this.task.stage = 'carry'; }; }
      if (index === carryEnd + 1) frame.mark = 'lower';
      if (index === openAt) frame.onSend = () => { this.task.stage = 'placed'; this.stats.placed += 1; };
      return frame;
    });
  }

  /** Holding a cube above the tower: lower onto its real top instead, then release and go home. */
  #replanPlacement(q, level) {
    const place = this.placePoint(level);
    const [x, y] = place;
    const hover = this.hoverFor(place[2]);
    const tip = tipOf(q, this.arm);
    const lower = planToolDownMotion(q, [[tip[0], tip[1], Math.max(hover, tip[2])], [x, y, Math.max(hover, tip[2])], place], this.arm, this.safety);
    if (!lower) { this.#recover(q, 'tower changed - cannot reach its new top'); return; }
    const end = lower.at(-1);
    const rise = planToolDownMotion(end, [[x, y, hover]], this.arm, this.safety) || [];
    const home = planSafeMotion(rise.at(-1) ?? end, [...HOME_POSE], this.arm, this.safety) || [];
    this.task = { ...this.task, level };
    this.stats.recoveries += 1;
    this.phase = `tower changed - re-aiming at level ${level + 1}`;
    const frames = [
      ...lower.map((frameQ) => ({ q: frameQ, grip: true })),
      ...Array(DWELL).fill({ q: end, grip: true }),
      { q: end, grip: false, onSend: () => { this.task.stage = 'placed'; this.stats.placed += 1; } },
      ...Array(DWELL).fill({ q: end, grip: false }),
      ...rise.map((frameQ) => ({ q: frameQ, grip: false })),
      ...home.map((frameQ) => ({ q: frameQ, grip: false })),
    ];
    this.queue = frames;
  }

  /**
   * Abandon the current plan: open, back straight up out of the objects if
   * the tool is down among them, and let the next frame decide afresh from
   * the scene as it now is.
   */
  #recover(q, reason) {
    this.stats.recoveries += 1;
    this.phase = reason;
    const frames = [{ q, grip: false }];
    const tip = tipOf(q, this.arm);
    const err = toolDownError(q, this.arm, tip);
    if (tip[2] < 85 && err.tilt < TOOL_DOWN_TOLERANCE) {
      const rise = planToolDownMotion(q, [[tip[0], tip[1], 90]], this.arm, this.safety);
      if (rise) frames.push(...rise.map((frameQ) => ({ q: frameQ, grip: false })));
    }
    this.task = null;
    this.queue = frames;
    this.cooldown = 0;
  }

  status() {
    return { phase: this.phase, tower: this.towerHeight ?? 0, goal: this.rigid.goal.height, ...this.stats };
  }
}

const key = (center) => center.map((value) => Math.round(value / 4)).join(',');
