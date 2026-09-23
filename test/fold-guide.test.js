import test from 'node:test';
import assert from 'node:assert/strict';
import compiled from '../data/compiled.json' with { type: 'json' };
import { ARM, ARM_B, HOME_POSE, forwardKinematics, planTowelFoldMotion } from '../src/core.js';
import { ClothSimulator } from '../src/cloth.js';
import { FOLD_STAGES, foldGuideStage } from '../src/fold-guide.js';

test('the Task 7 guide walks grasp -> lift -> place -> done with the real fold', () => {
  const motion = planTowelFoldMotion([{ q: [...HOME_POSE], arm: ARM }, { q: [...HOME_POSE], arm: ARM_B }], compiled.environment.safety);
  const cloth = new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2 });
  assert.equal(foldGuideStage(cloth), 'grasp');
  const seen = [];
  motion.frames.forEach((frame, f) => {
    const local = [forwardKinematics(frame[0], ARM), forwardKinematics(frame[1], ARM_B)].map(({ points }) => {
      const [x, y, z] = points.at(-1);
      return { x: (x - 325) / 100, y: (y - 240) / 100, z: z / 100 };
    });
    cloth.step({ targetA: local[0], targetB: local[1], grips: motion.grips[f] });
    const stage = foldGuideStage(cloth);
    if (seen.at(-1) !== stage) seen.push(stage);
  });
  assert.deepEqual(seen, ['grasp', 'lift', 'place', 'done'], `stages seen in order: ${seen.join(' -> ')}`);
  assert.ok(Object.keys(FOLD_STAGES).every((stage) => FOLD_STAGES[stage].length > 0));
});

test('the guide starts over when the towel is reset', () => {
  const cloth = new ClothSimulator({ columns: 8, rows: 6 });
  cloth.step({ targetA: { x: -0.75, y: -0.6, z: 0.02 }, targetB: { x: 0.75, y: -0.6, z: 0.02 }, grips: [true, true] });
  assert.equal(foldGuideStage(cloth), 'lift');
  cloth.reset();
  assert.equal(foldGuideStage(cloth), 'grasp');
  assert.equal(foldGuideStage(null), 'grasp');
});
