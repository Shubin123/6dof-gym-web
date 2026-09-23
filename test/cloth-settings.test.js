import test from 'node:test';
import assert from 'node:assert/strict';
import { ClothSimulator, CLOTH_PHYSICS_KEYS, DEFAULT_CLOTH_CONFIG } from '../src/cloth.js';
import {
  CLOTH_SETTINGS,
  CLOTH_SETTINGS_STORAGE_KEY,
  PREVIEW_FOLD_STEPS,
  STIFFNESS_LOAD_LIMIT,
  clearClothSettings,
  defaultClothSettings,
  loadClothSettings,
  previewFoldTargets,
  sanitizeClothSettings,
  saveClothSettings,
} from '../src/cloth-settings.js';

function memoryStorage() {
  const data = new Map();
  return {
    getItem: (key) => (data.has(key) ? data.get(key) : null),
    setItem: (key, value) => data.set(key, String(value)),
    removeItem: (key) => data.delete(key),
    data,
  };
}

test('every live-tunable cloth parameter has exactly one settings control', () => {
  assert.deepEqual(CLOTH_SETTINGS.map((s) => s.key).sort(), [...CLOTH_PHYSICS_KEYS].sort());
  for (const setting of CLOTH_SETTINGS) {
    const fallback = DEFAULT_CLOTH_CONFIG[setting.key];
    assert.ok(setting.min <= fallback && fallback <= setting.max, `${setting.key} default ${fallback} lies inside [${setting.min}, ${setting.max}]`);
    assert.ok(setting.description.length > 10, `${setting.key} explains itself`);
  }
});

test('sanitizeClothSettings clamps, rounds integer steps, and drops junk', () => {
  const clean = sanitizeClothSettings({ stretchStiffness: 7, gravity: 'abc', substeps: 12.6, bendStiffness: null, columns: 99, evil: '<script>' });
  assert.equal(clean.stretchStiffness, 1, 'clamped to the control maximum');
  assert.equal(clean.gravity, DEFAULT_CLOTH_CONFIG.gravity, 'non-numeric falls back to default');
  assert.equal(clean.substeps, 13, 'integer controls round');
  assert.equal(clean.bendStiffness, DEFAULT_CLOTH_CONFIG.bendStiffness);
  assert.ok(!('columns' in clean), 'mesh size is not a setting');
  assert.ok(!('evil' in clean));
  assert.deepEqual(sanitizeClothSettings('nope'), defaultClothSettings());
});

test('settings round-trip through storage and survive corrupt or blocked storage', () => {
  const storage = memoryStorage();
  assert.deepEqual(loadClothSettings(storage), defaultClothSettings(), 'empty storage gives defaults');

  const saved = saveClothSettings({ ...defaultClothSettings(), damping: 0.3 }, storage);
  assert.equal(saved.damping, 0.3);
  assert.equal(loadClothSettings(storage).damping, 0.3);

  storage.setItem(CLOTH_SETTINGS_STORAGE_KEY, '{not json');
  assert.deepEqual(loadClothSettings(storage), defaultClothSettings(), 'corrupt JSON gives defaults');

  const blocked = { getItem() { throw new Error('denied'); }, setItem() { throw new Error('denied'); }, removeItem() { throw new Error('denied'); } };
  assert.deepEqual(loadClothSettings(blocked), defaultClothSettings());
  assert.equal(saveClothSettings({ damping: 0.3 }, blocked), false);
  assert.deepEqual(clearClothSettings(blocked), defaultClothSettings());

  saveClothSettings({ damping: 0.3 }, storage);
  clearClothSettings(storage);
  assert.equal(storage.data.size, 0, 'reset removes the stored override');
});

test('configure() retunes a live simulator without touching its state', () => {
  const sim = new ClothSimulator({ columns: 8, rows: 6 });
  sim.step({ targetA: { x: -0.75, y: -0.6, z: 0.02 } });
  const before = Float32Array.from(sim.positions);
  const captured = [...sim.captured];

  sim.configure({ stretchStiffness: 0.2, bendStiffness: 0.05, substeps: 10.4, columns: 3 });
  assert.equal(sim.stretchStiffness, 0.2);
  assert.equal(sim.substeps, 10, 'sub-steps are whole passes');
  assert.equal(sim.columns, 8, 'mesh size cannot change on a live simulator');
  assert.deepEqual(Array.from(sim.positions), Array.from(before), 'positions untouched');
  assert.deepEqual(sim.captured, captured, 'grasp state untouched');
  const stretchSprings = [...sim.springKinds.keys()].filter((s) => sim.springKinds[s] === 0);
  assert.ok(stretchSprings.every((s) => Math.abs(sim.springStiffness[s] - 0.2) < 1e-6), 'per-spring stiffness store follows the new value');
});

test('stiffness load separates the shipped defaults from the setting that jittered', () => {
  assert.ok(new ClothSimulator().getStiffnessLoad() < STIFFNESS_LOAD_LIMIT * 0.75, 'defaults are comfortably stable');
  const stiff = new ClothSimulator({ stretchStiffness: 0.9, shearStiffness: 0.6, bendStiffness: 0.18 });
  assert.ok(stiff.getStiffnessLoad() > STIFFNESS_LOAD_LIMIT, 'the pre-tuning stiffnesses are flagged unstable');
});

test('the settings page preview fold actually folds the towel with default settings', () => {
  const sim = new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2, ...defaultClothSettings() });
  let peak = 0;
  for (let n = 0; n < PREVIEW_FOLD_STEPS; n += 1) {
    const { targetA, targetB } = previewFoldTargets(n);
    sim.step({ targetA, targetB });
    peak = Math.max(peak, sim.getMaxStretchError());
  }
  assert.deepEqual(sim.captured, [true, true], 'both grippers keep their corners');
  const metrics = sim.getFoldMetrics();
  assert.equal(metrics.folded, true);
  assert.ok(peak < 0.35, `preview stays physically plausible: peak stretch ${peak.toFixed(3)}`);
  assert.deepEqual(previewFoldTargets(0).phase, 'grasp');
  assert.deepEqual(previewFoldTargets(PREVIEW_FOLD_STEPS - 1).phase, 'hold');
});
