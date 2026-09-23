/**
 * Tunable towel physics, shared by the cloth settings page (cloth.html) and
 * the live lab. Values persist in localStorage; the lab picks up changes made
 * in another tab through the `storage` event.
 */
import { CLOTH_PHYSICS_KEYS, DEFAULT_CLOTH_CONFIG } from './cloth.js';

export const CLOTH_SETTINGS_STORAGE_KEY = 'armlab.cloth-settings.v1';

/** Stiffness load above which the explicit spring update goes unstable. */
export const STIFFNESS_LOAD_LIMIT = 4;

/**
 * One entry per control on the settings page. `min`/`max` bound what the
 * page and sanitizeClothSettings() accept.
 */
export const CLOTH_SETTING_GROUPS = Object.freeze([
  {
    id: 'springs',
    title: 'Springs',
    blurb: 'Each spring moves its two ends by stiffness / 2 of its length error per sub-step.',
    settings: [
      { key: 'stretchStiffness', label: 'Stretch stiffness', min: 0, max: 1, step: 0.01, description: 'Grid edges along the weave. Higher keeps the towel from stretching under a pull.' },
      { key: 'shearStiffness', label: 'Shear stiffness', min: 0, max: 1, step: 0.01, description: 'Quad diagonals. Resists the weave skewing into a parallelogram.' },
      { key: 'bendStiffness', label: 'Bend stiffness', min: 0, max: 0.3, step: 0.001, description: 'Across shared edges. A towel is limp; raise it for card-like fabric.' },
      { key: 'springDamping', label: 'Spring damping', min: 0, max: 0.5, step: 0.01, description: 'Dashpot on each spring. Kills jiggle without slowing free fall; high values go unstable.' },
    ],
  },
  {
    id: 'motion',
    title: 'Motion',
    blurb: 'Per-step values, spread evenly over the sub-steps.',
    settings: [
      { key: 'gravity', label: 'Gravity', min: -0.06, max: 0, step: 0.001, unit: '/step²', description: 'Downward acceleration. About -0.039 is real gravity for 10 cm units at 50 steps/s.' },
      { key: 'damping', label: 'Air damping', min: 0, max: 0.6, step: 0.01, description: 'Share of velocity lost per step. Stops a yanked corner flinging the towel.' },
      { key: 'maxForce', label: 'Max move', min: 0.002, max: 0.1, step: 0.001, unit: '/sub-step', description: 'Cap on how far a point moves per sub-step.' },
      { key: 'substeps', label: 'Sub-steps', min: 4, max: 64, step: 1, description: 'Solver passes per step. More is more accurate and slower.' },
    ],
  },
  {
    id: 'contact',
    title: 'Contact',
    blurb: 'Table friction, fold layering and the grippers.',
    settings: [
      { key: 'staticFriction', label: 'Static friction', min: 0, max: 3, step: 0.05, description: 'Resting cloth holds while sideways pull is under this × the table push.' },
      { key: 'kineticFriction', label: 'Kinetic friction', min: 0, max: 3, step: 0.05, description: 'Sliding cloth loses this × the table push per sub-step.' },
      { key: 'thickness', label: 'Layer thickness', min: 0.005, max: 0.1, step: 0.001, unit: 'u', description: 'Gap kept between the folded half and the half beneath it.' },
      { key: 'graspRadius', label: 'Grasp radius', min: 0.05, max: 0.6, step: 0.01, unit: 'u', description: 'How close a gripper must come to a corner to take it.' },
      { key: 'releaseRadius', label: 'Release radius', min: 0.1, max: 1.2, step: 0.01, unit: 'u', description: 'How far Arm A may leave its grasp point before it lets go and the corner is pinned.' },
    ],
  },
]);

export const CLOTH_SETTINGS = Object.freeze(CLOTH_SETTING_GROUPS.flatMap((group) => group.settings));

export function defaultClothSettings() {
  return Object.fromEntries(CLOTH_PHYSICS_KEYS.map((key) => [key, DEFAULT_CLOTH_CONFIG[key]]));
}

/** Keep only known keys, clamped to their range; anything else falls back to the default. */
export function sanitizeClothSettings(input) {
  const settings = defaultClothSettings();
  if (!input || typeof input !== 'object') return settings;
  for (const { key, min, max, step } of CLOTH_SETTINGS) {
    const value = Number(input[key]);
    if (input[key] === null || input[key] === '' || !Number.isFinite(value)) continue;
    const clamped = Math.min(max, Math.max(min, value));
    settings[key] = step >= 1 ? Math.round(clamped) : clamped;
  }
  return settings;
}

function storageOf(storage) {
  if (storage) return storage;
  try {
    return globalThis.localStorage ?? null;
  } catch {
    return null;
  }
}

export function loadClothSettings(storage) {
  try {
    const raw = storageOf(storage)?.getItem(CLOTH_SETTINGS_STORAGE_KEY);
    return sanitizeClothSettings(raw ? JSON.parse(raw) : null);
  } catch {
    return defaultClothSettings();
  }
}

/** Persist settings; returns the sanitized values actually stored (or false if storage is unavailable). */
export function saveClothSettings(settings, storage) {
  const clean = sanitizeClothSettings(settings);
  try {
    const target = storageOf(storage);
    if (!target) return false;
    target.setItem(CLOTH_SETTINGS_STORAGE_KEY, JSON.stringify(clean));
    return clean;
  } catch {
    return false;
  }
}

export function clearClothSettings(storage) {
  try {
    storageOf(storage)?.removeItem(CLOTH_SETTINGS_STORAGE_KEY);
  } catch {
    // Storage blocked: nothing persisted, nothing to clear.
  }
  return defaultClothSettings();
}

/** Call `listener(settings)` whenever another tab saves or clears cloth settings. */
export function onClothSettingsChange(listener) {
  if (typeof window === 'undefined') return () => {};
  const handler = (event) => {
    if (event.key === CLOTH_SETTINGS_STORAGE_KEY || event.key === null) listener(loadClothSettings());
  };
  window.addEventListener('storage', handler);
  return () => window.removeEventListener('storage', handler);
}

/** Frame counts of the settings page's scripted fold: grasp, carry, then hold. */
export const PREVIEW_FOLD = Object.freeze({ grasp: 15, carry: 80, hold: 70 });
export const PREVIEW_FOLD_STEPS = PREVIEW_FOLD.grasp + PREVIEW_FOLD.carry + PREVIEW_FOLD.hold;

/**
 * Gripper targets (cloth-local units) for step `n` of the preview fold: Arm A
 * holds the front-left corner down while Arm B lifts the front-right corner
 * over the midline and lays it just inside the left edge.
 */
export function previewFoldTargets(n) {
  const targetA = { x: -0.75, y: -0.6, z: 0.02 };
  const t = Math.min(1, Math.max(0, (n - PREVIEW_FOLD.grasp) / PREVIEW_FOLD.carry));
  const eased = t * t * (3 - 2 * t);
  const targetB = { x: 0.75 - eased * 1.4, y: -0.6, z: 0.02 + Math.sin(eased * Math.PI) * 0.4 + eased * 0.04 };
  const phase = n < PREVIEW_FOLD.grasp ? 'grasp' : n < PREVIEW_FOLD.grasp + PREVIEW_FOLD.carry ? 'carry' : 'hold';
  return { targetA, targetB, phase };
}
