import './styles.css';
import { ClothSimulator } from './cloth.js';
import {
  CLOTH_SETTING_GROUPS,
  CLOTH_SETTINGS,
  PREVIEW_FOLD_STEPS,
  STIFFNESS_LOAD_LIMIT,
  clearClothSettings,
  defaultClothSettings,
  loadClothSettings,
  onClothSettingsChange,
  previewFoldTargets,
  saveClothSettings,
} from './cloth-settings.js';

const $ = (selector) => document.querySelector(selector);
const decimalsOf = (step) => (step >= 1 ? 0 : Math.max(0, -Math.floor(Math.log10(step))));
const formatValue = (setting, value) => Number(value).toFixed(decimalsOf(setting.step));

// Same grid as the lab (src/main.js cloth2d, src/viewport3d.js makeCloth).
const GRID = { columns: 14, rows: 11, width: 1.5, height: 1.2 };
const STEPS_PER_SECOND = 50;

let settings = loadClothSettings();
const defaults = defaultClothSettings();
const sim = new ClothSimulator({ ...GRID, ...settings });
const preview = { step: 0, peak: 0, paused: false, accumulator: 0, last: null, cost: 0, phase: 'grasp' };

// --- form -------------------------------------------------------------------

function renderForm() {
  $('#settings-groups').innerHTML = CLOTH_SETTING_GROUPS.map((group) => `
    <fieldset class="settings-group">
      <legend>${group.title}</legend>
      <p class="settings-blurb">${group.blurb}</p>
      ${group.settings.map((setting) => `
        <div class="setting-row" data-key="${setting.key}">
          <label class="setting-label" for="set-${setting.key}">${setting.label}${setting.unit ? ` <small>${setting.unit}</small>` : ''}</label>
          <input id="set-${setting.key}" data-key="${setting.key}" type="range" min="${setting.min}" max="${setting.max}" step="${setting.step}" value="${settings[setting.key]}" aria-describedby="desc-${setting.key}" />
          <input class="setting-number" data-key="${setting.key}" type="number" min="${setting.min}" max="${setting.max}" step="${setting.step}" value="${formatValue(setting, settings[setting.key])}" aria-label="${setting.label} value" />
          <p class="setting-description" id="desc-${setting.key}">${setting.description} <span class="setting-default">Default ${formatValue(setting, defaults[setting.key])}</span></p>
        </div>`).join('')}
    </fieldset>`).join('');
}

function syncForm() {
  for (const setting of CLOTH_SETTINGS) {
    const value = settings[setting.key];
    const row = document.querySelector(`.setting-row[data-key="${setting.key}"]`);
    row.querySelector('input[type="range"]').value = String(value);
    const number = row.querySelector('.setting-number');
    if (document.activeElement !== number) number.value = formatValue(setting, value);
    row.classList.toggle('changed', Math.abs(value - defaults[setting.key]) > setting.step / 2);
  }
  const changed = CLOTH_SETTINGS.filter((setting) => Math.abs(settings[setting.key] - defaults[setting.key]) > setting.step / 2).length;
  $('#settings-saved').textContent = changed ? `Saved · ${changed} changed from default` : 'Defaults';
  updateStability();
}

function applySetting(key, raw) {
  const value = Number(raw);
  if (!Number.isFinite(value)) return;
  const saved = saveClothSettings({ ...settings, [key]: value });
  settings = saved || { ...settings, [key]: value };
  sim.configure(settings);
  syncForm();
  if (!saved) $('#settings-saved').textContent = 'Not saved: browser storage is unavailable';
}

function bindForm() {
  const form = $('#cloth-settings');
  form.addEventListener('input', (event) => {
    const key = event.target.dataset?.key;
    if (key && event.target.value !== '') applySetting(key, event.target.value);
  });
  // A half-typed number is left alone while focused; tidy it on blur.
  form.addEventListener('focusout', (event) => {
    if (event.target.classList?.contains('setting-number')) syncForm();
  });
  $('#settings-reset').addEventListener('click', () => {
    settings = clearClothSettings();
    sim.configure(settings);
    syncForm();
    restartPreview();
  });
  onClothSettingsChange((next) => {
    settings = next;
    sim.configure(settings);
    syncForm();
  });
}

function updateStability() {
  const load = sim.getStiffnessLoad();
  const share = Math.min(1, load / STIFFNESS_LOAD_LIMIT);
  $('#stability-value').textContent = `${load.toFixed(2)} / ${STIFFNESS_LOAD_LIMIT}`;
  const bar = $('#stability-bar');
  bar.style.width = `${share * 100}%`;
  const level = load >= STIFFNESS_LOAD_LIMIT ? 'unstable' : load >= STIFFNESS_LOAD_LIMIT * 0.75 ? 'marginal' : 'stable';
  $('#stability').dataset.level = level;
  $('#stability-note').textContent = {
    stable: 'Stable: the largest per-point spring sum is well under the explicit-update limit.',
    marginal: 'Marginal: close to the limit. Expect jitter on fast pulls.',
    unstable: 'Unstable: over the limit. The cloth will jitter and only the max-move cap holds it together. Lower the stiffnesses.',
  }[level];
}

// --- preview ----------------------------------------------------------------

function restartPreview() {
  sim.reset();
  preview.step = 0;
  preview.peak = 0;
}

function stepPreview() {
  if (preview.step >= PREVIEW_FOLD_STEPS) restartPreview();
  const { targetA, targetB, phase } = previewFoldTargets(preview.step);
  const started = performance.now();
  sim.step({ targetA, targetB });
  preview.cost = preview.cost * 0.9 + (performance.now() - started) * 0.1;
  preview.phase = phase;
  preview.targets = [targetA, targetB];
  preview.step += 1;
  preview.peak = Math.max(preview.peak, sim.getMaxStretchError());
}

const canvas = $('#preview');
const ctx = canvas.getContext('2d');
const TILT = 0.95; // radians from straight down

function project(x, y, z, view) {
  return [view.cx + x * view.scale, view.cy - (y * Math.cos(TILT) + z * Math.sin(TILT)) * view.scale];
}

function themeColor(name) {
  return getComputedStyle(canvas).getPropertyValue(name).trim();
}

function draw() {
  const ratio = window.devicePixelRatio || 1;
  const { clientWidth: width, clientHeight: height } = canvas;
  if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const view = { cx: width / 2, cy: height * 0.62, scale: Math.min(width / 2.3, height / 1.25) };

  // Table.
  const table = [[-1.05, -0.85], [1.05, -0.85], [1.05, 0.85], [-1.05, 0.85]].map(([x, y]) => project(x, y, sim.tableZ, view));
  ctx.fillStyle = themeColor('--preview-table');
  ctx.beginPath();
  table.forEach(([px, py], i) => (i ? ctx.lineTo(px, py) : ctx.moveTo(px, py)));
  ctx.closePath();
  ctx.fill();

  // Fold target: the left half's footprint.
  const target = [[-0.75, -0.6], [0, -0.6], [0, 0.6], [-0.75, 0.6]].map(([x, y]) => project(x, y, sim.tableZ, view));
  ctx.strokeStyle = themeColor('--preview-target');
  ctx.setLineDash([5, 5]);
  ctx.beginPath();
  target.forEach(([px, py], i) => (i ? ctx.lineTo(px, py) : ctx.moveTo(px, py)));
  ctx.closePath();
  ctx.stroke();
  ctx.setLineDash([]);

  // Towel triangles, far to near.
  const p = sim.positions;
  const faces = sim.topology.faces;
  const order = [];
  for (let f = 0; f < faces.length; f += 3) {
    const [a, b, c] = [faces[f] * 3, faces[f + 1] * 3, faces[f + 2] * 3];
    const depth = (p[a + 1] + p[b + 1] + p[c + 1]) * Math.cos(TILT) - (p[a + 2] + p[b + 2] + p[c + 2]) * Math.sin(TILT);
    order.push([depth, f]);
  }
  order.sort((m, n) => n[0] - m[0]);
  const top = themeColor('--preview-cloth');
  const under = themeColor('--preview-cloth-under');
  for (const [, f] of order) {
    const [a, b, c] = [faces[f] * 3, faces[f + 1] * 3, faces[f + 2] * 3];
    const ux = p[b] - p[a]; const uy = p[b + 1] - p[a + 1]; const uz = p[b + 2] - p[a + 2];
    const vx = p[c] - p[a]; const vy = p[c + 1] - p[a + 1]; const vz = p[c + 2] - p[a + 2];
    const nx = uy * vz - uz * vy; const ny = uz * vx - ux * vz; const nz = ux * vy - uy * vx;
    const len = Math.hypot(nx, ny, nz) || 1;
    const light = Math.abs((nx * -0.3 + ny * -0.4 + nz * 0.87) / len);
    const pa = project(p[a], p[a + 1], p[a + 2], view);
    const pb = project(p[b], p[b + 1], p[b + 2], view);
    const pc = project(p[c], p[c + 1], p[c + 2], view);
    const facingUp = (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0]) < 0;
    ctx.fillStyle = facingUp ? top : under;
    ctx.globalAlpha = 0.55 + light * 0.45;
    ctx.beginPath();
    ctx.moveTo(...pa);
    ctx.lineTo(...pb);
    ctx.lineTo(...pc);
    ctx.closePath();
    ctx.fill();
    ctx.globalAlpha = 0.25;
    ctx.strokeStyle = themeColor('--preview-wire');
    ctx.stroke();
  }
  ctx.globalAlpha = 1;

  // Grippers.
  (preview.targets || []).forEach((targetPoint, index) => {
    const [gx, gy] = project(targetPoint.x, targetPoint.y, targetPoint.z, view);
    const [sx, sy] = project(targetPoint.x, targetPoint.y, sim.tableZ, view);
    ctx.strokeStyle = themeColor(index ? '--preview-arm-b' : '--preview-arm-a');
    ctx.setLineDash([2, 3]);
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(gx, gy);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = ctx.strokeStyle;
    ctx.beginPath();
    ctx.arc(gx, gy, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.font = '10px "DM Mono", monospace';
    ctx.fillText(index ? 'B' : 'A', gx + 8, gy - 6);
  });
}

function updateMetrics() {
  const metrics = sim.getFoldMetrics();
  $('#metric-stretch').textContent = `${(metrics.stretchError * 100).toFixed(1)}%`;
  $('#metric-peak').textContent = `${(preview.peak * 100).toFixed(1)}%`;
  $('#metric-fold').textContent = `${(metrics.frontDistance * 10).toFixed(1)} cm${metrics.folded ? ' · folded' : ''}`;
  $('#metric-cost').textContent = `${preview.cost.toFixed(2)} ms/step`;
  $('#preview-phase').textContent = `${preview.phase} · ${preview.step} / ${PREVIEW_FOLD_STEPS}`;
}

function frame(now) {
  if (preview.last === null) preview.last = now;
  const elapsed = Math.min(0.1, (now - preview.last) / 1000);
  preview.last = now;
  if (!preview.paused) {
    preview.accumulator += elapsed;
    while (preview.accumulator >= 1 / STEPS_PER_SECOND) {
      preview.accumulator -= 1 / STEPS_PER_SECOND;
      stepPreview();
    }
  }
  draw();
  updateMetrics();
  requestAnimationFrame(frame);
}

renderForm();
bindForm();
syncForm();
$('#preview-restart').addEventListener('click', restartPreview);
$('#preview-pause').addEventListener('click', (event) => {
  preview.paused = !preview.paused;
  event.currentTarget.textContent = preview.paused ? '▶ Resume' : '❚❚ Pause';
});
stepPreview();
requestAnimationFrame(frame);
