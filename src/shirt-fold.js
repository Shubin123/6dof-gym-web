/**
 * Specialized bimanual multi-fold garment planner and topology for shirt handling (Task 17).
 *
 * Robotic shirt folding benchmark:
 * Unlike flat rectangular towels, garments possess non-trivial deformable geometry
 * with sleeves extending outward from a central torso. A proper fold requires a
 * sequential multi-fold strategy:
 *  1. Left sleeve fold: Arm A grasps the left sleeve cuff and folds it inward across the torso
 *  2. Right sleeve fold: Arm B grasps the right sleeve cuff and folds it inward across the torso
 *  3. Bottom hem fold (bimanual): Both arms grasp the bottom hem corners together,
 *     lift along parallel arcs over the waist crease line, and place on the upper chest/collar
 *  4. Cloth relaxation: Arms retract safely home while multi-layer cloth physics settles
 */

import { evaluateCellSafety, HOME_POSE, planSafeCellMotion, planSafeMotion } from './core.js';
import { planClearCartesianMotion, solveClearIK } from './arm-track.js';

const hold = (frames, length) => {
  const last = frames.at(-1);
  return [...frames, ...Array(Math.max(0, length - frames.length)).fill(last)];
};

const reverseSegment = (from, frames) => [...frames.slice(0, -1).reverse(), from];

/** Points along a parabolic arc from `from` to `to` in scene pixels [x, y], peaking at `height`. */
export function arcPoints(from, to, { startZ = 4, endZ = 6, height = 40, count = 8 } = {}) {
  const points = [];
  for (let i = 1; i <= count; i += 1) {
    const angle = (i / count) * Math.PI;
    const t = (1 - Math.cos(angle)) / 2;
    const z = i === count ? endZ : startZ + (endZ - startZ) * t + height * Math.sin(angle);
    points.push([from[0] + (to[0] - from[0]) * t, from[1] + (to[1] - from[1]) * t, z]);
  }
  return points;
}

/**
 * Creates a structured indexed triangle mesh and topology for a T-shirt.
 *
 * Torso: columns 4 to 12 (width 0.9m, height 1.2m), centered at (0, 0).
 * Left sleeve: columns 0 to 4, rows 1 to 5 (extending left to -0.9m).
 * Right sleeve: columns 12 to 16, rows 1 to 5 (extending right to +0.9m).
 * Cutouts: Empty space below sleeves (flanks), above sloped shoulders, and collar scoop.
 */
export function createShirtMesh({
  columns = 16,
  rows = 12,
  width = 1.8,
  height = 1.2,
  tableZ = 0.018,
} = {}) {
  function isCellInShirt(c, r) {
    // Cutouts above sloped sleeve tops
    if ((c < 4 || c >= 12) && r === 0) return false;
    // Cutouts under sleeves (empty flank spaces beside torso)
    if ((c < 4 || c >= 12) && r >= 5) return false;
    // Collar notch at neck
    if ((c === 7 || c === 8) && r === 0) return false;
    return true;
  }

  const positions = [];
  const index = [];
  const coordToVertex = new Map();
  const quads = [];

  function getVertex(c, r) {
    const key = `${c},${r}`;
    if (coordToVertex.has(key)) return coordToVertex.get(key);
    const v = positions.length / 3;
    const x = (c * width) / columns - width / 2;
    const y = (r * height) / rows - height / 2;
    positions.push(x, y, tableZ);
    coordToVertex.set(key, v);
    return v;
  }

  for (let r = 0; r < rows; r += 1) {
    for (let c = 0; c < columns; c += 1) {
      if (!isCellInShirt(c, r)) continue;
      const v00 = getVertex(c, r);
      const v10 = getVertex(c + 1, r);
      const v01 = getVertex(c, r + 1);
      const v11 = getVertex(c + 1, r + 1);
      // Quad winding: v00, v10, v11, v01
      quads.push([v00, v10, v11, v01]);
      index.push(v00, v01, v10);
      index.push(v01, v11, v10);
    }
  }

  // Tag vertices by anatomical garment region for multi-layer collision separation
  const numVertices = positions.length / 3;
  const regions = new Array(numVertices);
  for (const [key, v] of coordToVertex.entries()) {
    const [c, r] = key.split(',').map(Number);
    if (c < 4) regions[v] = 'sleeve_left';
    else if (c > 12) regions[v] = 'sleeve_right';
    else if (r <= 6) regions[v] = 'torso_upper';
    else regions[v] = 'torso_lower';
  }

  // Key landmarks for grasps and fold targets
  const landmarks = {
    leftSleeveCuff: coordToVertex.get('0,3'),
    leftSleeveTarget: coordToVertex.get('6,3'),
    rightSleeveCuff: coordToVertex.get('16,3'),
    rightSleeveTarget: coordToVertex.get('10,3'),
    hemLeft: coordToVertex.get('4,12'),
    hemRight: coordToVertex.get('12,12'),
    hemTargetLeft: coordToVertex.get('4,2'),
    hemTargetRight: coordToVertex.get('12,2'),
  };

  // Crease line vertex segments across the 3 fold axes:
  // 1. Left sleeve fold seam (column 4)
  // 2. Right sleeve fold seam (column 12)
  // 3. Waist / hem fold line (row 7)
  const leftCrease = [];
  for (let r = 1; r <= 5; r += 1) if (coordToVertex.has(`4,${r}`)) leftCrease.push(coordToVertex.get(`4,${r}`));
  const rightCrease = [];
  for (let r = 1; r <= 5; r += 1) if (coordToVertex.has(`12,${r}`)) rightCrease.push(coordToVertex.get(`12,${r}`));
  const hemCrease = [];
  for (let c = 4; c <= 12; c += 1) if (coordToVertex.has(`${c},7`)) hemCrease.push(coordToVertex.get(`${c},7`));

  return {
    mesh: { positions: new Float32Array(positions), index: new Uint32Array(index) },
    columns,
    rows,
    width,
    height,
    tableZ,
    numVertices,
    landmarks,
    regions,
    quads,
    creaseLines: [leftCrease, rightCrease, hemCrease],
  };
}

/** Precomputed shirt topology cache */
let cachedShirtInfo = null;
export function getShirtMeshInfo() {
  if (!cachedShirtInfo) cachedShirtInfo = createShirtMesh();
  return cachedShirtInfo;
}

/** One arm carry planner, testing grasp-to-place and place-to-grasp backward routes. */
function carryPlan(arm, from, to, { graspZ = 4, hoverZ = 38, placeZ = 6, arcHeight = 35 } = {}, safety) {
  const arc = arcPoints(from, to, { startZ: graspZ, endZ: placeZ, height: arcHeight });
  for (const grasp of solveClearIK([...from, graspZ], arm, safety, [], { limit: 5 })) {
    const rise = planClearCartesianMotion(grasp, [[...from, hoverZ]], arm, safety);
    const carry = rise && planClearCartesianMotion(grasp, arc, arm, safety);
    if (carry) return { grasp, rise, arc: carry };
  }
  const back = arcPoints(to, from, { startZ: placeZ, endZ: graspZ, height: arcHeight });
  for (const place of solveClearIK([...to, placeZ], arm, safety, [], { limit: 5 })) {
    const reversed = planClearCartesianMotion(place, back, arm, safety);
    if (!reversed) continue;
    const grasp = reversed.at(-1);
    const rise = planClearCartesianMotion(grasp, [[...from, hoverZ]], arm, safety);
    if (rise) return { grasp, rise, arc: reverseSegment(place, reversed) };
  }
  return null;
}

const cachedShirtFolds = new Map();
const atHome = (q) => q.every((value, joint) => Math.abs(value - HOME_POSE[joint]) < 1e-3);

/**
 * Plans the complete 3-stage bimanual shirt multi-fold:
 *  1. Left sleeve fold: Arm A grasps left cuff, folds inward over torso, releases, clears.
 *  2. Right sleeve fold: Arm B grasps right cuff, folds inward over torso, releases, clears.
 *  3. Bimanual hem fold: Both arms grasp hem corners, lift upward over waist crease, place, retract.
 *  4. Settle tail.
 */
export function planShirtMultiFoldMotion(poses, safety = {}, profile = {}) {
  if (poses.length < 2) return null;
  const fromHome = poses.every(({ q }) => atHome(q));
  const cacheKey = JSON.stringify([profile, safety]);
  if (fromHome && cachedShirtFolds.has(cacheKey)) return structuredClone(cachedShirtFolds.get(cacheKey));
  const result = planShirtFold(poses, safety, profile);
  if (fromHome && result) cachedShirtFolds.set(cacheKey, structuredClone(result));
  return result;
}

function planShirtFold(poses, safety, profile) {
  const arms = poses.map(({ arm }) => arm);
  const midline = (arms[0].base[0] + arms[1].base[0]) / 2;
  const half = (safety.arm_clearance_px ?? 0) / 2;
  const [minX, maxX, minY, maxY] = safety.goal_workspace;

  // Split cell workspace into safe lanes to avoid elbow interference during IK solve
  const laneA = { ...safety, goal_workspace: [minX, midline - half, minY, maxY] };
  const laneB = { ...safety, goal_workspace: [midline + half, maxX, minY, maxY] };

  const optsSleeve = {
    graspZ: profile.graspZ ?? 4,
    hoverZ: profile.hoverZ ?? 38,
    placeZ: profile.placeHeight ?? 6,
    arcHeight: profile.arcHeightSleeve ?? 35,
  };
  const optsHem = {
    graspZ: profile.graspZ ?? 4,
    hoverZ: profile.hoverZ ?? 40,
    placeZ: profile.placeHeight ?? 8,
    arcHeight: profile.arcHeightHem ?? 30,
  };
  const settleFrames = profile.settleFrames ?? 60;
  const dwellFrames = 3;

  // Scene pixel targets for garment landmarks (centred on [380, 240])
  const sleeveAFrom = profile.sleeveAFrom ?? [290, 210];
  const sleeveATo = profile.sleeveATo ?? [358, 210];
  const sleeveBFrom = profile.sleeveBFrom ?? [470, 210];
  const sleeveBTo = profile.sleeveBTo ?? [402, 210];
  const hemAFrom = profile.hemAFrom ?? [335, 300];
  const hemATo = profile.hemATo ?? [335, 200];
  const hemBFrom = profile.hemBFrom ?? [425, 300];
  const hemBTo = profile.hemBTo ?? [425, 200];

  const cLeft = carryPlan(arms[0], sleeveAFrom, sleeveATo, optsSleeve, laneA);
  const cRight = carryPlan(arms[1], sleeveBFrom, sleeveBTo, optsSleeve, laneB);
  const cHemA = carryPlan(arms[0], hemAFrom, hemATo, optsHem, laneA);
  const cHemB = carryPlan(arms[1], hemBFrom, hemBTo, optsHem, laneB);

  if (!cLeft || !cRight || !cHemA || !cHemB) return null;

  const frames = [];
  const grips = [];
  const stageEnds = [];
  let curA = [...poses[0].q];
  let curB = [...poses[1].q];
  let grip = [false, false];

  const cellSafe = (frame) => evaluateCellSafety(frame.map((q, index) => ({ q, arm: arms[index] })), safety).safe;
  const push = (qA, qB) => {
    curA = [...qA];
    curB = [...qB];
    frames.push([[...curA], [...curB]]);
    grips.push([...grip]);
  };
  const dwell = (count) => { for (let i = 0; i < count; i += 1) push(curA, curB); };
  const endStage = () => stageEnds.push(frames.length - 1);

  const together = (tracks) => {
    const length = Math.max(...tracks.map((t) => t.length));
    const padded = tracks.map((t) => hold(t, length));
    for (let i = 0; i < length; i += 1) {
      if (!cellSafe([padded[0][i], padded[1][i]])) return false;
      push(padded[0][i], padded[1][i]);
    }
    return true;
  };

  // ==========================================
  // STAGE 1: Left sleeve fold inward (Arm A)
  // ==========================================
  // 1a. Approach Arm A above left cuff
  const appLeft = planSafeMotion(curA, cLeft.rise.at(-1), arms[0], safety, [{ q: curB, arm: arms[1] }]);
  if (!appLeft) return null;
  appLeft.forEach((qA) => push(qA, curB));
  endStage();

  // 1b. Descend straight to cuff, grasp, carry arc inward over left chest
  reverseSegment(cLeft.grasp, cLeft.rise).forEach((qA) => push(qA, curB));
  grip = [true, false];
  dwell(dwellFrames);
  cLeft.arc.forEach((qA) => push(qA, curB));
  dwell(dwellFrames);
  // Release on torso
  grip = [false, false];
  dwell(dwellFrames);
  endStage();

  // 1c. Rise straight off shirt and retract to HOME
  const riseLeft = planClearCartesianMotion(curA, [[sleeveATo[0], sleeveATo[1], optsSleeve.hoverZ]], arms[0], laneA);
  if (!riseLeft) return null;
  riseLeft.forEach((qA) => push(qA, curB));
  const retLeft = planSafeMotion(curA, [...HOME_POSE], arms[0], safety, [{ q: curB, arm: arms[1] }]);
  if (!retLeft) return null;
  retLeft.forEach((qA) => push(qA, curB));
  endStage();

  // ==========================================
  // STAGE 2: Right sleeve fold inward (Arm B)
  // ==========================================
  // 2a. Approach Arm B above right cuff
  const appRight = planSafeMotion(curB, cRight.rise.at(-1), arms[1], safety, [{ q: curA, arm: arms[0] }]);
  if (!appRight) return null;
  appRight.forEach((qB) => push(curA, qB));
  endStage();

  // 2b. Descend straight to cuff, grasp, carry arc inward over right chest
  reverseSegment(cRight.grasp, cRight.rise).forEach((qB) => push(curA, qB));
  grip = [false, true];
  dwell(dwellFrames);
  cRight.arc.forEach((qB) => push(curA, qB));
  dwell(dwellFrames);
  // Release on torso
  grip = [false, false];
  dwell(dwellFrames);
  endStage();

  // 2c. Rise straight off shirt and retract to HOME
  const riseRight = planClearCartesianMotion(curB, [[sleeveBTo[0], sleeveBTo[1], optsSleeve.hoverZ]], arms[1], laneB);
  if (!riseRight) return null;
  riseRight.forEach((qB) => push(curA, qB));
  const retRight = planSafeMotion(curB, [...HOME_POSE], arms[1], safety, [{ q: curA, arm: arms[0] }]);
  if (!retRight) return null;
  retRight.forEach((qB) => push(curA, qB));
  endStage();

  // ==========================================
  // STAGE 3: Bimanual hem fold upward
  // ==========================================
  // 3a. Bimanual approach to bottom hem corners
  const appHem = planSafeCellMotion(
    [{ q: curA, arm: arms[0] }, { q: curB, arm: arms[1] }],
    [cHemA.rise.at(-1), cHemB.rise.at(-1)],
    safety,
  );
  if (!appHem) return null;
  appHem.frames.forEach(([qA, qB]) => push(qA, qB));
  endStage();

  // 3b. Descend together straight onto hem corners, grasp, carry arc over waist crease
  if (!together([reverseSegment(cHemA.grasp, cHemA.rise), reverseSegment(cHemB.grasp, cHemB.rise)])) return null;
  grip = [true, true];
  dwell(dwellFrames);
  if (!together([cHemA.arc, cHemB.arc])) return null;
  dwell(dwellFrames);
  // Release hem on upper torso
  grip = [false, false];
  dwell(dwellFrames);
  endStage();

  // 3c. Rise off garment and retract both arms to HOME
  const riseHemA = planClearCartesianMotion(curA, [[hemATo[0], hemATo[1], optsHem.hoverZ]], arms[0], laneA);
  const riseHemB = planClearCartesianMotion(curB, [[hemBTo[0], hemBTo[1], optsHem.hoverZ]], arms[1], laneB);
  if (!riseHemA || !riseHemB || !together([riseHemA, riseHemB])) return null;

  const retHemA = planSafeMotion(curA, [...HOME_POSE], arms[0], safety, [{ q: curB, arm: arms[1] }]);
  if (retHemA) retHemA.forEach((qA) => push(qA, curB));
  const retHemB = planSafeMotion(curB, [...HOME_POSE], arms[1], safety, [{ q: curA, arm: arms[0] }]);
  if (retHemB) retHemB.forEach((qB) => push(curA, qB));
  endStage();

  // ==========================================
  // STAGE 4: Cloth settling dwell
  // ==========================================
  dwell(settleFrames);
  endStage();

  return { frames, grips, stageEnds, sourceFrames: frames.length, reducedBy: 0 };
}

/** Evaluates sleeve & hem alignment metrics for shirt multi-fold. */
export function getShirtFoldMetrics(cloth) {
  const p = cloth.positions;
  const info = getShirtMeshInfo();
  const { landmarks } = info;

  const dist2D = (i1, i2) => Math.hypot(p[i1 * 3] - p[i2 * 3], p[i1 * 3 + 1] - p[i2 * 3 + 1]);

  const leftSleeveGap = dist2D(landmarks.leftSleeveCuff, landmarks.leftSleeveTarget);
  const rightSleeveGap = dist2D(landmarks.rightSleeveCuff, landmarks.rightSleeveTarget);
  const hemGaps = [
    dist2D(landmarks.hemLeft, landmarks.hemTargetLeft),
    dist2D(landmarks.hemRight, landmarks.hemTargetRight),
  ];
  const maxHemGap = Math.max(...hemGaps);

  const folded = leftSleeveGap < 0.22 && rightSleeveGap < 0.22 && maxHemGap < 0.24;
  const stretchError = cloth.getMaxStretchError ? cloth.getMaxStretchError() : 0;
  const settled = cloth.settled ?? true;

  return {
    leftSleeveGap,
    rightSleeveGap,
    hemGaps,
    maxHemGap,
    folded,
    stretchError,
    settled,
  };
}

/** Dense stage scoring for the shirt multi-fold task. */
export function scoreShirtFold(recipe, cloth) {
  const metrics = getShirtFoldMetrics(cloth);
  const wasCapA = cloth.wasCaptured?.[0] ?? false;
  const wasCapB = cloth.wasCaptured?.[1] ?? false;
  const isCapA = cloth.captured?.[0] ?? false;
  const isCapB = cloth.captured?.[1] ?? false;

  const clamp01 = (v) => Math.max(0, Math.min(1, v));

  const scores = {
    left_sleeve: metrics.leftSleeveGap < 0.22 ? 1 : clamp01(1 - (metrics.leftSleeveGap - 0.22) / 0.5),
    right_sleeve: metrics.rightSleeveGap < 0.22 ? 1 : clamp01(1 - (metrics.rightSleeveGap - 0.22) / 0.5),
    hem_carry: metrics.maxHemGap < 0.24 ? 1 : clamp01(1 - (metrics.maxHemGap - 0.24) / 0.7),
    place_hem: metrics.folded && !isCapA && !isCapB && wasCapA && wasCapB ? 1 : 0,
    settle: metrics.folded && metrics.stretchError < 0.25 ? 1 : 0,
  };

  const stage = recipe.stages.find((entry) => scores[entry.id] < 0.999) || recipe.stages.at(-1);
  const reward = recipe.stages.reduce((sum, entry) => sum + entry.weight * scores[entry.id], 0);

  return {
    reward,
    stage: stage.label,
    complete: scores.place_hem >= 0.999 && scores.settle >= 0.999,
    metrics,
  };
}

/** 2D SVG Guide for Shirt Multi-Fold. */
export function shirtFoldGuideSvg(cloth, origin = [380, 240], pxPerUnit = 100) {
  const info = getShirtMeshInfo();
  const metrics = getShirtFoldMetrics(cloth);
  const [ox, oy] = origin;
  const fmt = (v) => Number(v.toFixed(1));

  // Crease lines in scene coordinates
  const leftX = ox - 45; // Seam at c = 4
  const rightX = ox + 45; // Seam at c = 12
  const waistY = oy; // Waist line at r = 6/7

  // Target bounding box for folded garment
  const targetX = ox - 45;
  const targetY = oy - 60;
  const targetW = 90;
  const targetH = 60;

  // Arrow markers and paths
  // Arrow 1: Left sleeve fold inward
  const a1From = [ox - 90, oy - 30];
  const a1To = [ox - 22, oy - 30];
  // Arrow 2: Right sleeve fold inward
  const a2From = [ox + 90, oy - 30];
  const a2To = [ox + 22, oy - 30];
  // Arrow 3: Hem fold upward (from hem landmarks [335, 300] and [425, 300] to [335, 200] and [425, 200])
  const a3FromL = [ox - 45, oy + 60];
  const a3ToL = [ox - 45, oy - 40];
  const a3FromR = [ox + 45, oy + 60];
  const a3ToR = [ox + 45, oy - 40];

  const isSleevePhase = metrics.leftSleeveGap >= 0.22 || metrics.rightSleeveGap >= 0.22;
  const aHeldSleeve = isSleevePhase && Boolean(cloth.captured?.[0]);
  const bHeldSleeve = isSleevePhase && Boolean(cloth.captured?.[1]);
  const aHeldHem = !isSleevePhase && Boolean(cloth.captured?.[0]);
  const bHeldHem = !isSleevePhase && Boolean(cloth.captured?.[1]);

  const status = metrics.folded
    ? `Folded · Sleeves & hem aligned (${fmt(metrics.leftSleeveGap * 10)} / ${fmt(metrics.rightSleeveGap * 10)} cm)`
    : metrics.leftSleeveGap < 0.22 && metrics.rightSleeveGap < 0.22
    ? 'Sleeves folded · Hem lifting upward'
    : metrics.leftSleeveGap < 0.22
    ? 'Left sleeve tucked · Folding right sleeve'
    : 'Shirt multi-fold · 1: Left sleeve, 2: Right sleeve, 3: Hem';

  return `
    <defs>
      <marker id="fold-arrowhead" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
        <path d="M0 0L10 5L0 10z" class="fold-arrowhead" />
      </marker>
    </defs>
    <!-- Folded target compact zone -->
    <rect class="cloth-2d-target" x="${fmt(targetX)}" y="${fmt(targetY)}" width="${fmt(targetW)}" height="${fmt(targetH)}" rx="3" />
    <text class="cloth-2d-target-label" x="${fmt(ox)}" y="${fmt(targetY - 8)}" text-anchor="middle">FOLDED GARMENT TARGET</text>

    <!-- Fold Crease Lines -->
    <line class="fold-line" x1="${fmt(leftX)}" y1="${fmt(oy - 55)}" x2="${fmt(leftX)}" y2="${fmt(oy + 10)}" />
    <line class="fold-line" x1="${fmt(rightX)}" y1="${fmt(oy - 55)}" x2="${fmt(rightX)}" y2="${fmt(oy + 10)}" />
    <line class="fold-line" x1="${fmt(leftX - 10)}" y1="${fmt(waistY)}" x2="${fmt(rightX + 10)}" y2="${fmt(waistY)}" />

    <!-- Guided fold trajectory arrows -->
    <path class="fold-arrow" d="M${fmt(a1From[0])} ${fmt(a1From[1])} Q${fmt((a1From[0] + a1To[0]) / 2)} ${fmt(a1From[1] - 22)} ${fmt(a1To[0])} ${fmt(a1To[1])}" marker-end="url(#fold-arrowhead)" />
    <path class="fold-arrow" d="M${fmt(a2From[0])} ${fmt(a2From[1])} Q${fmt((a2From[0] + a2To[0]) / 2)} ${fmt(a2From[1] - 22)} ${fmt(a2To[0])} ${fmt(a2To[1])}" marker-end="url(#fold-arrowhead)" />
    <path class="fold-arrow" d="M${fmt(a3FromL[0])} ${fmt(a3FromL[1])} Q${fmt(a3FromL[0] - 20)} ${fmt((a3FromL[1] + a3ToL[1]) / 2)} ${fmt(a3ToL[0])} ${fmt(a3ToL[1])}" marker-end="url(#fold-arrowhead)" />
    <path class="fold-arrow" d="M${fmt(a3FromR[0])} ${fmt(a3FromR[1])} Q${fmt(a3FromR[0] + 20)} ${fmt((a3FromR[1] + a3ToR[1]) / 2)} ${fmt(a3ToR[0])} ${fmt(a3ToR[1])}" marker-end="url(#fold-arrowhead)" />

    <!-- Landmark Grip Rings -->
    <g class="fold-mark ${aHeldSleeve ? 'held' : 'pending'}" data-arm="A">
      <circle cx="${fmt(a1From[0])}" cy="${fmt(a1From[1])}" r="8" />
      <text x="${fmt(a1From[0])}" y="${fmt(a1From[1] - 12)}" text-anchor="middle">A sleeve</text>
    </g>
    <g class="fold-mark ${bHeldSleeve ? 'held' : 'pending'}" data-arm="B">
      <circle cx="${fmt(a2From[0])}" cy="${fmt(a2From[1])}" r="8" />
      <text x="${fmt(a2From[0])}" y="${fmt(a2From[1] - 12)}" text-anchor="middle">B sleeve</text>
    </g>
    <g class="fold-mark ${aHeldHem ? 'held' : 'pending'}" data-arm="A">
      <circle cx="${fmt(a3FromL[0])}" cy="${fmt(a3FromL[1])}" r="7" />
      <text x="${fmt(a3FromL[0])}" y="${fmt(a3FromL[1] + 16)}" text-anchor="middle">A hem</text>
    </g>
    <g class="fold-mark ${bHeldHem ? 'held' : 'pending'}" data-arm="B">
      <circle cx="${fmt(a3FromR[0])}" cy="${fmt(a3FromR[1])}" r="7" />
      <text x="${fmt(a3FromR[0])}" y="${fmt(a3FromR[1] + 16)}" text-anchor="middle">B hem</text>
    </g>

    <text class="fold-stage" x="96" y="112">${status}</text>
  `;
}
