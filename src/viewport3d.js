/**
 * Optional three.js viewport for the lab stage.
 *
 * The 2-D SVG scene is the source of truth: this module is a spatial rendering
 * of exactly the same planar chain, so the top-down footprint of the 3-D arm
 * matches the SVG pixel-for-pixel. What the SVG cannot show is height, and
 * that is what this view adds — the prismatic tool axis and the object's
 * position above the table are real coordinates here, not a drop shadow.
 *
 * Loaded on demand — three.js only enters the network path when an operator
 * actually switches the viewport to 3-D.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { ARM, ARM_B, clamp, forwardKinematics, GOAL_Z } from './core.js';

const PX = 100; // scene pixels per world unit
const UP = new THREE.Vector3(0, 1, 0);
const FORWARD = new THREE.Vector3(1, 0, 0);
const COLORS = {
  floor: 0x131925,
  grid: 0x2b3648,
  workspace: 0x1b2433,
  link: 0xe9e5dc,
  linkAlt: 0xc9fb5d,
  linkB: 0x69d2df,
  joint: 0x3d4a60,
  base: 0x222d40,
  goal: 0xffbd4a,
  object: 0xc9fb5d,
};

/** Scene pixels to world units. The arm's z is a real height, not a decoration. */
const toWorld = ([x, y, z = 0], origin = ARM.base) => new THREE.Vector3((x - origin[0]) / PX, z / PX, (y - origin[1]) / PX);

function makeWorkspaceFloor(workspace) {
  const [minX, maxX, minY, maxY] = workspace.goal_workspace;
  const group = new THREE.Group();
  const origin = ARM.base;

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(24, 24),
    new THREE.MeshStandardMaterial({ color: COLORS.floor, roughness: 0.95, metalness: 0 }),
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -0.01;
  floor.receiveShadow = true;
  group.add(floor);

  const grid = new THREE.GridHelper(24, 48, COLORS.grid, COLORS.grid);
  grid.material.transparent = true;
  grid.material.opacity = 0.35;
  group.add(grid);

  const pad = new THREE.Mesh(
    new THREE.PlaneGeometry((maxX - minX) / PX, (maxY - minY) / PX),
    new THREE.MeshStandardMaterial({ color: COLORS.workspace, roughness: 0.8, transparent: true, opacity: 0.9 }),
  );
  pad.rotation.x = -Math.PI / 2;
  pad.position.copy(toWorld([(minX + maxX) / 2, (minY + maxY) / 2, 0.2], origin));
  pad.receiveShadow = true;
  group.add(pad);

  const outline = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.PlaneGeometry((maxX - minX) / PX, (maxY - minY) / PX)),
    new THREE.LineBasicMaterial({ color: COLORS.linkAlt, transparent: true, opacity: 0.55 }),
  );
  outline.rotation.x = -Math.PI / 2;
  outline.position.copy(toWorld([(minX + maxX) / 2, (minY + maxY) / 2, 0.4], origin));
  group.add(outline);

  for (const arm of [ARM, ARM_B]) {
    const reach = new THREE.Mesh(
      new THREE.RingGeometry(workspace.max_reach_px / PX - 0.02, workspace.max_reach_px / PX, 96),
      new THREE.MeshBasicMaterial({ color: arm.mirror ? COLORS.linkB : COLORS.linkAlt, transparent: true, opacity: 0.2, side: THREE.DoubleSide }),
    );
    reach.rotation.x = -Math.PI / 2;
    reach.position.copy(toWorld([...arm.base, 0.6]));
    group.add(reach);
  }

  return group;
}

function makeArm(arm) {
  const group = new THREE.Group();
  const accent = arm.mirror ? COLORS.linkB : COLORS.linkAlt;
  const linkMaterial = new THREE.MeshStandardMaterial({ color: COLORS.link, roughness: 0.45, metalness: 0.15 });
  const accentMaterial = new THREE.MeshStandardMaterial({ color: accent, roughness: 0.4, metalness: 0.1 });
  const jointMaterial = new THREE.MeshStandardMaterial({ color: COLORS.joint, roughness: 0.6, metalness: 0.3 });

  const column = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.42, arm.baseHeight / PX, 32), new THREE.MeshStandardMaterial({ color: COLORS.base, roughness: 0.7, metalness: 0.25 }));
  column.position.copy(toWorld([...arm.base, arm.baseHeight / 2]));
  column.castShadow = true;
  column.receiveShadow = true;
  group.add(column);

  const links = [];
  const joints = [];
  for (let i = 0; i < arm.lengths.length; i += 1) {
    const thickness = 0.2 - i * 0.017;
    // A unit-length bar along +Y, rotated onto each 3-D segment at layout time.
    const link = new THREE.Mesh(new THREE.BoxGeometry(thickness, 1, thickness * 1.15), i % 2 ? accentMaterial : linkMaterial);
    link.castShadow = true;
    link.receiveShadow = true;
    group.add(link);
    links.push(link);

    const joint = new THREE.Mesh(new THREE.SphereGeometry(thickness * 0.66, 18, 12), jointMaterial);
    joint.castShadow = true;
    group.add(joint);
    joints.push(joint);
  }

  const tool = new THREE.Group();
  const fingerMaterial = new THREE.MeshStandardMaterial({ color: COLORS.goal, roughness: 0.4, metalness: 0.2 });
  [-1, 1].forEach((side) => {
    const finger = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.05, 0.05), fingerMaterial);
    finger.position.set(0.11, 0, side * 0.09);
    finger.castShadow = true;
    tool.add(finger);
  });
  const wrist = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.16, 16), jointMaterial);
  wrist.rotation.z = Math.PI / 2;
  tool.add(wrist);
  group.add(tool);

  return { arm, group, links, joints, tool, column };
}

function makeGoal(mirrored) {
  const group = new THREE.Group();
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.23, 0.022, 12, 48),
    new THREE.MeshStandardMaterial({ color: COLORS.goal, emissive: COLORS.goal, emissiveIntensity: 0.6, roughness: 0.4 }),
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = 0.01;
  group.add(ring);

  // A drop line from the floor marker up to the object, so its height is readable.
  const beam = new THREE.Mesh(
    new THREE.CylinderGeometry(0.012, 0.012, 1, 8),
    new THREE.MeshBasicMaterial({ color: COLORS.goal, transparent: true, opacity: 0.4 }),
  );
  group.add(beam);

  const cube = new THREE.Mesh(
    new THREE.BoxGeometry(0.24, 0.24, 0.24),
    new THREE.MeshStandardMaterial({ color: mirrored ? COLORS.linkB : COLORS.object, roughness: 0.35, metalness: 0.1 }),
  );
  cube.castShadow = true;
  group.add(cube);

  // Up/down arrows above and below the cube: the affordance that says the
  // object's height is something you can take hold of.
  const handleMaterial = new THREE.MeshBasicMaterial({ color: COLORS.goal, transparent: true, opacity: 0 });
  const handles = [1, -1].map((side) => {
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.07, 0.14, 14), handleMaterial);
    cone.rotation.x = side > 0 ? 0 : Math.PI;
    cone.position.y = side * 0.24;
    return cone;
  });
  group.add(...handles);

  // A generous invisible cylinder so the cube is easy to grab on a small screen.
  const grip = new THREE.Mesh(
    new THREE.CylinderGeometry(0.2, 0.2, 0.62, 10),
    new THREE.MeshBasicMaterial({ visible: false }),
  );
  group.add(grip);

  return { group, cube, beam, grip, handles, handleMaterial };
}

/** A small, gripper-constrained spring cloth for the task-07 demonstration. */
function makeCloth() {
  const columns = 12;
  const rows = 10;
  const geometry = new THREE.PlaneGeometry(1.5, 1.2, columns, rows);
  const rest = Float32Array.from(geometry.attributes.position.array);
  const velocity = new Float32Array(rest.length);
  const material = new THREE.MeshStandardMaterial({
    color: 0x49c7e8,
    emissive: 0x0b4560,
    emissiveIntensity: 0.55,
    roughness: 0.92,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.94,
  });
  const mesh = new THREE.Mesh(geometry, material);
  const grid = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ color: 0xe1f9ff, wireframe: true, transparent: true, opacity: 0.45 }));
  const group = new THREE.Group();
  // Plane geometry starts in XY; local Z is the physical lift after rotation.
  group.rotation.x = -Math.PI / 2;
  group.position.copy(toWorld([325, 240, 4]));
  group.add(mesh, grid);
  group.visible = false;
  return { group, geometry, rest, velocity, columns, rows, captured: [false, false], taskId: null, frames: [], settled: false };
}

export function createViewport3D(container, { workspace, onGoalPick, onGoalHeight, onClothFrame }) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.domElement.classList.add('viewport-3d');
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0e131d);
  scene.fog = new THREE.Fog(0x0e131d, 14, 28);

  const camera = new THREE.PerspectiveCamera(42, 16 / 10, 0.1, 100);
  camera.position.set(-2.4, 5.6, 6.8);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 2.5;
  controls.maxDistance = 18;
  controls.maxPolarAngle = Math.PI / 2 - 0.05;
  controls.target.set(1.8, 0.5, -0.9);

  scene.add(new THREE.HemisphereLight(0xd8e4ff, 0x0b0f17, 1.1));
  const key = new THREE.DirectionalLight(0xffffff, 2.1);
  key.position.set(4, 9, 4);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  key.shadow.camera.left = -9;
  key.shadow.camera.right = 9;
  key.shadow.camera.top = 9;
  key.shadow.camera.bottom = -9;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xc9fb5d, 0.5);
  rim.position.set(-5, 3, -4);
  scene.add(rim);

  scene.add(makeWorkspaceFloor(workspace));

  // Both arms and both goals exist up front; a single-arm task simply hides the second.
  const rigs = [ARM, ARM_B].map((arm) => {
    const rig = makeArm(arm);
    const goal = makeGoal(arm.mirror);
    scene.add(rig.group, goal.group);
    return { ...rig, goal };
  });
  const cloth = makeCloth();
  scene.add(cloth.group);
  const motionLines = rigs.map((rig) => {
    const geometry = new THREE.BufferGeometry();
    const line = new THREE.Line(geometry, new THREE.LineDashedMaterial({ color: rig.arm.mirror ? COLORS.linkB : COLORS.linkAlt, dashSize: 0.12, gapSize: 0.08, transparent: true, opacity: 0.8 }));
    line.visible = false;
    scene.add(line);
    return line;
  });

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const floorPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const dragPlane = new THREE.Plane();
  const hit = new THREE.Vector3();
  let pressedAt = null;
  let hovered = null;
  let drag = null;

  const setPointer = (event) => {
    const rect = renderer.domElement.getBoundingClientRect();
    pointer.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
    raycaster.setFromCamera(pointer, camera);
  };

  /** The visible goal object under the pointer, if any. */
  const pickGoal = (event) => {
    setPointer(event);
    const grips = rigs.filter((rig) => rig.goal.group.visible).map((rig) => rig.goal.grip);
    const [first] = raycaster.intersectObjects(grips, false);
    return first ? rigs.find((rig) => rig.goal.grip === first.object) : null;
  };

  /**
   * Stand a plane up through the goal, facing the camera.
   *
   * Dragging then reads height straight off the ray/plane intersection, so the
   * cube tracks the pointer exactly instead of drifting as the camera orbits.
   */
  const facingPlane = (origin) => {
    const normal = new THREE.Vector3().subVectors(camera.position, origin);
    normal.y = 0;
    if (normal.lengthSq() < 1e-6) normal.set(0, 0, 1);
    dragPlane.setFromNormalAndCoplanarPoint(normal.normalize(), origin);
    return dragPlane;
  };

  const setHovered = (rig) => {
    if (hovered === rig) return;
    if (hovered) hovered.goal.handleMaterial.opacity = 0;
    hovered = rig;
    if (hovered) hovered.goal.handleMaterial.opacity = 0.9;
    renderer.domElement.style.cursor = rig ? 'ns-resize' : '';
  };

  const onPointerDown = (event) => {
    pressedAt = { x: event.clientX, y: event.clientY };
    const rig = pickGoal(event);
    if (!rig || !onGoalHeight) return;
    const origin = rig.goal.cube.getWorldPosition(new THREE.Vector3());
    if (!raycaster.ray.intersectPlane(facingPlane(origin), hit)) return;
    // Grab the cube where it was actually clicked, so it does not jump.
    drag = { rig, offset: hit.y - origin.y };
    controls.enabled = false;
    setHovered(rig);
    renderer.domElement.setPointerCapture(event.pointerId);
  };

  const onPointerMove = (event) => {
    if (!drag) {
      setHovered(pickGoal(event));
      return;
    }
    setPointer(event);
    const origin = drag.rig.goal.cube.getWorldPosition(new THREE.Vector3());
    if (!raycaster.ray.intersectPlane(facingPlane(origin), hit)) return;
    const height = clamp((hit.y - drag.offset) * PX, GOAL_Z.min, GOAL_Z.max);
    onGoalHeight(drag.rig.arm.id, height, { committed: false });
  };

  const endDrag = (event) => {
    if (!drag) return false;
    const { rig } = drag;
    drag = null;
    controls.enabled = true;
    if (renderer.domElement.hasPointerCapture?.(event.pointerId)) renderer.domElement.releasePointerCapture(event.pointerId);
    // Replanning is deferred to the release: solving on every pointer move
    // would put a full IK search inside the drag loop.
    onGoalHeight(rig.arm.id, null, { committed: true });
    return true;
  };

  const onPointerUp = (event) => {
    const wasDragging = endDrag(event);
    const start = pressedAt;
    pressedAt = null;
    if (wasDragging || !start || !onGoalPick) return;
    if (Math.hypot(event.clientX - start.x, event.clientY - start.y) > 6) return;
    setPointer(event);
    if (!raycaster.ray.intersectPlane(floorPlane, hit)) return;
    onGoalPick([hit.x * PX + ARM.base[0], hit.z * PX + ARM.base[1]]);
  };

  const onPointerLeave = () => { if (!drag) setHovered(null); };

  renderer.domElement.addEventListener('pointerdown', onPointerDown);
  renderer.domElement.addEventListener('pointermove', onPointerMove);
  renderer.domElement.addEventListener('pointerup', onPointerUp);
  renderer.domElement.addEventListener('pointercancel', onPointerUp);
  renderer.domElement.addEventListener('pointerleave', onPointerLeave);

  const resize = () => {
    const width = container.clientWidth;
    const height = container.clientHeight;
    if (!width || !height) return;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  };
  const observer = new ResizeObserver(resize);
  observer.observe(container);
  resize();

  let current = { arms: [], taskId: null, policyProgress: 0 };
  let frame = null;

  function layoutCloth() {
    cloth.group.visible = current.taskId === 'fold';
    if (!cloth.group.visible) {
      cloth.taskId = null;
      return;
    }
    if (cloth.taskId !== current.taskId) {
      cloth.taskId = current.taskId;
      cloth.captured = [false, false];
      cloth.velocity.fill(0);
      cloth.frames = [];
      cloth.settled = false;
      cloth.geometry.attributes.position.array.set(cloth.rest);
    }

    // The two front corners can only be captured when an actual tool reaches
    // them. No progress-based shape morphing: the mesh moves from gravity,
    // springs, a table plane, and those explicit gripper constraints.
    const width = cloth.columns + 1;
    const anchors = [0, cloth.columns];
    const targets = [rigs[0]?.tool, rigs[1]?.tool].map((tool) => tool?.getWorldPosition(new THREE.Vector3()));
    const positions = cloth.geometry.attributes.position.array;
    const localTargets = targets.map((target) => target ? cloth.group.worldToLocal(target.clone()) : null);
    anchors.forEach((vertex, armIndex) => {
      if (!localTargets[armIndex]) return;
      const offset = vertex * 3;
      const distanceToCorner = Math.hypot(
        positions[offset] - localTargets[armIndex].x,
        positions[offset + 1] - localTargets[armIndex].y,
        positions[offset + 2] - localTargets[armIndex].z,
      );
      if (distanceToCorner < 0.36) cloth.captured[armIndex] = true;
    });

    for (let offset = 0; offset < positions.length; offset += 3) {
      const pinned = anchors.some((vertex, index) => vertex * 3 === offset && cloth.captured[index]);
      if (pinned) continue;
      cloth.velocity[offset + 2] = (cloth.velocity[offset + 2] - 0.0018) * 0.985;
      positions[offset + 2] = Math.max(0.018, positions[offset + 2] + cloth.velocity[offset + 2]);
    }

    const relax = (first, second) => {
      const a = first * 3; const b = second * 3;
      const dx = positions[b] - positions[a]; const dy = positions[b + 1] - positions[a + 1]; const dz = positions[b + 2] - positions[a + 2];
      const length = Math.hypot(dx, dy, dz) || 1;
      const rx = cloth.rest[b] - cloth.rest[a]; const ry = cloth.rest[b + 1] - cloth.rest[a + 1];
      const restLength = Math.hypot(rx, ry) || 1;
      const correction = (length - restLength) / length * 0.5;
      const firstPinned = anchors.some((vertex, index) => vertex === first && cloth.captured[index]);
      const secondPinned = anchors.some((vertex, index) => vertex === second && cloth.captured[index]);
      if (!firstPinned) { positions[a] += dx * correction; positions[a + 1] += dy * correction; positions[a + 2] += dz * correction; }
      if (!secondPinned) { positions[b] -= dx * correction; positions[b + 1] -= dy * correction; positions[b + 2] -= dz * correction; }
    };
    for (let iteration = 0; iteration < 4; iteration += 1) {
      for (let row = 0; row <= cloth.rows; row += 1) for (let column = 0; column <= cloth.columns; column += 1) {
        const vertex = row * width + column;
        if (column < cloth.columns) relax(vertex, vertex + 1);
        if (row < cloth.rows) relax(vertex, vertex + width);
      }
      anchors.forEach((vertex, armIndex) => {
        if (!cloth.captured[armIndex] || !localTargets[armIndex]) return;
        const offset = vertex * 3;
        positions[offset] = localTargets[armIndex].x;
        positions[offset + 1] = localTargets[armIndex].y;
        positions[offset + 2] = Math.max(0.018, localTargets[armIndex].z);
      });
    }
    cloth.geometry.attributes.position.needsUpdate = true;
    cloth.geometry.computeVertexNormals();
    // Keep a bounded frame buffer for observing actual cloth motion and
    // determine settling from the oldest/newest buffered mesh snapshots.
    cloth.frames.push(Float32Array.from(positions));
    if (cloth.frames.length > 120) cloth.frames.shift();
    if (cloth.frames.length === 120) {
      const oldest = cloth.frames[0];
      const newest = cloth.frames.at(-1);
      let displacement = 0;
      for (let index = 0; index < newest.length; index += 3) displacement += Math.hypot(newest[index] - oldest[index], newest[index + 1] - oldest[index + 1], newest[index + 2] - oldest[index + 2]);
      cloth.settled = displacement / (newest.length / 3) < 0.002;
    }
    onClothFrame?.({ frames: cloth.frames.length, captured: [...cloth.captured], settled: cloth.settled });
  }

  function layoutArm(rig, armState) {
    const { points, forward } = forwardKinematics(armState.q, rig.arm);
    const world = points.map((point) => toWorld(point));

    for (let i = 0; i < rig.links.length; i += 1) {
      const from = world[i];
      const to = world[i + 1];
      const link = rig.links[i];
      const span = new THREE.Vector3().subVectors(to, from);
      const length = Math.max(span.length(), 0.001);
      link.position.copy(from).lerp(to, 0.5);
      link.scale.y = length;
      link.quaternion.setFromUnitVectors(UP, span.normalize());
      rig.joints[i].position.copy(from);
    }

    const tip = world.at(-1);
    rig.tool.position.copy(tip);
    // Scene axes are (x, depth, height); the world's are (x, height, depth).
    rig.tool.quaternion.setFromUnitVectors(FORWARD, new THREE.Vector3(forward[0], forward[2], forward[1]).normalize());

    const goal = toWorld(armState.goal);
    rig.goal.group.position.set(goal.x, 0, goal.z);
    rig.goal.cube.position.y = goal.y;
    rig.goal.cube.rotation.y += 0.004;
    rig.goal.grip.position.y = goal.y;
    rig.goal.handles.forEach((handle, index) => { handle.position.y = goal.y + (index === 0 ? 0.24 : -0.24); });
    rig.goal.beam.scale.y = Math.max(goal.y, 0.001);
    rig.goal.beam.position.y = goal.y / 2;

    const controlA = tip.clone().lerp(goal, 0.35).add(new THREE.Vector3(0, 0.7, 0));
    const controlB = tip.clone().lerp(goal, 0.7).add(new THREE.Vector3(0, 0.7, 0));
    const curve = new THREE.CubicBezierCurve3(tip, controlA, controlB, goal);
    const motionLine = motionLines[rig.arm.id === 'B' ? 1 : 0];
    motionLine.geometry.setFromPoints(curve.getPoints(32));
    motionLine.computeLineDistances();
    motionLine.visible = true;
  }

  function layout() {
    rigs.forEach((rig, index) => {
      const armState = current.arms[index];
      const visible = Boolean(armState);
      rig.group.visible = visible;
      rig.goal.group.visible = visible;
      motionLines[index].visible = visible;
      if (visible) layoutArm(rig, armState);
    });
    layoutCloth();
  }

  function loop() {
    controls.update();
    layout();
    renderer.render(scene, camera);
    frame = requestAnimationFrame(loop);
  }

  return {
    element: renderer.domElement,
    update(next) { current = next; },
    start() { if (frame === null) { resize(); loop(); } },
    stop() { if (frame !== null) { cancelAnimationFrame(frame); frame = null; } },
    dispose() {
      this.stop();
      observer.disconnect();
      renderer.domElement.removeEventListener('pointerdown', onPointerDown);
      renderer.domElement.removeEventListener('pointermove', onPointerMove);
      renderer.domElement.removeEventListener('pointerup', onPointerUp);
      renderer.domElement.removeEventListener('pointercancel', onPointerUp);
      renderer.domElement.removeEventListener('pointerleave', onPointerLeave);
      controls.dispose();
      scene.traverse((node) => {
        if (node.geometry) node.geometry.dispose();
        if (node.material) (Array.isArray(node.material) ? node.material : [node.material]).forEach((material) => material.dispose());
      });
      renderer.dispose();
      renderer.domElement.remove();
    },
  };
}
