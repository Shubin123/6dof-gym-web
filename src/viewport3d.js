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
import { ClothSimulator } from './cloth.js';

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

/** Spring-network cloth (see src/cloth.js) for the task-07 demonstration. */
function makeCloth() {
  const simulator = new ClothSimulator({ columns: 14, rows: 11, width: 1.5, height: 1.2 });
  const geometry = new THREE.PlaneGeometry(simulator.width, simulator.height, simulator.columns, simulator.rows);
  const material = new THREE.MeshStandardMaterial({
    color: 0x49c7e8,
    emissive: 0x0b4560,
    emissiveIntensity: 0.55,
    roughness: 0.88,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.94,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  const grid = new THREE.Mesh(
    geometry,
    new THREE.MeshBasicMaterial({ color: 0xe1f9ff, wireframe: true, transparent: true, opacity: 0.35 }),
  );
  const group = new THREE.Group();
  // The translucent rectangle is the observable success reference: the
  // right half of the towel should land over this left-side footprint.
  const foldTarget = new THREE.Mesh(
    new THREE.PlaneGeometry(simulator.width / 2, simulator.height),
    new THREE.MeshBasicMaterial({ color: COLORS.linkAlt, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false }),
  );
  foldTarget.position.set(-simulator.width / 4, 0, simulator.thickness * 1.5);
  group.add(foldTarget);
  group.rotation.x = -Math.PI / 2;
  group.position.copy(toWorld([325, 240, 4]));
  group.add(mesh, grid);
  group.visible = false;
  return { group, geometry, simulator, taskId: null };
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

  let current = { arms: [], taskId: null, policyProgress: 0, clothSnapshot: null };
  let frame = null;

  function layoutCloth() {
    cloth.group.visible = current.taskId === 'fold';
    if (!cloth.group.visible) {
      cloth.taskId = null;
      return;
    }
    if (cloth.taskId !== current.taskId) {
      cloth.taskId = current.taskId;
      cloth.simulator.reset();
    }

    // Physics is advanced in main.js at the accepted control rate. Rendering
    // only consumes its immutable snapshot, keeping both viewport modes and
    // the timeline scrubber aligned frame-for-frame.
    if (!current.clothSnapshot) return;
    cloth.simulator.restore(current.clothSnapshot);

    cloth.geometry.attributes.position.array.set(cloth.simulator.positions);
    cloth.geometry.attributes.position.needsUpdate = true;
    cloth.geometry.computeVertexNormals();
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
    restoreCloth(snapshot) {
      if (cloth?.simulator && snapshot) {
        cloth.simulator.restore(snapshot);
        cloth.geometry.attributes.position.array.set(cloth.simulator.positions);
        cloth.geometry.attributes.position.needsUpdate = true;
        cloth.geometry.computeVertexNormals();
      }
    },
    getClothSnapshot() {
      return cloth?.simulator?.snapshot();
    },
    resetCloth() {
      if (cloth?.simulator) {
        cloth.simulator.reset();
        cloth.geometry.attributes.position.array.set(cloth.simulator.positions);
        cloth.geometry.attributes.position.needsUpdate = true;
        cloth.geometry.computeVertexNormals();
      }
    },
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
