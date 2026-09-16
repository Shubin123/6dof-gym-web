/**
 * Optional three.js viewport for the lab stage.
 *
 * The 2-D SVG scene is the source of truth: this module is a spatial rendering
 * of exactly the same planar chain, so the top-down footprint of the 3-D arm
 * matches the SVG pixel-for-pixel. Links are stacked at descending heights the
 * way a real SCARA-style arm is built, which reads as depth without inventing
 * kinematics the simulation does not have.
 *
 * Loaded on demand — three.js only enters the network path when an operator
 * actually switches the viewport to 3-D.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { ARM, forwardKinematics } from './core.js';

const PX = 100; // scene pixels per world unit
const LINK_HEIGHTS = [0.78, 0.72, 0.66, 0.6, 0.54, 0.48, 0.42];
const COLORS = {
  floor: 0x131925,
  grid: 0x2b3648,
  workspace: 0x1b2433,
  link: 0xe9e5dc,
  linkAlt: 0xc9fb5d,
  joint: 0x3d4a60,
  base: 0x222d40,
  goal: 0xffbd4a,
  object: 0xc9fb5d,
};

const toWorld = ([x, y], height = 0) => new THREE.Vector3((x - ARM.base[0]) / PX, height, (y - ARM.base[1]) / PX);

function makeWorkspaceFloor(workspace) {
  const [minX, maxX, minY, maxY] = workspace.goal_workspace;
  const group = new THREE.Group();

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
  pad.position.copy(toWorld([(minX + maxX) / 2, (minY + maxY) / 2], 0.002));
  pad.receiveShadow = true;
  group.add(pad);

  const outline = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.PlaneGeometry((maxX - minX) / PX, (maxY - minY) / PX)),
    new THREE.LineBasicMaterial({ color: COLORS.linkAlt, transparent: true, opacity: 0.55 }),
  );
  outline.rotation.x = -Math.PI / 2;
  outline.position.copy(toWorld([(minX + maxX) / 2, (minY + maxY) / 2], 0.004));
  group.add(outline);

  const reach = new THREE.Mesh(
    new THREE.RingGeometry(workspace.max_reach_px / PX - 0.02, workspace.max_reach_px / PX, 96),
    new THREE.MeshBasicMaterial({ color: COLORS.linkAlt, transparent: true, opacity: 0.25, side: THREE.DoubleSide }),
  );
  reach.rotation.x = -Math.PI / 2;
  reach.position.y = 0.006;
  group.add(reach);

  return group;
}

function makeArm() {
  const group = new THREE.Group();
  const linkMaterial = new THREE.MeshStandardMaterial({ color: COLORS.link, roughness: 0.45, metalness: 0.15 });
  const accentMaterial = new THREE.MeshStandardMaterial({ color: COLORS.linkAlt, roughness: 0.4, metalness: 0.1 });
  const jointMaterial = new THREE.MeshStandardMaterial({ color: COLORS.joint, roughness: 0.6, metalness: 0.3 });

  const pedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.34, 0.42, 0.78, 32), new THREE.MeshStandardMaterial({ color: COLORS.base, roughness: 0.7, metalness: 0.25 }));
  pedestal.position.y = 0.39;
  pedestal.castShadow = true;
  pedestal.receiveShadow = true;
  group.add(pedestal);

  const links = [];
  const joints = [];
  for (let i = 0; i < ARM.lengths.length; i += 1) {
    const thickness = 0.2 - i * 0.017;
    const link = new THREE.Mesh(new THREE.BoxGeometry(1, thickness, thickness * 1.15), i % 2 ? accentMaterial : linkMaterial);
    link.castShadow = true;
    link.receiveShadow = true;
    group.add(link);
    links.push(link);

    const joint = new THREE.Mesh(new THREE.CylinderGeometry(thickness * 0.72, thickness * 0.72, 0.1 + thickness, 20), jointMaterial);
    joint.castShadow = true;
    group.add(joint);
    joints.push(joint);
  }

  const tool = new THREE.Group();
  const fingerMaterial = new THREE.MeshStandardMaterial({ color: COLORS.goal, roughness: 0.4, metalness: 0.2 });
  const fingers = [-1, 1].map((side) => {
    const finger = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.05, 0.05), fingerMaterial);
    finger.position.set(0.11, 0, side * 0.09);
    finger.castShadow = true;
    return finger;
  });
  const wrist = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.16, 16), jointMaterial);
  wrist.rotation.z = Math.PI / 2;
  tool.add(wrist, ...fingers);
  group.add(tool);

  return { group, links, joints, tool, fingers };
}

function makeGoal() {
  const group = new THREE.Group();
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.23, 0.022, 12, 48),
    new THREE.MeshStandardMaterial({ color: COLORS.goal, emissive: COLORS.goal, emissiveIntensity: 0.6, roughness: 0.4 }),
  );
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = 0.01;
  group.add(ring);

  const beam = new THREE.Mesh(
    new THREE.CylinderGeometry(0.012, 0.012, 1.2, 8),
    new THREE.MeshBasicMaterial({ color: COLORS.goal, transparent: true, opacity: 0.35 }),
  );
  beam.position.y = 0.6;
  group.add(beam);

  const cube = new THREE.Mesh(
    new THREE.BoxGeometry(0.24, 0.24, 0.24),
    new THREE.MeshStandardMaterial({ color: COLORS.object, roughness: 0.35, metalness: 0.1 }),
  );
  cube.position.y = 0.13;
  cube.castShadow = true;
  group.add(cube);

  return { group, cube };
}

export function createViewport3D(container, { workspace, onGoalPick }) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.domElement.classList.add('viewport-3d');
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0e131d);
  scene.fog = new THREE.Fog(0x0e131d, 12, 26);

  const camera = new THREE.PerspectiveCamera(42, 16 / 10, 0.1, 100);
  camera.position.set(-3.6, 5.4, 6.4);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 2.5;
  controls.maxDistance = 16;
  controls.maxPolarAngle = Math.PI / 2 - 0.05;
  controls.target.set(1.6, 0.4, -0.8);

  scene.add(new THREE.HemisphereLight(0xd8e4ff, 0x0b0f17, 1.1));
  const key = new THREE.DirectionalLight(0xffffff, 2.1);
  key.position.set(4, 8, 4);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  key.shadow.camera.left = -8;
  key.shadow.camera.right = 8;
  key.shadow.camera.top = 8;
  key.shadow.camera.bottom = -8;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xc9fb5d, 0.5);
  rim.position.set(-5, 3, -4);
  scene.add(rim);

  scene.add(makeWorkspaceFloor(workspace));
  const arm = makeArm();
  scene.add(arm.group);
  const goal = makeGoal();
  scene.add(goal.group);

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const floorPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const hit = new THREE.Vector3();
  let pressedAt = null;

  const onPointerDown = (event) => { pressedAt = { x: event.clientX, y: event.clientY }; };
  const onPointerUp = (event) => {
    if (!pressedAt || !onGoalPick) return;
    const dragged = Math.hypot(event.clientX - pressedAt.x, event.clientY - pressedAt.y) > 6;
    pressedAt = null;
    if (dragged) return;
    const rect = renderer.domElement.getBoundingClientRect();
    pointer.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
    raycaster.setFromCamera(pointer, camera);
    if (!raycaster.ray.intersectPlane(floorPlane, hit)) return;
    onGoalPick([hit.x * PX + ARM.base[0], hit.z * PX + ARM.base[1]]);
  };
  renderer.domElement.addEventListener('pointerdown', onPointerDown);
  renderer.domElement.addEventListener('pointerup', onPointerUp);

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

  let current = { q: [...ARM.lengths].map(() => 0), goal: [...ARM.base] };
  let frame = null;

  function layout() {
    const { points, angle } = forwardKinematics(current.q);
    for (let i = 0; i < arm.links.length; i += 1) {
      const from = toWorld(points[i], LINK_HEIGHTS[i]);
      const to = toWorld(points[i + 1], LINK_HEIGHTS[i + 1]);
      const link = arm.links[i];
      link.position.copy(from).lerp(to, 0.5);
      link.scale.x = Math.max(from.distanceTo(to), 0.001);
      link.rotation.y = -Math.atan2(to.z - from.z, to.x - from.x);

      const joint = arm.joints[i];
      joint.position.copy(from);
      joint.position.y = (LINK_HEIGHTS[i] + LINK_HEIGHTS[i + 1]) / 2;
      joint.scale.y = Math.max((LINK_HEIGHTS[i] - LINK_HEIGHTS[i + 1]) / 0.06, 1);
    }
    const tip = toWorld(points.at(-1), LINK_HEIGHTS.at(-1));
    arm.tool.position.copy(tip);
    arm.tool.rotation.y = -angle;

    const goalPoint = toWorld(current.goal, 0);
    goal.group.position.set(goalPoint.x, 0, goalPoint.z);
    goal.cube.rotation.y += 0.004;
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
      renderer.domElement.removeEventListener('pointerup', onPointerUp);
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
