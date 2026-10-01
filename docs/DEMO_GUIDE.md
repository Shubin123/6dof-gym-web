# ArmLab demo guide

ArmLab is a browser-only simulation for checking task geometry, recording the interaction, and replaying it. It does not control a physical robot.

## Start a demo

1. Run `npm install` once, then `npm run dev` and open the local URL Vite prints.
2. Choose a scenario in the **Task scenario** menu, or select an example below the lab and choose **Load in lab**.
3. In the 2-D view, click the workspace to move the closest arm's goal. The goal is clamped to the configured table and reach envelope.
4. Use the joint sliders for manual poses. A proposed move is rejected if it would cross the floor, workspace edge, or the other arm's clearance zone.
5. Select **Run demo policy** to follow a prevalidated geometric path. Select it again while it is running to halt safely. The status line always reports whether the run reached the goal, reached its step budget, met a safety boundary, or was stopped by the operator.

Use **Reset** for a full scenario reset: it restores home poses, both task goal cubes and their declared heights, Arm A as the active controller, the cloth/timeline state, and fresh collision-checked IK plans. It does not leave a manually dragged cube or scrubbed frame as the base-policy start state.

Use **Step budget** to set the run limit from 1 to 500 steps. Each task initially selects its recommended horizon; Towel fold starts at 500 so its lift, release, cross-over, placement, and settle stages all complete - the plan itself runs about 431 steps, and Towel fold reaches its goal only once the whole plan (including the cloth solver's settle tail) has played out, not as soon as either gripper passes near its starting grasp point. Increasing the budget lets a run continue longer, but never bypasses the safety checks.

Before free-space motion begins, the planner uses two stages: it first finds a safe route, then a reducer retests longer shortcuts at the normal per-joint step cap. A shortcut is kept only if every resampled frame clears the floor, table boundary, and other arm. Towel fold keeps its full pin, lift, cross, placement, and cloth-settling frames because reducing the dynamic phases would stretch the cloth unrealistically; every stage in which a gripper holds cloth moves the tool along straight lines (see below).

When a policy starts, the status line immediately changes through **trying IK candidates**, **checking collision-safe route**, and (for free-space tasks) **reducing verified route**. The progress bar animates while planning because no path length is known until a candidate has passed safety checks. Select **Cancel solver** to stop before any arm motion. Once planning succeeds, the bar becomes the exact control-step progress indicator.

**Retry until goal** retries only after a step-budget halt, returning to the home pose for a fresh safe plan. It stops after a reached goal and never retries a safety halt. Scenario 07 turns it on by default.

## Safety policy

The browser validates the whole arm chain before accepting a move, not only the goal marker. Every manual joint input, replay frame, and policy frame must satisfy all four rules:

1. Every link stays at or above the floor plane.
2. Every link remains inside the marked table workspace.
3. Two active arms maintain the configured clearance from one another.
4. No joint changes by more than the configured per-frame delta.

The fourth rule prevents arm teleporting: a slider move is capped to one safe control increment, and a policy frame that attempts a larger jump is rejected before either arm state changes. A rejected policy frame halts at the last safe pose; the **Safety envelope** shows whether the cause was the floor, workspace edge, arm collision, or joint-step limit.

## 2-D and 3-D controls

| View | Controls |
| --- | --- |
| 2-D | Click the stage to move the goal for the closest base column. Use **Goal Z** to change height. |
| 3-D | Drag empty space to orbit, use the wheel or trackpad to zoom, click the floor to move a goal, and drag a goal cube vertically to change its height. Dashed cubic Bézier lines preview tool motion; the arms themselves still execute only collision-checked, joint-limited frames. IK replans when the drag is released. |

The 3-D viewport is loaded only after selecting **3D**. It needs a browser with WebGL enabled. If it cannot start, switch back to **2D**; all planning, recording, and replay controls remain available there.

## Two-arm scenarios

**Towel fold**, **Shirt multi-fold**, **Table reset**, and **Safe handoff** use both arms. The **Arm A / Arm B** switch selects which joint sliders and Goal Z control operate. The planner evaluates both safe arm orders and maintains the configured inter-arm clearance throughout the path.

Scenario 07, **Towel fold**, displays a gripper-constrained spring cloth in both views and an outlined **FOLDED TARGET**. In place of goal cubes it shows a fold guide: a dashed **FOLD LINE**, a ring on the corner each arm takes, an arrow arcing from Arm B's corner to where it is laid, and a caption naming the current step (1 · grasp both corners, 2 · A pins while B lifts, 3 · A lets go while B places, Folded). The step is read from the cloth itself, so it stays true during manual runs and timeline scrubbing. Both tools approach above their corners, descend straight onto them and close; each closed gripper is drawn with its jaws shut on the corner. Arm B then carries its corner along an arc over the fold line while Arm A pins its own, Arm A opens and lifts straight off (the corner stays pinned exactly where it was held), and Arm B lays its corner on Arm A's. The towel stays on its footprint throughout, with layer thickness preserved and a rolling 120-frame mesh history for the loading slider. Both views consume the same control-rate cloth snapshots, so changing viewport cannot change the simulated fold.

Scenario 17, **Shirt multi-fold**, introduces complex non-rectangular garment handling under the laundry handling section. Rather than a simple flat rectangle, it models a structured T-shirt with collar, flank cutouts, torso, and sleeves across regional anatomical tags (`sleeve_left`, `sleeve_right`, `torso_upper`, `torso_lower`). The bimanual folding policy orchestrates a sequential multi-fold: Arm A folds the left sleeve inward across the torso midline, Arm B folds the right sleeve inward across the left sleeve, and then both arms grasp the bottom hem corners together, carrying them in a parallel arc over the waist crease line to the collar. Multi-layer collision separation and high garment Coulomb friction prevent premature unfolding or weave interpenetration. Both 2-D SVG and 3-D Three.js viewports render the garment outline, sleeve quads, and crease lines.

## Auto-propagation scenario

Scenario 18, **Auto-propagate arm**, introduces modular robotic self-replication under the **Auto propagate** category. The active robot arm retrieves modular industrial sub-assemblies (`arm_base` shoulder turret, `arm_link` articulated boom, and `arm_tool` wrist & gripper) from the parts depot tray and performs a mechanical installation of a full 6-DOF sibling robot arm atop an identical base mounting podium.

- **Repositionable mounting podium:** Before running the policy, the user can click anywhere on the table in the 2-D or 3-D viewport to reposition the sibling arm's mounting podium. The podium matches the exact dimensions, materials, and geometry of the primary arm's base column (radius top 0.3 m / 30 mm, radius bottom 0.42 m / 42 mm, height 0.9 m / 90 mm, dark metallic `#222d40` finish, 8-bolt perimeter flange, and floor collar). The target location is interactively clamped to the verified build area (below).
- **Realistic 6-DOF robotic installation:** Rather than a toy block stack, the policy carries out a true multi-stage mechanical assembly of an articulated manipulator:
  1. **Shoulder Turret docking:** Retrieves the cast shoulder turret with rotary turntable and clevis horns (`arm_base`) from the staging depot and seats it into the podium interface flange.
  2. **Articulated Boom coupling:** Transports the heavy upper arm boom and elbow knuckle (`arm_link`) and couples it into the shoulder clevis.
  3. **Wrist & Gripper locking:** Mounts the 3-axis wrist and dual-finger parallel gripper toolhead (`arm_tool`) onto the forearm interface flange.
  4. **Commissioning & calibration sweep:** The primary arm retracts safely to home pose. The newly assembled 6-DOF sibling arm initiates a power-on self-test (POST), executing a live joint calibration sweep (J1 azimuth checkout, J2/J3 unpark, gripper jaw stroke cycle) before settling into an active online ready pose (`SIBLING_ARM_READY_POSE`).
- **Physical constraints:** The podium is a real 90 mm column, so the modules are seated on its top face (z = 105 / 133 / 157 mm) rather than on the table. Every frame of both arms is checked against the task's `rigid.constraints` block, on top of the usual floor, workspace, rate and inter-arm checks:
  - **Bend limit** (`bend_limit_rad`, default 1.65 rad, a margin inside the 1.7 rad hard stop; a number or one entry per joint): no joint may fold its two links past this angle. The planners search only inside it.
  - **Limb interference** (`limb_interference`): the arm has a real body (`ARM_BODY` / `armBodies` in `core.js`) laid out like a collaborative arm, and the 3-D view draws exactly the parts this check tests. Every joint is a drum housing on its rotation axis. The link coming in bolts onto one end of the drum and the link going out onto the other, offset along the axis (10.5 → 7.5 px). Both links are perpendicular to that axis, so they lie in parallel planes further apart than their two radii plus 2 px, and they cannot touch however far the joint turns. J2 and J3 turn about parallel axes, so their sides alternate, as on a real elbow. The tool link stays in its plane, and a short flange bracket steps it back onto the tool's centre line, so the kinematics and the tool tip are unchanged. Every pair of parts that isn't bolted together is checked on every frame. Sweeping each joint through its full ±1.7 rad from several poses leaves at least 4 px between parts, and none of 20,000 random poses inside the limits interfere, so the body no longer caps any joint below its hard stop. The depot modules sit at x = 360 / 405 / 450, and the verified podium area is $x \in [440, 460], y \in [250, 300]$.
  - **Self-clearance** (`self_clearance_px`, 40): non-adjacent links of one arm must stay apart.
  - **Arm-to-podium** (`obstacle_clearance_px`, 6): no link may enter the sibling podium, the growing module stack on it, or the primary arm's own pedestal. An arm may stand on its own column but not fold into it.
  - **Singularity avoidance** (`singularity_margin`, 0.02): no arm may come near a singular posture, the configurations where some tool motion would need unbounded joint speed (stretched or folded elbow, aligned wrist axes, wrist over the base axis, and every mixed case). This chain has no spherical wrist, so rather than listing those cases by hand, the check uses the arm's full 6 × 6 tool Jacobian. Its position rows are divided by total reach so they are dimensionless, and the smallest singular value (`jacobianSingularValues` in `core.js`) must stay at or above the margin. For scale, the home pose sits at 0.066 and a straight arm at 0. Tool-down poses at the far end of the old depot came within 0.004 of singular, so the depot tray was widened and its modules re-spaced (see limb interference above for their final slots). That spacing also stops the open fingers clipping a neighbouring module.
  - **Arm-to-arm:** the sibling's calibration sweep (J1 azimuth checkout either side of its park pose, then unpark into ready) is planned against the parked primary arm and keeps `arm_clearance_px` from it.

  Telemetry names the constraint that blocks a pose (bend, limb interference, self-collision, pedestal, podium, module stack, singularity). Placement is limited to the verified build area $x \in [440, 460], y \in [250, 300]$; closer to the primary arm, a tool-down carry would need joints folded past the bend limit.
- **Jerk / snap / crackle / pop weighted motion:** Both arms' paths are smoothed by `src/limb-dynamics.js` under a weighted cost on the third to sixth derivatives of the joint angles (`KINEMATIC_WEIGHTS`: jerk 1, snap 0.5, crackle 0.25, pop 0.125). A planned path already runs near the rate cap, so each segment between gripper changes is first re-timed. Where a module is held, or the open fingers are around one, the path must not leave its verified tool-down line, so the arm eases to half speed into each sharp turn instead of rounding it. Elsewhere, free-space corners are also rounded off. Every smoothed frame is re-checked against the rate cap, joint limits, tool tilt and every constraint above, and a segment that fails keeps its original frames. On the default site this cuts peak jerk 2.5× and RMS pop 8× for about 17 % more steps (budget 1400).
- **Per-limb interaction:** In the 3-D view, grab any link or joint of an arm and drag it. Only the joints up to the grabbed limb move toward the pointer, and the limbs beyond it ride along. The arm does not jump there: a `LimbController` eases it through a four-stage cascade (one stage per weighted derivative), one rate-capped tick per frame. Each tick must pass the task's full cell check. A limb pushed into a limit stops against it, and the safety line names the limit (bend limit, limbs touching, self-collision, arm pedestal, sibling podium, module stack, singularity, floor, table edge). Joint sliders use the same check.
- **Physics and telemetry:** Cannon-es rigid contact physics simulates authentic mechanical docking, seating stability, and multi-tier alignment. Real-time telemetry displays the mechanical assembly stage, replication progress, and positioning precision. Both 2-D SVG and 3-D WebGL viewports render the identical podium and full 6-DOF sibling arm kinematics.

## Task-specific policy recipes

`src/task-policies.js` is the extension point for a new use case. A recipe declares a label, a safe warm-start profile, and weighted intermediate rewards. Scenario 07 uses five stages: secure both corners, pin/release the left edge, cross the fold line, place on the target, and settle below the stretch threshold. The browser runs a deterministic synthetic cloth calibration before planning, but every generated frame still goes through IK, floor, workspace, inter-arm, and joint-step checks. It is a pseudo-training seam for task development—not a claim that the browser has trained a deployable neural policy.

To add a task-specific policy, add its recipe and optional profile in `src/task-policies.js`, then route the task planner to consume that profile. This keeps task optimisation local: tune the relevant stage weights and motion parameters without weakening the shared safety envelope.

## Recording and replay

1. Open the **Episode** tab and select **Record**.
2. Move a goal, adjust joints, or run the demo policy.
3. Select **Stop recording**, then choose **Replay episode** or **Download recording**.

Episodes are JSON previews with the task metadata, observations, safety-clamped actions, and optional voice fields. Import is limited to 8 MB. Voice capture is limited to 60 seconds and 5 MB; microphone and speech transcription depend on browser permissions and support. Audio can still be recorded when transcription is unavailable.

### Building a dataset

Below the single-episode controls, **Add to dataset** appends the current recording to an in-browser dataset instead of downloading it alone; the counter shows how many episodes are in it. **Download dataset** bundles them into one LeRobotDataset-shaped JSON (`info`/`tasks`/`episodes`/`frames`), and **Clear dataset** empties it. All episodes in one dataset must use the same arm count (single-arm tasks and bimanual tasks like Towel fold can't mix) - adding a mismatched episode is rejected with an explanation instead of silently corrupting the bundle. `scripts/lerobot_export.py` (see the [README](../README.md#exporting-a-demonstration-dataset)) turns the downloaded bundle into an actual LeRobotDataset directory with parquet files.

## Troubleshooting

- **The policy halts at a safety boundary:** choose **Reset**, then retry. If it repeats after moving a goal, pick a location farther from the table edge and the other arm.
- **The policy reaches its step budget:** reset the pose and retry the scenario's default goal. The status is a bounded simulation result, not a background process that continues running.
- **A manual slider snaps back:** that requested joint change was unsafe. Use smaller adjustments or reset first.
- **3-D is unavailable or blank:** enable hardware acceleration/WebGL or use 2-D. The 2-D stage covers the same arm state and goal data.
- **Voice buttons report unavailable:** serve the app from Vite, HTTPS, or localhost and allow microphone permission. File URLs do not grant microphone access in most browsers.

## Verification for contributors

Run `npm test` for the task-contract, IK, safety, replay-envelope, and safe-path checks. Run `npm run build` before deployment. The Pages workflow builds and deploys `main`.
