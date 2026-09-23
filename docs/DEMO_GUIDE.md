# ArmLab demo guide

ArmLab is a browser-only simulation for checking task geometry, recording the interaction, and replaying it. It does not control a physical robot.

## Start a demo

1. Run `npm install` once, then `npm run dev` and open the local URL Vite prints.
2. Choose a scenario in the **Task scenario** menu, or select an example below the lab and choose **Load in lab**.
3. In the 2-D view, click the workspace to move the closest arm's goal. The goal is clamped to the configured table and reach envelope.
4. Use the joint sliders for manual poses. A proposed move is rejected if it would cross the floor, workspace edge, or the other arm's clearance zone.
5. Select **Run demo policy** to follow a prevalidated geometric path. Select it again while it is running to halt safely. The status line always reports whether the run reached the goal, reached its step budget, met a safety boundary, or was stopped by the operator.

Use **Reset** for a full scenario reset: it restores home poses, both task goal cubes and their declared heights, Arm A as the active controller, the cloth/timeline state, and fresh collision-checked IK plans. It does not leave a manually dragged cube or scrubbed frame as the base-policy start state.

Use **Step budget** to set the run limit from 1 to 400 steps. Each task initially selects its recommended horizon; Towel fold starts at 200. Increasing the budget lets a run continue longer, but never bypasses the safety checks.

Before free-space motion begins, the planner uses two stages: it first finds a safe route, then a reducer retests longer shortcuts at the normal per-joint step cap. A shortcut is kept only if every resampled frame clears the floor, table boundary, and other arm. Towel fold keeps its full pin, lift, cross, placement, and cloth-settling frames because reducing the dynamic phases would stretch the cloth unrealistically.

When a policy starts, the status line immediately changes to **Planning · testing safe routes** and the progress bar animates as the solver tries inverse-kinematics seeds and collision-checked routes. This is intentionally indeterminate: no path length is known until a candidate has passed safety checks. Select **Cancel solver** to stop before any arm motion. Once planning succeeds, the bar becomes the exact control-step progress indicator.

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

**Towel fold**, **Table reset**, and **Safe handoff** use both arms. The **Arm A / Arm B** switch selects which joint sliders and Goal Z control operate. The planner evaluates both safe arm orders and maintains the configured inter-arm clearance throughout the path.

Scenario 07, **Towel fold**, also labels its task-specific browser controller and displays a gripper-constrained spring cloth in 3-D. The towel falls to the table and only captures a corner when a tool reaches it; it is not animated by policy progress alone. The **Cloth frames** indicator retains and analyzes a rolling 120-frame mesh history, reporting which corners are held and whether the cloth has settled. This browser model does not include cloth self-collision, friction, or material-specific dynamics, so it is a useful interaction illustration rather than a physical-cloth claim. Real training needs demonstrations, observations, and an external training runtime.

## Recording and replay

1. Open the **Episode** tab and select **Record**.
2. Move a goal, adjust joints, or run the demo policy.
3. Select **Stop recording**, then choose **Replay episode** or **Download recording**.

Episodes are JSON previews with the task metadata, observations, safety-clamped actions, and optional voice fields. Import is limited to 8 MB. Voice capture is limited to 60 seconds and 5 MB; microphone and speech transcription depend on browser permissions and support. Audio can still be recorded when transcription is unavailable.

## Troubleshooting

- **The policy halts at a safety boundary:** choose **Reset**, then retry. If it repeats after moving a goal, pick a location farther from the table edge and the other arm.
- **The policy reaches its step budget:** reset the pose and retry the scenario's default goal. The status is a bounded simulation result, not a background process that continues running.
- **A manual slider snaps back:** that requested joint change was unsafe. Use smaller adjustments or reset first.
- **3-D is unavailable or blank:** enable hardware acceleration/WebGL or use 2-D. The 2-D stage covers the same arm state and goal data.
- **Voice buttons report unavailable:** serve the app from Vite, HTTPS, or localhost and allow microphone permission. File URLs do not grant microphone access in most browsers.

## Verification for contributors

Run `npm test` for the task-contract, IK, safety, replay-envelope, and safe-path checks. Run `npm run build` before deployment. The Pages workflow builds and deploys `main`.
