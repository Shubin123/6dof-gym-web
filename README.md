# ArmLab — 6‑DOF Gym

Prototype: [https://shubin123.github.io/6dof-gym-web/](https://shubin123.github.io/6dof-gym-web/)

A static, interactive web prototype for designing and validating 6-DOF arm tasks. It demonstrates a task contract, a library of fully specified examples, browser episode recording, curated source data, and an explicit policy-selection ladder.

## Run locally

```bash
npm install
npm run dev
```

Run the data-contract checks and make the production bundle with:

```bash
npm test
npm run build
```

For a hands-on walkthrough, control reference, episode format, and browser troubleshooting, see [the demo guide](docs/DEMO_GUIDE.md).

## What is real in this prototype

- The interactive arm, workspace bounds, task selection, bounded episode recording, replay, and downloads work entirely in the browser.
- Voice instructions can be captured, embedded in an episode JSON file, imported, and replayed. Native browser speech recognition is used when available; it is not a bundled or remotely loading speech model.
- `data/compiled.json` is the shared, versioned task/data metadata that drives the cards and test checks.
- The **Run demo policy** control is deliberately a transparent, safety-capped geometric controller for validating a task—not a claim of embedded VLA inference.

## Task examples

Eleven examples ship in `data/compiled.json`, grouped into six families: basics, object manipulation, surface cleaning, laundry handling, kitchen and table reset, and failure recovery. Each one is a complete specification rather than a title and a goal position:

| Field | What it carries |
| --- | --- |
| `summary`, `instruction`, `metric` | What the task is and how it is scored |
| `success` | The signal that ends an episode successfully |
| `sensors` | The minimum sensor set the task actually needs |
| `curriculum` | The ordered progression from first demonstration to full variation |
| `failure_modes` | What is known to go wrong, so evaluation looks for it |
| `guardrail` | The safety boundary that is not the policy's to decide |
| `difficulty`, `horizon_steps`, `baseline` | Planning metadata and the smallest policy worth trying first |
| `arms`, `goal`, `goal_b`, `goal_height` | One gripper or two, and where in space each one works |

Every example is reachable in the browser lab: `npm test` asserts that each goal lies inside the declared workspace and reach shell, is reached in three dimensions by both the solver and the tracking controller, and keeps a complete specification.

## Safety envelope

The browser controller validates the entire arm geometry, not only its goal marker. Manual moves, imported replays, and demo-policy frames are rejected before a link can pass below the floor, cross the marked table edge, or enter the other arm's configured clearance. Both manual moves and policy frames are capped to the configured per-joint delta, so the arm cannot teleport between safe poses. Bimanual policy runs use deterministic collision-checked paths and halt in an explicit safety state if no valid route exists.

The safety policy is: (1) no link below the floor plane, (2) no link beyond the table workspace, (3) no inter-arm clearance violation, and (4) no joint jump above the per-frame limit. The controller checks the whole next frame before changing any arm state; any violation leaves the current safe pose intact and halts the policy with the reason shown in the Safety envelope.

This remains a geometric browser simulation, not a certified collision system or hardware controller. Physical deployments still require robot-specific meshes, self-collision checks, torque and velocity limits, a watchdog, dead-man control, and an independent e-stop.

## The arm

The chain is a real spatial one. Each joint turns about an axis of its own moving frame — yaw, pitch, pitch, yaw, pitch, roll — so a pose has a genuine height and lateral offset, and the arm bends out of any single plane. Both viewports are views of that one chain.

Goals are points in space: `x`, `y`, and a height above the table that each task declares and the **Goal Z** slider changes. The solver takes yaw from geometry and searches the remaining joints with multi-start CCD; guidance then tracks the plan under a per-step joint-delta cap.

A goal is projected into the arm's reach shell before it is used. The shell has an inner wall as well as an outer one — an articulated arm cannot fold back to touch its own shoulder, and the higher a goal sits the further out that wall moves. The correction is horizontal, so raising a goal slides it away from the column instead of dropping it back onto the table.

### Two arms

Tasks that genuinely need two grippers declare `arms: 2` and a second goal: towel fold, safe handoff, and table reset. The right-hand arm is the same manipulator mirrored about its own base column, so one joint vector describes either posture and one solver serves both. Clicking the scene moves the goal of whichever arm's base column is nearest; the **Arm A / Arm B** switch chooses which arm the joint sliders drive. A bimanual episode records both arms — 44 observation features and 14 action dimensions instead of 22 and 7.

## Viewports

The lab stage renders the same arm two ways, switched by the **2D / 3D** control:

- **2D** — the top-down SVG scene. Click anywhere to move the nearest arm's goal. Height shows as a cast shadow, as joint scale, and as a readout, because a top-down view cannot show it directly.
- **3D** — a three.js viewport. Drag to orbit, scroll to zoom, click the floor to move a goal, and **drag a cube up or down to set its height**. Dashed cubic Bézier lines preview the tool motion between its current pose and goal; they are visual guides only, while the actual arm moves through safety-checked, joint-limited frames. This is where height is literal: the column, the arc of the elbow, and the object floating above its floor marker are all real coordinates.

Hovering a cube raises an arrow above and below it and switches the cursor; grabbing one takes hold of the point you clicked, so the object tracks the pointer without jumping, and the orbit camera stands still for the duration of the drag. Dragging a cube also makes that arm the one the joint sliders drive, and the **Goal Z** slider follows along. The solver runs once on release rather than on every pointer move, which keeps a full IK search out of the drag loop.

three.js is loaded on demand. It sits in its own lazy chunk, so the initial page load is unchanged for anyone who never opens the 3-D view, and a browser without working WebGL falls back to the 2-D scene with a message instead of a broken stage.

## Running a policy

The demo planner is a transparent geometric controller, and a run is bounded rather than open-ended:

- **Halt state.** Every run ends in a named state — *goal reached* with the step count, *step budget exhausted*, *safety boundary*, or *halted by operator* — so a success is never confused with a run that simply gave up. The run button becomes a halt button while a policy is moving.
- **Two-stage solver.** For free-space tasks, the first stage finds a collision-checked route and a second-stage reducer then tries longer shortcuts. It resamples every candidate under the same joint-delta cap and rejects it unless every frame stays above the floor, inside the workspace, and clear of the other arm. This reduces control steps without relaxing safety. Task 07 deliberately retains every grasp, lift, cross, placement, and settling frame: reducing those dynamic-cloth phases would make the fabric stretch unrealistically.
- **Planning feedback.** Before an arm moves, the interface shows **Planning · testing safe routes**, an animated search bar, and elapsed solve time while it tries IK seeds and collision-checked paths. Select **Cancel solver** to stop before execution; once a route is found, the bar changes to exact control-step progress.
- **Speed.** A slider advances the run between 0.25× and 8× control steps per frame: slow enough to watch a correction, fast enough to skip a long reach.
- **Step budget.** The **Step budget** slider sets the maximum number of control steps (1–400). Each scenario loads its recommended horizon, including 200 for Towel fold; raise it to inspect a longer run or lower it to test bounded failure handling.
- **Retry until goal.** When enabled, a run that exhausts its step budget returns to home and retries; a reached goal stops the run, and a safety halt is never retried automatically. Scenario 07 enables this by default.

**Full reset** restores the selected scenario’s home poses, goal cubes and heights, active-arm selection, cloth/timeline state, and collision-checked IK plans. It is therefore safe to use after dragging a cube or scrubbing a recorded frame: the next base-policy run starts from the task’s declared contract rather than a leftover manual target.

### Scenario 07: Towel fold specialist

Scenario 07 now identifies its task-specific browser controller and renders a spring-cloth approximation in the 3-D viewport. The towel responds to gravity, table contact, structural springs, and only attaches to a corner once the corresponding gripper reaches it; it is not morphed merely because a progress counter advances. The visible 120-frame rolling buffer reports its captured corners and whether the mesh has settled. It remains an intentionally limited browser approximation: it does not model self-collision, friction, material anisotropy, or real gripper contact. Training a learned policy would require recorded demonstrations, camera observations, a training runtime, and evaluation data that this static repository does not include.

## Model and data direction

Use recordings from the actual workcell to fine-tune a policy. The curated source registry documents the implementation references:

- [SmolVLA documentation](https://huggingface.co/docs/lerobot/smolvla) — a practical open VLA starting point for an SO-101-style setup.
- [LeRobot SO-101 guide](https://huggingface.co/docs/lerobot/il_robots) — leader/follower teleoperation.
- [π₀ paper](https://www.physicalintelligence.company/download/pi0.pdf) — a generalist VLA research reference.
- [LeRobotDataset](https://huggingface.co/docs/lerobot/lerobot-dataset-v3) — target interchange format for collection and training.

`data/sources.json` also carries a `study_path`: LeRobot, MuJoCo, MoveIt 2, Isaac Lab, OpenVLA, and Modern Robotics, in the order they answer each other's assumptions.

Never connect the browser prototype directly to hardware. The original design requires a hardware-side safety wrapper, watchdog, dead-man control, and physical e-stop.

Voice capture is capped at 60 seconds / 5 MB and episode import at 8 MB, preventing recordings from retaining an unbounded browser-memory buffer.

## Deployment

The Pages workflow tests, builds, and deploys `main`. Repository Pages is configured to use **GitHub Actions** as the source.

## Attribution

See [AUTHORS.md](AUTHORS.md) for project authorship and source attribution.
