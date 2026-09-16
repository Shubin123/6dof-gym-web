# ArmLab — 6‑DOF Gym

A static, interactive web prototype derived from the local **6-DOF gym on the web** design vault. It demonstrates a 6-DOF arm task contract, a library of fully specified task examples, browser episode recording, curated source data, and an explicit policy-selection ladder.

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

Every example is reachable in the browser lab: `npm test` asserts that each goal lies inside the declared workspace, is IK-plannable, and keeps a complete specification.

The task families, curricula, safety boundaries, model map, and study path were folded in from the local **Arm Atlas — 6-DOF Learning Lab** study dashboard, which now lives on as a side piece to this repository.

## Viewports

The lab stage renders the same arm two ways, switched by the **2D / 3D** control:

- **2D** — the top-down SVG scene. Click anywhere to move the goal.
- **3D** — a three.js viewport. Drag to orbit, scroll to zoom, click the floor to move the goal.

The 3-D view is a spatial rendering of the *same* planar chain, not a second simulation: the top-down footprint matches the SVG exactly, and links are stacked at descending heights the way a SCARA-style arm is built. It adds no kinematics, collision model, or physics the 2-D scene does not already have.

three.js is loaded on demand. It sits in its own lazy chunk, so the initial page load is unchanged for anyone who never opens the 3-D view, and a browser without working WebGL falls back to the 2-D scene with a message instead of a broken stage.

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

The Pages workflow builds and deploys `main`. Repository Pages must be configured to use **GitHub Actions** as the source. With a private repository, site availability depends on the owner’s GitHub plan and Pages visibility configuration.
