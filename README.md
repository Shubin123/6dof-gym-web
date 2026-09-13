# ArmLab — 6‑DOF Gym

A static, interactive web prototype derived from the local **6-DOF gym on the web** design vault. It demonstrates a 6-DOF arm task contract, task workflows, browser episode recording, curated source data, and an explicit policy-selection ladder.

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

## Model and data direction

Use recordings from the actual workcell to fine-tune a policy. The curated source registry documents the implementation references:

- [SmolVLA documentation](https://huggingface.co/docs/lerobot/smolvla) — a practical open VLA starting point for an SO-101-style setup.
- [LeRobot SO-101 guide](https://huggingface.co/docs/lerobot/il_robots) — leader/follower teleoperation.
- [π₀ paper](https://www.physicalintelligence.company/download/pi0.pdf) — a generalist VLA research reference.
- [LeRobotDataset](https://huggingface.co/docs/lerobot/lerobot-dataset-v3) — target interchange format for collection and training.

Never connect the browser prototype directly to hardware. The original design requires a hardware-side safety wrapper, watchdog, dead-man control, and physical e-stop.

Voice capture is capped at 60 seconds / 5 MB and episode import at 8 MB, preventing recordings from retaining an unbounded browser-memory buffer.

## Deployment

The Pages workflow builds and deploys `main`. Repository Pages must be configured to use **GitHub Actions** as the source. With a private repository, site availability depends on the owner’s GitHub plan and Pages visibility configuration.
