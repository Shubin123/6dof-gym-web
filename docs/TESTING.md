# Testing guide

ArmLab's logic layer (`src/core.js`, `src/cloth.js`, `src/half-fold.js`, `src/shirt-fold.js`, `src/arm-track.js`, `src/rigid.js`, `src/stack-controller.js`, `src/rigid-plan.js`, `src/rigid-tasks.js`, `src/task-policies.js`) is plain, DOM-free JavaScript on purpose, so it runs directly under Node's built-in test runner with no browser, bundler, or mocking framework. The UI layer (`src/main.js`, `src/viewport3d.js`) wires that logic to the DOM and to three.js and is verified by hand in a real browser instead, as described below.

## Running the suite

```bash
npm test               # node --test — the same command CI runs before every deploy
npm run test:coverage  # adds line/branch/function coverage via node's built-in reporter
```

Each `test/*.test.js` file is its own process; the runner parallelizes across files but runs tests within a file sequentially. A few files are deliberately split out (see the comment at the top of `test/motion-safety.test.js`) so a heavy per-workflow search runs in its own process instead of adding to one file's sequential total.

As of this writing the suite is 91 tests across 16 files, all passing, with line coverage at 99.7% and branch coverage at 87% for the three logic modules. `core.js:442-445` is the one intentionally-uncovered branch: a defensive fallback inside `reduceSafeCellMotion` documented in its own comment as a guard against a caller passing a malformed `frames` array directly (something no code in this repo does — `reduceSafeCellMotion` is always called with output from `planSafeCellMotion`, whose adjacent frames are already delta-safe by construction).

## What each file covers

| File | Verifies |
| --- | --- |
| `test/core.test.js` | The task contract itself (goals inside the workspace and reach shell), forward/inverse kinematics, the safety-cell evaluator, episode-artifact shape, and the solver-ticker formatter. |
| `test/motion-safety.test.js` | Every declared workflow plans a full safe route end to end — solve IK, plan a safe cell motion, reduce it, and confirm every resulting frame clears the floor, the workspace edge, the other arm, and the per-joint delta cap. |
| `test/guidance-recovery.test.js` | The tracking controller (`guidedStep`) recovers cleanly from representative manual poses scattered across the workspace, not just from the home pose. |
| `test/cloth.test.js` | The kernel-based cloth simulator in isolation: mesh-derived topology (seam merging, spring CSR), resting stability and a flat midline, resting stability under gravity, table friction, magnet grasp/release and tracking, explicit gripper commands and where a released corner is pinned, the fold motion's layer separation, and snapshot/restore for timeline scrubbing. |
| `test/cloth-settings.test.js` | The cloth settings page's contract: one control per live-tunable parameter with its default in range, sanitizing and clamping stored values, surviving corrupt or blocked localStorage, `configure()` retuning a live simulator without touching its state, the stiffness-load stability estimate, and the preview's scripted fold actually folding. |
| `test/demo7.test.js` | The Towel fold task specification and its bimanual folding motion end to end with the plan's gripper commands: each gripper closes on its corner, the towel never leaves its footprint, and Arm A's corner is pinned where it was held. |
| `test/fold-guide.test.js` | The Task 7 guide's stage logic walks grasp → lift → place → done with the real fold and restarts on reset. |
| `test/viewport-cloth.test.js` | The 3-D towel is drawn in the same place as the simulated one: through the whole fold each held corner sits within 3 px of its gripper tip (a mirrored towel once drew them 85 px away), and the resting towel lies under its scene footprint. |
| `test/towel-fold-planner.test.js` | `planTowelFoldMotion` in isolation: a valid, collision-free, rate-capped bimanual route with one gripper command per frame; both grippers close on the corners, and while Arm B holds cloth its tool moves in small straight steps along the fold path. |
| `test/task-policies.test.js` | The declarative policy-recipe registry and the dense per-stage fold reward. |
| `test/regression.test.js` | Contract-level edge cases the integration tests above only exercise incidentally: `reduceSafeCellMotion`'s `keepTailFrames`/`requiredFrameIndexes` options and its empty-input result, `policyRecipeFor`'s fallback for an unknown task, and `scoreTaskStages`'s neutral-default and fully-solved boundary cases. Added to close the coverage gaps a plain `npm run test:coverage` pass turns up after a change to `core.js` or `task-policies.js`. |
| `test/dataset-export.test.js` | `buildDatasetManifest`: the empty-input and mixed-arm-count rejections, and that a multi-episode bundle gets correct feature shapes, deduplicated task indices, contiguous global frame indexing, and `next.done`/`next.success` flags. `scripts/lerobot_export.py` (Python, outside `npm test`) is verified by hand against a real downloaded bundle - see its own docstring. |
| `test/rigid.test.js` | The rigid-object tasks (13-15): objects rest and fall under gravity, tool-down IK points the tool straight down with horizontal jaws, each task's planned pick and place succeeds in the cannon-es simulation with every frame rate-capped and cell-safe, a stale plan closes on air (the grasp is contact-gated, not a magnet), a held cube cannot be lowered through another, and snapshot/restore mid-carry finishes the task identically. |
| `test/half-fold.test.js` | Task 12's half fold in both directions (back edge onto front, front onto back): a plan exists from home, every two-arm frame is cell-safe and rate-capped, both grippers close together and both let go, and after the cloth settles each carried corner lies within 1.5 cm of, and on top of, the corner it was laid on - four corners into two. Also that a flat towel is not scored as folded. |
| `test/shirt-fold.test.js` | Task 17's bimanual multi-fold garment manipulation: structured T-shirt mesh topology, regional tags and anatomical landmarks, table rest stability without penetration or seam self-lift, collision-free and rate-capped bimanual multi-fold motion plan (left sleeve inward, right sleeve inward, bottom hem upward), multi-layer thickness separation with garment friction, and stage score reaching completion. |
| `test/stack-fire.test.js` | Task 16 (Stack under fire), the closed-loop stacker against real physics with scripted shots: undisturbed it builds the three-cube tower and holds it; shot down it rebuilds, and a hard hit on a carried cube knocks it out of the gripper and is recovered from, while a slow ball does not; hammered repeatedly, no cube is ever left at rest outside the pen; balls are real bodies that are capped, cleared, and survive snapshot/restore. Every frame is checked against the joint-rate cap and the cell-safety envelope, as main.js does live. |

## Adding a regression test

When you fix a bug or add a branch to `core.js`, `cloth.js`, or `task-policies.js`, prefer extending `test/regression.test.js` (or the most specific existing file above) over writing a new file, unless the addition is its own coherent concern the way `motion-safety.test.js` and `guidance-recovery.test.js` are. Run `npm run test:coverage` before and after — a new branch that doesn't move the uncovered-lines list didn't get exercised.

## What isn't unit-tested, and how it's checked instead

`src/main.js` and `src/viewport3d.js` are UI glue: DOM event wiring, slider state, canvas/SVG rendering, and the three.js scene graph. They have no meaningful behavior outside a real DOM and WebGL context, so they're intentionally excluded from the Node test run rather than covered through a heavy DOM-mocking layer. Verify them by hand in a browser after any change that touches these files:

1. `npm run dev` and open the printed URL.
2. Open the live lab, and for at least one single-arm task and Task 07 (Towel fold):
   - Confirm the scenario loads with no console errors.
   - Click **Run demo policy**; confirm the planning ticker and then the step-progress bar advance, and the run ends in a named halt state.
   - Switch **2D ↔ 3D**; for Towel fold, confirm the cloth renders and settles instead of teleporting to its folded state.
   - Drag a goal (2D click, or 3D cube drag) and confirm the safety envelope rejects an out-of-bounds move without crashing.
   - Click **Reset** and confirm the scenario returns to its declared home state.
   - For a rigid-object task (13-15), run the demo policy and confirm it halts with "goal reached" and the object resting on its target in both 2D and 3D; click the table to move the target and run again. For Task 16, run it, click the tower in 2D and in 3D to shoot it down, and confirm the status line shows it rebuilding.
3. Check the browser console for uncaught errors throughout — a page can render its shell while a data path underneath it is broken.

This is the same check this repository's CI performs implicitly by building successfully (`npm run build` catches import and syntax errors in these files) plus the automated suite above, which validates every code path these files call into. It does not replace looking at the running app after a UI change.

## CI gate

`.github/workflows/deploy-pages.yml` runs `npm run test` and `npm run build` on every push to `main`, before the Pages deploy step. A failing test blocks the deploy.
