# Testing guide

ArmLab's logic layer (`src/core.js`, `src/cloth.js`, `src/task-policies.js`) is plain, DOM-free JavaScript on purpose, so it runs directly under Node's built-in test runner with no browser, bundler, or mocking framework. The UI layer (`src/main.js`, `src/viewport3d.js`) wires that logic to the DOM and to three.js and is verified by hand in a real browser instead, as described below.

## Running the suite

```bash
npm test               # node --test — the same command CI runs before every deploy
npm run test:coverage  # adds line/branch/function coverage via node's built-in reporter
```

Each `test/*.test.js` file is its own process; the runner parallelizes across files but runs tests within a file sequentially. A few files are deliberately split out (see the comment at the top of `test/motion-safety.test.js`) so a heavy per-workflow search runs in its own process instead of adding to one file's sequential total.

As of this writing the suite is 46 tests across 8 files, all passing, with line coverage at 99.7% and branch coverage at 87% for the three logic modules. `core.js:442-445` is the one intentionally-uncovered branch: a defensive fallback inside `reduceSafeCellMotion` documented in its own comment as a guard against a caller passing a malformed `frames` array directly (something no code in this repo does — `reduceSafeCellMotion` is always called with output from `planSafeCellMotion`, whose adjacent frames are already delta-safe by construction).

## What each file covers

| File | Verifies |
| --- | --- |
| `test/core.test.js` | The task contract itself (goals inside the workspace and reach shell), forward/inverse kinematics, the safety-cell evaluator, episode-artifact shape, and the solver-ticker formatter. |
| `test/motion-safety.test.js` | Every declared workflow plans a full safe route end to end — solve IK, plan a safe cell motion, reduce it, and confirm every resulting frame clears the floor, the workspace edge, the other arm, and the per-joint delta cap. |
| `test/guidance-recovery.test.js` | The tracking controller (`guidedStep`) recovers cleanly from representative manual poses scattered across the workspace, not just from the home pose. |
| `test/cloth.test.js` | The kernel-based cloth simulator in isolation: mesh-derived topology (seam merging, spring CSR), resting stability and a flat midline, resting stability under gravity, table friction, magnet grasp/release and tracking, the fold motion's layer separation, and snapshot/restore for timeline scrubbing. |
| `test/demo7.test.js` | The Towel fold task specification and its bimanual folding motion end to end, including cloth physics interacting with the planned trajectory. |
| `test/towel-fold-planner.test.js` | `planTowelFoldMotion` in isolation: a valid, collision-free, bimanual route for the fold task specifically. |
| `test/task-policies.test.js` | The declarative policy-recipe registry and the dense per-stage fold reward. |
| `test/regression.test.js` | Contract-level edge cases the integration tests above only exercise incidentally: `reduceSafeCellMotion`'s `keepTailFrames`/`requiredFrameIndexes` options and its empty-input result, `policyRecipeFor`'s fallback for an unknown task, and `scoreTaskStages`'s neutral-default and fully-solved boundary cases. Added to close the coverage gaps a plain `npm run test:coverage` pass turns up after a change to `core.js` or `task-policies.js`. |

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
3. Check the browser console for uncaught errors throughout — a page can render its shell while a data path underneath it is broken.

This is the same check this repository's CI performs implicitly by building successfully (`npm run build` catches import and syntax errors in these files) plus the automated suite above, which validates every code path these files call into. It does not replace looking at the running app after a UI change.

## CI gate

`.github/workflows/deploy-pages.yml` runs `npm run test` and `npm run build` on every push to `main`, before the Pages deploy step. A failing test blocks the deploy.
