# Authors and attribution

## Project authorship

- **Shubin** ([Shubin123](https://github.com/Shubin123)) — repository author and maintainer. The repository's Git history identifies the human author under `Shubin <44554264+Shubin123@users.noreply.github.com>` and `Shubin123 <wangshubin130@gmail.com>`.

## Adapted techniques

- **bandinopla** — [three-simplecloth](https://github.com/bandinopla/three-simplecloth) (MIT license). `src/cloth.js`'s self-collision / fold-layer-separation constraint caps how far a single solver iteration may move a vertex rather than snapping it exactly, the same fix three-simplecloth applies to its own spring and collision forces (a per-step force magnitude clamp) to keep a constraint from fighting the structural springs and oscillating. The rest of `src/cloth.js` (Verlet/PBD integration, spring topology, gripper anchoring, fold metrics) predates this and is original to this project; only that one technique is adapted, credited here and at its point of use in the source.

## Research, documentation, and software references

The project links to the original publishers for its educational and technical references. Those sources remain their respective authors' and publishers' work; they are not reproduced or claimed as original project content.

- Hugging Face — LeRobot and SmolVLA documentation
- Physical Intelligence — π₀ research paper
- Columbia University — Diffusion Policy
- MoveIt contributors — MoveIt 2
- DeepMind — MuJoCo
- NVIDIA — Isaac Lab
- OpenVLA authors — OpenVLA
- Kevin M. Lynch and Frank C. Park — *Modern Robotics*

See [data/sources.json](data/sources.json) for the direct source links and their use in the project.
