# Known limitations and agent disclosure rules

For UniFlight 1.0.2, agents should disclose relevant limitations rather than implying unsupported fidelity.

## Built-in model limitations
- Built-in gravity is primarily point-mass; advanced harmonics/third-body require extension.
- Contact is compliant/penalty-style, not a general complementarity/DAE contact solver.
- Chemistry models are engineering corrections, not general finite-rate multi-temperature reacting flow.
- Flexible dynamics are modal engineering models, not FEM.
- Slosh is low-order, not CFD/free-surface.
- Engine dynamics are low-order, not turbomachinery/feed-system simulation.
- Terrain/body abstractions do not imply arbitrary high-resolution shape/gravity fidelity unless supplied by custom data/models.
- Distributed HPC is an executor seam, not a bundled Dask/Ray/Slurm implementation.
- Plugins are trusted in-process code and are not sandboxed.

## Validation boundary
Internal verification and NESC-style benchmark agreement do not establish:
- mission flight validation;
- hardware qualification;
- operational safety;
- certification;
- independent IV&V.

## Version drift
This skill targets 1.0.2. If a live checkout is newer:
1. read its changelog/version;
2. inspect changed public APIs;
3. run its tests;
4. adapt instructions;
5. do not force old assumptions onto new code.
