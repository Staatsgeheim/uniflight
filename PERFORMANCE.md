# UniFlight F.1 Performance Notes

## Sandbox benchmark environment

- visible logical CPUs: 5
- campaign scenario: Nereid-F sampled-data terminal landing
- GNC period: 0.5 s
- campaign integrator: fixed RK4, dt=0.1 s
- deterministic base seed: 20260827

## Single-case backend timing

Observed representative sandbox timings:

- adaptive SciPy DOP853: approximately 10–12 s/case
- fixed RK4 dt=0.1 s: approximately 3–4 s/case

The numerical output of the nominal trajectory agrees closely; see `README.md` and `VERIFICATION.md`.

## Process scaling smoke test

4-case campaign:

| Execution | Elapsed | Throughput |
|---|---:|---:|
| serial RK4 | 9.51 s | 0.421 cases/s |
| 4 processes RK4 | 6.14 s | 0.651 cases/s |

Speedup: ~1.55x in the constrained sandbox. Case results were exactly identical.

This should **not** be used to predict scaling on a 32-core workstation. Four cases are too few to amortize process startup, and the sandbox CPU allocation is constrained. The architecture exposes coarse-grained embarrassingly parallel work, so larger campaigns are expected to scale substantially better until memory bandwidth, process overhead, or CPU quotas dominate.

## Profiling changes included

- removed per-case full trajectory storage in campaign mode
- removed per-case GNC record allocation in campaign mode
- replaced numerical point-mass gravity Jacobian in EKF prediction with analytical gravity-gradient matrix
- fixed-step integrator returns segment endpoints only
- multiprocessing distributes whole trajectories, minimizing synchronization

## Further optimization candidates

F.1 intentionally stops before low-level specialization. If profiling on the target workstation shows the need, later work can add:

- Numba/JAX compiled RHS kernels
- batched/vectorized ensemble propagation
- shared immutable environment tables
- alternative multiprocessing start methods on POSIX
- CPU affinity / NUMA-aware worker placement
- compiled quaternion/aerodynamic kernels
- trajectory checkpoint/restart
- GPU ensemble propagation for very large campaigns

Those optimizations should be driven by profiler data from the actual workstation rather than guessed in advance.
