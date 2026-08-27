# External benchmarking and NESC workflow

## Scientific category
An NESC comparison is independent benchmark verification, not real-flight validation.

## Generic workflow
1. Obtain reference data independently and preserve it unchanged.
2. Record SHA-256.
3. Reproduce the published model assumptions exactly.
4. Convert units only at input/output boundaries.
5. Run a tight adaptive reference solution.
6. Export candidate channels on a known time grid.
7. Inspect reference time vectors for quantization/representation artifacts.
8. Compare on physically intended timestamps.
9. Report max absolute, RMS, terminal, and relative errors.
10. Compare error magnitude to spread among independent reference implementations where available.
11. Perform solver refinement to show candidate error is not integrator noise.
12. Package inputs, scripts, outputs, hashes, and instructions.

## NESC Case 04 lesson
The Case 04 package exposed a subtle issue: one reference file stored nominal 0.01-s timestamps with tiny floating representation offsets. Literal interpolation created an artificial high-frequency sawtooth in residual plots. Snapping timestamps to their intended uniform grid removed the artifact.

Therefore:
- never assume stored decimal timestamps are exact physical jitter;
- characterize `diff(time)` first;
- infer/confirm intended output cadence from benchmark documentation;
- only snap when the nominal grid is justified;
- document the alignment method.

## Benchmark acceptance
Do not cherry-pick one reference. Report every independent reference trajectory and explain known modeling differences/outliers.

## Reproduction package contents
Include:
- untouched reference archive;
- exact framework wheel/source version;
- simulation script;
- comparison script;
- plotting script;
- requirements;
- Windows/POSIX runner;
- baseline outputs;
- SHA256 manifest.
