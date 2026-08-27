# Milestone O — Formal Verification & Numerical Credibility

**UniFlight 1.0.0**

Milestone O is the final milestone in the POST2-class research-capability roadmap. It adds a formal verification layer while keeping the project boundary explicit: UniFlight performs mathematical/software verification but does not claim real-mission validation, flight heritage, certification, or independent IV&V pedigree.

## Verification taxonomy

The verification layer separates: analytical limiting cases, manufactured solutions, conservation/invariant checks, derivative checks, convergence studies, cross-integrator comparisons, hybrid-event verification, regression baselines, and external reference time-history comparison.

## Built-in suite

`uniflight-verify run` evaluates twelve self-contained internal cases:

1. RK4 manufactured exponential convergence.
2. Adaptive manufactured sine solution.
3. Tsiolkovsky limiting case.
4. Circular Kepler orbit/invariants.
5. Point-mass gravity Jacobian.
6. Constant-rate quaternion kinematics.
7. Axisymmetric torque-free rigid body.
8. Hybrid event-root timing.
9. DOP853/RK4 cross-integrator comparison.
10. Rigid two-body separation momentum conservation.
11. Frame-graph round trip.
12. Long-run quaternion norm stability.

Two NASA/NESC external benchmark manifests are included as `SKIP` placeholders until independently obtained reference trajectories are supplied. They are not counted as internal passes.

## Acceptance convention

A scalar comparison uses

`error <= absolute_tolerance + relative_tolerance * max(|reference|, scale_floor)`.

Tolerance values are explicit in each case or baseline; there is no hidden global epsilon.

## External-reference boundary

`ReferenceTimeHistory` and `uniflight-verify compare-csv` allow candidate time histories to be interpolated onto independent reference timestamps and compared channel-by-channel with explicit tolerances. External NASA/NESC data are intentionally not bundled.

## Non-claims

UniFlight 1.0.0 does not claim flight-data correlation, flight heritage, mission certification, NASA endorsement, independent IV&V, or compliance with a particular agency software-assurance process.
