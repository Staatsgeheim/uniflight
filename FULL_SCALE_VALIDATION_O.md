# Full-scale independent verification guide — Milestone O

Run the internal formal suite:

```bash
uniflight-verify run --output reports/o_verification.json --markdown reports/o_verification.md
```

For stronger independent verification, obtain public NASA/NESC 6-DOF check-case time histories independently and compare them using:

```bash
uniflight-verify compare-csv reference.csv actual.csv \
  --channels altitude speed \
  --abs-tol 1e-6 --rel-tol 1e-8 \
  --output reports/o_external_comparison.json
```

Recommended numerical studies include RK4 refinement with `h, h/2, h/4, h/8`, progressively tighter DOP853 tolerances, event-time comparisons, and conservation-law monitoring.

External benchmark comparison is verification against an independent reference, not validation against a flown mission.
