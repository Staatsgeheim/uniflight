# UniFlight 1.0.1 verification

This document supersedes the stale Milestone-N verification summary that shipped in
1.0.0. Validation is executed in bounded groups because the sandbox limits individual
commands to roughly one minute.

## Regression total

**171/171 tests pass.**

The 1.0.0 baseline contained 165 tests. Version 1.0.1 adds six regression cases for
review findings:

1. adaptive simultaneous state events are all collected at one root;
2. adaptive tied events execute in descending priority order even when a lower-priority
   guard is declared first;
3. fixed-step RK4 preserves the same tie/priority semantics;
4. continuing non-jump state roots do not enter zero-time cycles in either integrator;
5. `MissionDocument.raw` is recursively immutable after hashing;
6. every bundled YAML/TOML mission validates against the emitted Draft 2020-12 JSON Schema.

## Formal numerical verification

`uniflight-verify run` reports:

```text
total   = 14
passed  = 12
failed  = 0
skipped = 2
success = true
```

The two skipped entries are external NASA/NESC reference manifests. They are deliberately
not counted as passes until independently sourced reference trajectories are compared.

## Release-review fixes

Version 1.0.1 additionally verifies:

- SciPy terminal-event tie collection is completed by guard evaluation at the located
  terminal state and crossing-direction confirmation from the prior accepted state;
- pure `CONTINUE`/no-jump events are nonterminal under adaptive integration, eliminating
  unnecessary restarts and left-endpoint re-trigger loops;
- fixed-step guards within the configured guard tolerance are normalized to zero before
  crossing tests, preventing duplicate numerical roots on restart;
- mission provenance data is recursively frozen, while `mutable_copy()` returns a deep,
  ordinary mutable structure for optimization/Monte Carlo overrides;
- the emitted mission schema contains all runtime-valid root sections, including
  `atmospheres`, `environments`, `solvers`, and `metadata`.

## CI/release hygiene

`.github/workflows/ci.yml` runs Python 3.11, 3.12, and 3.13 and includes:

- correctness-oriented Ruff checks;
- pytest with branch coverage and an 80% minimum;
- wheel build;
- clean wheel installation;
- `uniflight-mission` smoke validation;
- `uniflight-verify` smoke verification.

Development extras now include pytest, pytest-cov, build, jsonschema, Ruff, and mypy.
The repository also includes the MIT `LICENSE` file.

## Scope boundary

The release still does not claim real-flight validation, flight heritage, certification,
or independent IV&V. External benchmark comparison remains a separately reported
verification activity.
