# 1.0.1 release-review remediation

This note records the engineering-review findings validated and remediated after the
1.0.0 release candidate.

## Validated high-severity findings

### Simultaneous adaptive events

**Confirmed.** In 1.0.0 every SciPy event wrapper was terminal. `solve_ivp` could stop on
the first declared terminal guard and omit a simultaneous higher-priority guard.

**Fix.** The adaptive integrator now distinguishes pure observation events from
state-changing/terminating events. At a terminal root it evaluates all guards at the
located root state and confirms crossing direction from the preceding accepted state,
then augments the tied root set before the simulation kernel applies priority-ordered
jump maps.

### Continuing non-jump state roots

**Confirmed.** Restarting at `np.nextafter(t_event, tf)` with the exact root state could
make SciPy immediately report the same initial zero again.

**Fix.** Pure `CONTINUE` events with no jump are nonterminal in the adaptive solver and
are recorded without restarting the dynamics. Fixed-step RK4 now normalizes guard
values inside its configured guard tolerance to zero, preserving its existing rule that
a zero at the left endpoint is not a new crossing.

### Mission identity mutability

**Confirmed.** The 1.0.0 `MissionDocument` froze only the outer mapping.

**Fix.** Mission data is recursively frozen (`mappingproxy` + tuples; arrays made
read-only). `mutable_copy()` recursively thaws the document for controlled override
workflows. The digest is computed before freezing and remains consistent with the frozen
content.

## Validated medium-severity findings

### JSON Schema mismatch

**Confirmed.** The emitted schema rejected runtime-valid root keys.

**Fix.** `atmospheres`, `environments`, `solvers`, and `metadata` are now emitted. A
regression test validates every bundled YAML/TOML mission with a standards-compliant
Draft 2020-12 validator.

### Release hygiene

**Confirmed.** `VERIFICATION.md` was stale, no license file was present, and development
quality tooling/CI was under-specified.

**Fix.**

- version bumped to 1.0.1;
- `VERIFICATION.md` synchronized to the current test/verification results;
- MIT `LICENSE` added;
- Python 3.11-3.13 GitHub Actions matrix added;
- branch-coverage gate set to 80%;
- Ruff correctness lint added;
- wheel build + clean-install CLI smoke tests added;
- Ruff/mypy/coverage/build/jsonschema tools added to the development extra.

## External scientific benchmark

The two NASA/NESC external manifests remain explicitly skipped. This remediation does
not turn unavailable external data into a pass and does not strengthen the project's
scientific-validation claims beyond the evidence actually run.
