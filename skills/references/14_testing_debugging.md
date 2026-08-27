# Testing, debugging, and release hygiene

## Test order
1. single focused regression;
2. module test file;
3. related milestone group;
4. full suite in bounded groups if runtime limits exist;
5. formal verifier if numerics changed;
6. examples;
7. wheel install + CLI smoke if packaging/CLI changed.

## Debugging event failures
Inspect:
- guard value before/after step;
- direction;
- terminal/continue semantics;
- priority;
- root tolerance;
- same-time event collection;
- post-jump guard value;
- iteration count.

## Debugging NaNs/divergence
Check:
- units;
- mass/inertia positivity;
- quaternion normalization;
- atmosphere below/above validity;
- Mach/Re/Kn extremes;
- interpolation/extrapolation;
- contact stiffness vs timestep;
- controller saturation;
- solver tolerances.

## Debugging frame/sign errors
Construct trivial basis-vector tests. Verify body→inertial then inverse returns the original. For aero, test zero alpha/beta and simple positive-alpha cases.

## Monte Carlo debugging
Reproduce one failed case by exact seed and parameters before changing campaign logic. Separate physical failure from timeout/acceptance-criterion failure.

## Static/release checks
```bash
pytest
pytest --cov=uniflight --cov-branch --cov-fail-under=80
ruff check src tests
mypy src/uniflight
python -m build
```
Then install the wheel into a clean environment and smoke-test CLIs.

## Documentation synchronization
When changing public behavior update:
- README;
- relevant milestone/API doc;
- schema if MDL changes;
- version;
- verification counts/results only after actually rerunning them.
