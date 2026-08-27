# UniFlight verification record — Milestone B

Validated in the provided execution environment using:

```bash
PYTHONPATH=src python -m pytest -q
```

Result: **17 passed**.

## Milestone A regression cases

| ID | Case | Acceptance criterion |
|---|---|---|
| 001 | Tsiolkovsky variable-mass rocket | relative error < `1e-9` |
| 002 | circular Kepler orbit | energy/angular-momentum drift < `2e-11`; position closure < `5e-10 R` |
| 003 | short-time radial free fall | agrees with independent local Taylor limit |
| 004 | constant-rate quaternion propagation | analytic quaternion agreement |
| 005 | hybrid event root timing | expected root time |
| 006 | jump-map plumbing | state transition applied at event |
| 007 | state-view immutability/schema behavior | mutation prevented, shapes enforced |
| 008 | frame transform round trip | numerical round-trip precision |

## Milestone B cases

| ID | Case | What is checked |
|---|---|---|
| 009 | Gas mixture closure | molar mass, `cp/cv`, gamma, mass fractions, viscosity, mean free path |
| 010 | Spherical hydrostatic atmosphere | pressure equals exact spherical hydrostatic integral; density EOS consistency |
| 011 | Environment rotation + wind | fluid velocity = body rotational velocity + wind |
| 012 | Atmosphere ceiling | clean transition to explicit vacuum sample |
| 013 | Relative-flow state | speed, dynamic pressure, Mach, Reynolds, Knudsen |
| 014 | Continuum drag | force opposes flow and equals `q Cd A` |
| 015 | Mach-table Cd | interpolation at an interior Mach number |
| 016 | Pressure-corrected rocket | ambient-pressure thrust and mass flow in atmosphere/vacuum |
| 017 | Integrated atmospheric ascent | burnout event timing/mass; drag reduces burnout speed and altitude |

## Model-specific formulas independently exercised

### Isothermal spherical hydrostatics

For constant composition and temperature in point-mass gravity,

```text
p(h) = p0 exp[ mu/(R T) (1/(R+h) - 1/R) ]
rho  = p/(R T)
```

Test 010 evaluates the implementation against this expression independently.

### Continuum drag

```text
q = 1/2 rho |V_rel|^2
F_D = -q Cd A V_rel/|V_rel|
```

Test 014 compares the full vector result to the scalar analytic magnitude and direction.

### Pressure thrust

For the Milestone-B engine closure at full throttle,

```text
T = mdot ve + (pe - pa) Ae
```

Test 016 checks both finite ambient pressure and vacuum-above-ceiling cases.

## Regression policy

All Milestone A tests remain in the Milestone B package and must continue passing. Future milestones should append verification cases rather than replace these baselines.
