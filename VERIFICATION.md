# Milestone C verification record

Milestone C is accepted when all Milestone A/B regressions and the new coupled 6-DOF cases pass.

## Result

**26 tests passed.**

### Existing regression coverage

1. Tsiolkovsky rocket equation
2. Kepler two-body energy/angular momentum
3. vacuum radial free fall
4. constant-rate quaternion propagation
5. state packing/immutability
6. frame vector/tensor round trip
7. event root timing
8. jump-map plumbing
9. gas-mixture thermodynamic closure
10. exact spherical hydrostatic pressure integral
11. body rotation + wind environment velocity
12. atmospheric ceiling → vacuum
13. relative-flow Mach/Re/Kn/q calculation
14. continuum point-mass drag magnitude/direction
15. Mach-table drag interpolation
16. pressure-corrected rocket thrust/mass flow
17. end-to-end 3-DOF atmospheric ascent

### Milestone C verification cases

18. analytic alpha/beta recovery and orthonormal wind frame
19. attitude transformation of inertial relative flow into body axes
20. 6-DOF wind-axis force and body-moment coefficient scaling
21. trilinear Mach/alpha/beta aerodynamic database interpolation
22. ellipsoid projected-area change with flow orientation
23. constant body torque vs. Euler rigid-body equation
24. torque-free asymmetric rigid body: rotational energy and inertial angular-momentum conservation
25. two-axis TVC: body/inertial force and mounting-arm moment
26. end-to-end fictional-world 6-DOF atmospheric flight coupling translation, rotation, attitude, aerodynamics, TVC, gravity, and mass depletion

## Current numerical thresholds

- Wind-frame orthogonality: near machine precision in analytic cases
- Torque-free rotational energy relative drift: < 2e-10
- Torque-free inertial angular momentum relative drift: < 3e-10
- Quaternion norm in coupled 6-DOF example/test: < 2e-8 error without post-step projection
- Existing Milestone A/B tolerances remain unchanged

## Command

```bash
PYTHONPATH=src pytest -q
```
