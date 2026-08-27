# Milestone D verification record

Milestone D is accepted when every Milestone A/B/C regression and the new entry/re-entry cases pass.

## Result

**37 tests passed.**

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
18. analytic alpha/beta recovery and orthonormal wind frame
19. attitude transformation of inertial relative flow into body axes
20. 6-DOF wind-axis force and body-moment coefficient scaling
21. trilinear Mach/alpha/beta aerodynamic database interpolation
22. ellipsoid projected-area change with flow orientation
23. constant body torque vs. Euler rigid-body equation
24. torque-free asymmetric rigid body: rotational energy and inertial angular-momentum conservation
25. two-axis TVC: body/inertial force and mounting-arm moment
26. end-to-end fictional-world 6-DOF atmospheric flight

### Milestone D verification cases

27. entry-state schema extends canonical 6-DOF state with TPS temperature, heat load, and TPS mass
28. Knudsen dispatcher endpoints and log-space midpoint smoothness
29. free-molecular reference drag magnitude and direction
30. low/high Mach coefficient blend endpoint and midpoint behavior
31. Newtonian-inspired hypersonic reference model incidence behavior
32. generalized Sutton–Graves velocity-cubed scaling
33. thermochemical hook boundedness and endothermic heating correction
34. lumped TPS sub-ablation heating and heat-load integration
35. TPS ablation coupled consistently to TPS mass and total vehicle mass
36. mass-flow aggregator combines propulsion and an independent mass-loss source
37. end-to-end post-deorbit 6-DOF entry crossing free-molecular → transitional → continuum flow with heating and ablation

## End-to-end case observations

The verification trajectory uses the fictional body **Nereid-D** and is intentionally not Earth calibrated.

Representative results from the checked configuration:

- initial altitude: 450 km
- terminating altitude: 30 km
- entry time to termination: ~1811 s
- initial speed: ~1394 m/s
- final speed: ~313 m/s
- initial Knudsen number: ~75
- final Knudsen number: ~7.6e-7
- maximum dynamic pressure: ~2.1 kPa
- maximum reference heat flux: ~79 kW/m²
- integrated heat load: ~21.7 MJ/m²
- maximum TPS temperature: 900 K
- TPS / vehicle mass loss: ~2.8 kg
- quaternion norm error: below displayed precision

The test specifically checks that the start state selects the free-molecular branch, the final state selects the continuum branch, heat load grows, TPS mass decreases, total vehicle mass decreases by the same ablated amount, and velocity decreases through the entry.

## Current numerical thresholds

- Milestone A/B/C thresholds remain unchanged
- Kn transition midpoint: machine-precision agreement with 0.5 for the declared log-space bridge
- Sutton–Graves velocity scaling: relative agreement with exact V^3 ratio to ~1e-12 in the reference case
- final 30 km event altitude: < 1e-5 m error
- ablation mass bookkeeping: < 1e-6 kg mismatch between TPS loss and canonical vehicle loss
- quaternion norm in re-entry test: < 2e-8 error without explicit post-step normalization

## Command

```bash
PYTHONPATH=src pytest -q
```
