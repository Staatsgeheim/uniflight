# Milestone E verification record

Milestone E is accepted when every Milestone A/B/C/D regression and the new entry-descent-landing cases pass.

## Result

**47 tests passed.**

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
12. atmospheric ceiling -> vacuum
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
27. entry-state schema extension
28. Knudsen regime-dispatch endpoints/midpoint
29. free-molecular reference drag
30. low/high-Mach blend
31. Newtonian-inspired hypersonic incidence behavior
32. generalized Sutton-Graves velocity scaling
33. bounded thermochemical heating correction
34. lumped TPS heating / heat-load integration
35. ablation mass coupling
36. canonical mass-flow aggregation
37. end-to-end rarefied-to-continuum heated entry

### Milestone E verification cases

38. first-order deployable rate and state ownership
39. parachute drag direction and deployment-area scaling
40. radial terrain AGL / normal / surface-point query
41. landing-gear spring-damper normal contact force
42. regularized Coulomb friction opposes tangential foot motion and respects the friction bound
43. two-body separation conserves linear momentum and prescribed daughter relative velocity
44. jettison jump removes canonical mass and resets deployable state
45. powered-descent throttle includes local-gravity feedforward
46. hybrid mode engine sequences terminal events and mode intervals
47. end-to-end parachute -> jettison -> powered descent -> gear deployment -> touchdown -> first contact-compression stop

## End-to-end EDL observations

The verification trajectory uses fictional body **Nereid-E** and is intentionally not calibrated to Earth or any operational vehicle.

Representative checked results:

- initial AGL: 3000 m
- initial radial speed: -120 m/s
- parachute maximum area: 80 m^2
- powered-descent transition / parachute jettison: ~185.764 s at 500 m
- touchdown: ~301.202 s
- first zero-radial-speed compression point: ~301.271 s
- final CG altitude: ~1.975 m
- final radial speed: numerically zero at the first compression maximum
- final vehicle mass: ~364.228 kg
- gear deployment fraction: 1.0
- quaternion norm: 1.0 to displayed precision
- landing-gear contact active at termination

The powered engine is removed from the contact-mode RHS at touchdown, representing an immediate engine cutoff. The contact phase then evolves under gravity and landing-gear forces until the first compression stop.

## Current numerical thresholds

- all Milestone A-D thresholds remain unchanged
- two-body separation momentum residual: < 1e-12 in the reference test
- hybrid synthetic event timing: < 1e-9 s
- final EDL radial speed at first compression stop: < 1e-5 m/s
- gear deployment at landing: > 0.95
- quaternion norm error in EDL: < 1e-7

## Command

```bash
PYTHONPATH=src pytest -q
```
