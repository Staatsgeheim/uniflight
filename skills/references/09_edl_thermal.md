# Entry, descent, landing, thermal, and TPS

## Entry aerodynamics
Combine continuum/hypersonic/rarefied models according to Mach/Kn validity. Use smooth regime blending where configured.

## Heating
- `SuttonGravesHeating`
- `PowerLawRadiativeHeating`
- `AerothermalModel` interface

Sutton-Graves coefficients are atmosphere/body/model dependent. Do not transplant an Earth coefficient to another atmosphere without justification.

## TPS
`LumpedAblatingTPS` evolves temperature/heat load/recession/mass in an engineering lumped model. Couple ablated mass through mass flow/mass state consistently.

## Deployables/parachutes
- `FirstOrderDeployable`
- `InflatingParachute`

Deployment is a hybrid/stateful process; test opening loads and inflation transients.

## Powered descent
- `VerticalDescentThrottle`
- `VectorLandingGuidance`
- GNC stack in control/guidance modules

## Terrain/contact
- `RadialTerrain`
- table-driven terrain adapter
- `LandingGearContact`
- dynamic gear

Contact is compliant. Use small enough steps/tolerances to resolve stiff compression dynamics; check penetration/compression and energy behavior.

## EDL event sequence
Typical guards:
entry interface → peak/conditions → deploy → heatshield/jettison → powered descent → touchdown/contact.
Define event priority where multiple criteria can occur together.
