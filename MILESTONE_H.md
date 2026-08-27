# Milestone H — General Targeting and Trajectory Optimization

UniFlight 0.8.0 adds the first POST2-class trajectory-design layer on top of the A–G/F.1 simulation stack.

## Architectural rule

Optimization treats the simulation as a deterministic black box:

`physical design variables -> trajectory evaluation -> named metrics -> objective/constraints`

The optimizer does not reach into or alter the flight-dynamics kernel. Therefore the same targeting machinery can wrap ascent, orbit, entry, EDL, GNC, future multi-vehicle propagation, or external user plugins.

## New abstractions

- `DesignVariable` — bounded scalar physical variable with optimizer scaling
- `DesignSpace` — packing/scaling/bounds/name mapping
- `TrajectoryProblem` — cached design-to-metrics evaluation contract
- `MetricObjective` — minimize/maximize a named scalar metric
- `MetricConstraint` — scalar/vector lower/equality/upper nonlinear constraints
- `TrajectoryTargeter` — bounded nonlinear least-squares targeting
- `TrajectoryOptimizer` — constrained SLSQP optimization with derivative-free COBYLA fallback
- `finite_difference_jacobian` — bound-aware central/one-sided sensitivities
- `MultipleShootingTranscription` — continuity-defect generation for arbitrary segment propagators
- `parallel_batch_evaluate` — process-parallel independent candidate evaluation

## Event targeting

Event times are ordinary design variables. A trajectory evaluator may propagate to an event and expose event-state/time metrics; the target residual then closes on those metrics. The included reference case targets the apogee event of a radial ascent.

## Multiple shooting

H provides the transcription primitive rather than imposing one mission topology. For nodes `x_i` and segment propagators `Phi_i`, the defect vector is

`d_i = Phi_i(x_i, p) - x_(i+1)`.

Those defects can be surfaced as equality metrics in a `TrajectoryProblem`, while node states and global mission parameters are represented by design variables. A future mission-definition layer will automate that packing.

## Gradients and derivatives

H deliberately starts with robust finite differences. The interface leaves room for analytical, complex-step, automatic-differentiation, or adjoint derivatives later. Scaling is explicit so finite differences operate in approximately nondimensional optimizer coordinates.

## Evaluation caching

SciPy optimizers commonly request objective and constraints repeatedly at the same design vector. `TrajectoryProblem` therefore keeps a bounded exact-value LRU cache, so one trajectory propagation can feed all objective/constraint callbacks at a point.

## Reference problem

The Nereid-H case launches radially from a fictional airless body.

1. A one-variable targeter finds burn duration at fixed `mdot=5 kg/s` for a 20 km apogee.
2. A two-variable constrained optimization minimizes propellant using `mdot` and burn duration while enforcing the same 20 km apogee equality.
3. The powered phase uses UniFlight numerical dynamics. The repeated optimizer coast metric uses the exact two-body energy relation; a separate explicit event-propagation path verifies the same apogee target.

The optimized solution pushes mass flow to approximately its allowed upper bound, as physically expected for a minimum-propellant radial burn where higher thrust reduces gravity loss.

## Deliberate H boundaries

H does not yet provide:

- an automatic mission-language transcription into shooting nodes and constraints
- adjoint/automatic-differentiation gradients
- pseudospectral/direct-collocation transcription
- distributed optimization across machines
- a general sparse NLP backend such as IPOPT/SNOPT
- dynamic multi-vehicle topology inside the optimizer

Those are later POST2-parity roadmap items rather than gaps in the H public contract.
