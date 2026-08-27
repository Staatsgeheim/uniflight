# Agent recipes

## Recipe A — new 3-DOF ascent
- spherical body + gravity;
- atmosphere/environment;
- 3-DOF schema;
- ideal rocket + drag;
- dry-mass/cutoff event;
- DOP853 reference;
- verify vacuum/no-drag limit against rocket equation.

## Recipe B — new 6-DOF atmospheric vehicle
- 6-DOF schema;
- mass properties;
- quaternion initial attitude;
- aero coefficient model + geometry;
- propulsion wrench;
- rigid-body dynamics;
- verify force/moment signs and quaternion norm.

## Recipe C — reentry/EDL
- entry schema;
- regime-blended aero;
- heating + TPS;
- deployables;
- powered descent;
- terrain/contact;
- ordered events;
- report peak heat/q/deceleration and touchdown state.

## Recipe D — closed-loop landing
- physical case;
- sensors;
- navigation EKF;
- guidance;
- attitude/throttle control;
- actuator dynamics;
- sampled-data engine;
- Monte Carlo dispersions;
- diagnose failures by event/criterion.

## Recipe E — staging
- parent 6-DOF vehicle;
- separation guard;
- daughter templates;
- rigid separation handler;
- assert momentum conservation;
- propagate daughters concurrently;
- optional DOF demotion for coast.

## Recipe F — optimization
- deterministic evaluator;
- design variables/bounds;
- metrics;
- objective/constraints;
- SLSQP target;
- derivative-free/multistart check;
- independently rerun optimum.

## Recipe G — engineering database
- create N-D table;
- units/provenance/validity;
- persist/checksum;
- register catalog version;
- adapt to domain model;
- test node interpolation and boundaries.

## Recipe H — plugin
- separate distribution;
- entry point `uniflight.plugins`;
- exact Plugin API version;
- namespaced registrations;
- mission exact version requirement;
- install/discover/compile/run smoke;
- test mismatch/collision failures.

## Recipe I — HPC campaign
- declarative analysis or Python campaign;
- stable campaign ID;
- SQLite store;
- serial debug first;
- process backend with BLAS threads=1;
- checkpoint/restart;
- export JSON summary.

## Recipe J — external benchmark
- untouched external data + hash;
- exact published assumptions;
- tight adaptive run;
- time-grid audit;
- per-channel residuals;
- convergence run;
- reproducibility bundle.
