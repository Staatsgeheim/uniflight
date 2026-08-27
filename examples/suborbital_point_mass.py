"""Minimal arbitrary-body suborbital propagation: no Earth constants are embedded."""
import numpy as np
from uniflight import core_3dof_schema, PointMassGravity, TranslationalKinematics, DynamicsAssembler, SimulationEngine

# A fictional spherical body's gravitational parameter (m^3/s^2).
mu = 8.0e11
radius = 1.2e6
schema = core_3dof_schema()
y0 = schema.pack({
    "position": np.array([radius, 0.0, 0.0]),
    "velocity": np.array([0.0, 900.0, 300.0]),
    "mass": 1000.0,
})
assembler = DynamicsAssembler(schema, [TranslationalKinematics(PointMassGravity(mu))])
result = SimulationEngine(assembler.rhs).run((0.0, 1200.0), y0)
final = schema.unpack(result.states[-1])
print(f"steps={len(result.times)}")
print(f"final radius={np.linalg.norm(final['position']):.3f} m")
print(f"final speed={np.linalg.norm(final['velocity']):.3f} m/s")
