from __future__ import annotations

from fastmcp import FastMCP

REMINDERS = """
Mandatory:
- SI internally.
- Quaternion maps body → inertial.
- Body axes are +x forward, +y right, +z down.
- Never mutate sampled estimator/controller state inside an adaptive RHS.
- Pin datasets/plugins.
- Record solver tolerances/steps and seeds.
- Verify before declaring success.
- External benchmark agreement is not flight validation.
"""

SERVER_INSTRUCTIONS = f"""
UniFlight is a planet-agnostic 3/6-DOF research flight-dynamics engine. These tools call UniFlight public APIs. Do not reimplement physics, integrate outside UniFlight, or invent plugin-install/shell/python_exec tools.

Workflow:
1. system_capabilities / system_version
2. mission_validate then mission_compile (persist) before compute
3. simulation_run (request a background task for long runs)
4. inspect with simulation_summary, paginated simulation_events / simulation_vehicle_history
5. optimize or analyze only after a deterministic evaluator exists; replay reported optima
6. verify with verification_builtin or an explicit CSV/run comparison before claiming success

Pagination: use page.limit/cursor for events, history, catalogs, and campaign cases. Never request an entire campaign in one call.
Campaigns: analysis_* tools write SQLite checkpoints on the coordinator; workers must not open the store. Restart uses the same campaign_id and mission SHA.
Auth: STDIO is trusted-local. HTTP requires configured tokens.

{REMINDERS.strip()}
""".strip()


PROMPTS = {
    "create_uniflight_mission": "Guide from mission intent to valid MDL. Infer body, vehicle, initial state, phases, models, solver, events, outputs, and datasets, then validate before execution.",
    "debug_uniflight_simulation": "Start with run summary/events, then examine state, flow, force breakdown, solver settings, event roots, units, frames, and model validity.",
    "design_entry_trajectory": "Emphasize entry state, atmosphere, continuum/rarefied regime, aero, heating/TPS, peak-q/heat/deceleration outputs, and solver verification.",
    "design_landing_simulation": "Emphasize EDL event sequencing, deployables, powered descent, GNC chronology, terrain/contact, touchdown metrics, and Monte Carlo robustness.",
    "optimize_trajectory": "Build a deterministic evaluator, bounded/scaled variables, objective/constraints, validate the declaration, solve, and independently replay the optimum.",
    "analyze_monte_carlo_failures": "Query failure pages rather than dumping all cases, cluster failure reasons, replay representative cases, and distinguish physical failure from timeout/criteria failure.",
    "verify_against_external_reference": "Require untouched reference data/hash, published assumptions, channel mapping, timestamp audit, explicit tolerance/alignment, solver refinement, and evidence category external_benchmark.",
    "create_uniflight_plugin": "Create a separate package using uniflight.plugins, exact Plugin API version, namespaced capabilities, version-pinned mission requirement, and compatibility/collision tests.",
    "review_uniflight_mission": "Check MDL schema, units, frames, fidelity, data/plugin pinning, events/priorities, solver settings, outputs, reproducibility, and scientific claims.",
}


def register_prompts(mcp: FastMCP) -> None:
    for name, body in PROMPTS.items():
        def _factory(n: str, b: str):
            @mcp.prompt(name=n, version="1")
            async def _prompt() -> str:
                return f"{b}\n{REMINDERS}"
            return _prompt
        _factory(name, body)
