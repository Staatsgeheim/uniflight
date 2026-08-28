from __future__ import annotations

from typing import Any

from fastmcp import FastMCP
from fastmcp.dependencies import Depends, Progress

from .contracts import policy_for, tool_registry, tool_schema_resolved
from .errors import DomainError
from .models import PageRequest
from .runtime import fail, get_auth, get_services, page_req, system_version_payload
from .services import AppServices
from .auth import AuthorizationContext


def _capability_items() -> list[dict[str, Any]]:
    items = []
    for name, meta in tool_registry().items():
        category, _, rest = name.partition("_")
        items.append({
            "category": category,
            "capability_id": rest or name,
            "owner": "uniflight",
            "version": str(policy_for(name)["component_version"]),
            "description": str(meta.get("description") or name),
        })
    return items


CAPABILITIES = _capability_items()


def _kw(name: str) -> dict[str, Any]:
    pol = policy_for(name)
    return {
        "name": name,
        "version": str(pol["component_version"]),
        "tags": set(pol.get("tags") or []),
        "annotations": dict(pol["annotations"]),
        "timeout": float(pol["timeout_s"]),
        "task": pol.get("task_mode") == "optional",
        "output_schema": tool_schema_resolved(name, "output"),
    }


def _guard(auth: AuthorizationContext, name: str) -> None:
    for scope in policy_for(name)["required_scopes"]:
        auth.require(scope)


def register_tools(mcp: FastMCP) -> None:
    @mcp.tool(**_kw("system_version"))
    async def system_version(
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "system_version")
            return system_version_payload()
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("system_capabilities"))
    async def system_capabilities(
        category: str | None = None,
        page: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "system_capabilities")
            items = [c for c in CAPABILITIES if not category or c["category"] == category]
            items = sorted(items, key=lambda r: (r["category"], r["capability_id"]))
            window, info = services.cursors.paginate(
                items, page_req(page), tool="system_capabilities",
                tenant=auth.tenant_id, filters={"category": category},
            )
            return {"ok": True, "items": window, "page": info.model_dump()}
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("mission_validate"))
    async def mission_validate(
        document: Any,
        format: str | None = None,
        base_uri: str | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "mission_validate")
            return services.missions.validate(document, format, base_uri, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("mission_inspect"))
    async def mission_inspect(
        mission: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "mission_inspect")
            return services.missions.inspect(mission, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("mission_compile"))
    async def mission_compile(
        document: Any,
        format: str | None = None,
        persist: bool = True,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "mission_compile")
            return services.missions.compile(document, format, persist, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("mission_apply_overrides"))
    async def mission_apply_overrides(
        mission: dict[str, Any],
        overrides: list[dict[str, Any]],
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "mission_apply_overrides")
            return services.missions.apply_overrides(mission, overrides, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("simulation_run"))
    async def simulation_run(
        mission: dict[str, Any],
        solver_override: dict[str, Any] | None = None,
        save_history: bool = True,
        output_interval_s: float | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "simulation_run")
            await progress.set_message("propagating")
            return services.runs.run(
                mission, solver_override, save_history, output_interval_s, auth, progress,
            )
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("simulation_summary"))
    async def simulation_summary(
        run: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "simulation_summary")
            return services.runs.summary(run, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("simulation_events"))
    async def simulation_events(
        run: dict[str, Any],
        vehicle_id: str | None = None,
        event_id: str | None = None,
        start_time_s: float | None = None,
        end_time_s: float | None = None,
        page: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "simulation_events")
            return services.runs.events(run, {
                "vehicle_id": vehicle_id, "event_id": event_id,
                "start_time_s": start_time_s, "end_time_s": end_time_s,
            }, page_req(page), auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("simulation_state_at"))
    async def simulation_state_at(
        run: dict[str, Any],
        vehicle_id: str,
        time_s: float,
        fields: list[str] | None = None,
        interpolation: str | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "simulation_state_at")
            return services.runs.state_at(run, vehicle_id, time_s, fields, interpolation, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("simulation_vehicle_history"))
    async def simulation_vehicle_history(
        run: dict[str, Any],
        vehicle_id: str,
        fields: list[str] | None = None,
        start_time_s: float | None = None,
        end_time_s: float | None = None,
        stride: int | None = None,
        page: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "simulation_vehicle_history")
            return services.runs.history(run, vehicle_id, {
                "fields": fields, "start_time_s": start_time_s,
                "end_time_s": end_time_s, "stride": stride,
            }, page_req(page), auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("simulation_export_csv"))
    async def simulation_export_csv(
        run: dict[str, Any],
        vehicle_id: str,
        fields: list[str],
        start_time_s: float | None = None,
        end_time_s: float | None = None,
        sample_interval_s: float | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "simulation_export_csv")
            return services.runs.export_csv(run, vehicle_id, fields, start_time_s, end_time_s, sample_interval_s, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("simulation_compare_solvers"))
    async def simulation_compare_solvers(
        mission: dict[str, Any],
        solver_a: dict[str, Any],
        solver_b: dict[str, Any],
        metrics: list[str] | None = None,
        sample_interval_s: float | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "simulation_compare_solvers")
            await progress.set_message("comparing solvers")
            return services.runs.compare_solvers(
                mission, solver_a, solver_b, metrics, sample_interval_s, auth, progress,
            )
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("environment_sample"))
    async def environment_sample(
        mission: dict[str, Any],
        environment_id: str,
        time_s: float,
        position_i_m: list[float],
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "environment_sample")
            return services.runs.environment_sample(mission, environment_id, time_s, position_i_m, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("vehicle_flow_state"))
    async def vehicle_flow_state(
        run: dict[str, Any],
        vehicle_id: str,
        time_s: float,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "vehicle_flow_state")
            return services.runs.flow_state(run, vehicle_id, time_s, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("vehicle_forces"))
    async def vehicle_forces(
        run: dict[str, Any],
        vehicle_id: str,
        time_s: float,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "vehicle_forces")
            return services.runs.forces(run, vehicle_id, time_s, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("optimization_validate"))
    async def optimization_validate(
        mission: dict[str, Any],
        declaration: dict[str, Any],
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "optimization_validate")
            return services.analysis.optimize_validate(mission, declaration, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("optimization_evaluate"))
    async def optimization_evaluate(
        mission: dict[str, Any],
        design: dict[str, Any],
        declaration: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "optimization_evaluate")
            return services.analysis.optimize_evaluate(mission, design, declaration, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("optimization_run"))
    async def optimization_run(
        mission: dict[str, Any],
        declaration: dict[str, Any],
        settings: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "optimization_run")
            await progress.set_message("optimizing")
            return services.analysis.optimize_run(mission, declaration, settings, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("analysis_sweep"))
    async def analysis_sweep(
        mission: dict[str, Any],
        campaign_id: str,
        variables: list[dict[str, Any]],
        backend: dict[str, Any] | None = None,
        store_uri: str | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "analysis_sweep")
            await progress.set_message("sweep")
            return services.analysis.sweep(mission, campaign_id, variables, backend, store_uri, auth, progress)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("analysis_monte_carlo"))
    async def analysis_monte_carlo(
        mission: dict[str, Any],
        campaign_id: str,
        cases: int,
        seed: int,
        dispersions: list[dict[str, Any]],
        backend: dict[str, Any] | None = None,
        store_uri: str | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "analysis_monte_carlo")
            await progress.set_message("monte carlo")
            return services.analysis.monte_carlo(
                mission, campaign_id, cases, seed, dispersions, backend, store_uri, auth, progress,
            )
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("analysis_sobol"))
    async def analysis_sobol(
        mission: dict[str, Any],
        campaign_id: str,
        variables: list[dict[str, Any]],
        base_samples: int,
        seed: int,
        backend: dict[str, Any] | None = None,
        store_uri: str | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "analysis_sobol")
            await progress.set_message("sobol")
            return services.analysis.sobol(
                mission, campaign_id, variables, base_samples, seed, backend, store_uri, auth, progress,
            )
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("analysis_optimization_batch"))
    async def analysis_optimization_batch(
        mission: dict[str, Any],
        campaign_id: str,
        declaration: dict[str, Any],
        starts: list[dict[str, Any]],
        backend: dict[str, Any] | None = None,
        store_uri: str | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "analysis_optimization_batch")
            await progress.set_message("multistart")
            return services.analysis.optimization_batch(
                mission, campaign_id, declaration, starts, backend, store_uri, auth, progress,
            )
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("analysis_status"))
    async def analysis_status(
        campaign: dict[str, Any],
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "analysis_status")
            return services.analysis.status(campaign["campaign_id"], auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("analysis_cases"))
    async def analysis_cases(
        campaign: dict[str, Any],
        status: str | None = None,
        page: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "analysis_cases")
            return services.analysis.cases(campaign["campaign_id"], status, page_req(page), auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("analysis_failures"))
    async def analysis_failures(
        campaign: dict[str, Any],
        error_code: str | None = None,
        page: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "analysis_failures")
            return services.analysis.failures(campaign["campaign_id"], error_code, page_req(page), auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("analysis_case_replay"))
    async def analysis_case_replay(
        campaign: dict[str, Any],
        case_id: str,
        solver_override: dict[str, Any] | None = None,
        save_history: bool = True,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "analysis_case_replay")
            return services.analysis.replay(campaign["campaign_id"], case_id, solver_override, save_history, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("data_catalog_list"))
    async def data_catalog_list(
        dataset_id_prefix: str | None = None,
        kind: str | None = None,
        page: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "data_catalog_list")
            return services.data.catalog(dataset_id_prefix, kind, page_req(page), auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("data_table_query"))
    async def data_table_query(
        dataset_id: str,
        version: str,
        coordinates: dict[str, float],
        outputs: list[str] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "data_table_query")
            return services.data.query(dataset_id, version, coordinates, outputs, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("data_validity_check"))
    async def data_validity_check(
        dataset_id: str,
        version: str,
        coordinates: dict[str, float],
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "data_validity_check")
            return services.data.validity(dataset_id, version, coordinates, auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("verification_builtin"))
    async def verification_builtin(
        include_external_manifests: bool = False,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "verification_builtin")
            await progress.set_message("verification")
            return services.verification.builtin(include_external_manifests, auth, progress)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("verification_compare_csv"))
    async def verification_compare_csv(
        reference_artifact_id: str,
        actual_artifact_id: str,
        time_column: str,
        channels: list[str],
        tolerances: dict[str, Any],
        time_alignment: dict[str, Any] | str | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "verification_compare_csv")
            return services.verification.compare_csv(
                reference_artifact_id, actual_artifact_id, time_column, channels, tolerances, time_alignment, auth,
            )
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("verification_compare_runs"))
    async def verification_compare_runs(
        run_a: dict[str, Any],
        run_b: dict[str, Any],
        vehicle_id: str,
        fields: list[str],
        sample_interval_s: float,
        tolerances: dict[str, Any],
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "verification_compare_runs")
            await progress.set_message("compare runs")
            return services.verification.compare_runs(
                run_a, run_b, vehicle_id, fields, sample_interval_s, tolerances, auth, progress,
            )
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("verification_convergence"))
    async def verification_convergence(
        mission: dict[str, Any],
        solver_family: str,
        refinements: list[dict[str, Any]],
        metrics: list[str],
        reference_mode: str | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
        progress: Progress = Progress(),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "verification_convergence")
            await progress.set_message("convergence")
            return services.verification.convergence(
                mission, solver_family, refinements, metrics, reference_mode, auth, progress,
            )
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("plugin_list"))
    async def plugin_list(
        capability_category: str | None = None,
        page: dict[str, Any] | None = None,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "plugin_list")
            return services.plugins.list(capability_category, page_req(page), auth)
        except Exception as exc:
            return fail(exc, auth)

    @mcp.tool(**_kw("plugin_inspect"))
    async def plugin_inspect(
        plugin_id: str,
        services: AppServices = Depends(get_services),
        auth: AuthorizationContext = Depends(get_auth),
    ) -> dict[str, Any]:
        try:
            _guard(auth, "plugin_inspect")
            return services.plugins.inspect(plugin_id, auth)
        except Exception as exc:
            return fail(exc, auth)
