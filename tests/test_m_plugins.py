from __future__ import annotations

from types import MappingProxyType, SimpleNamespace
import numpy as np
import pytest

from uniflight import (
    PLUGIN_API_VERSION, PluginDescriptor, PluginManager, PluginRegistrar,
    PluginRequirement, PluginCompatibilityError, PluginRequirementError,
    MissionRegistry, MissionCompiler, MissionDocument, MissionValidationError,
    MissionCompilationError, UniverseMutation,
)
from uniflight import plugins as plugin_module


class _EPs(list):
    def select(self, *, group):
        return list(self) if group == "uniflight.plugins" else []


class _EP:
    def __init__(self, name, obj, value="fake:plugin"):
        self.name=name; self._obj=obj; self.value=value
        self.dist=SimpleNamespace(metadata={"Name":f"dist-{name}"},name=f"dist-{name}")
    def load(self): return self._obj


def _descriptor(version="1.0.0", api=PLUGIN_API_VERSION):
    def register(r):
        r.register("propulsion","accel",lambda spec,ctx: float(spec["accel"]),description="fake model")
        def dyn(spec,ctx):
            schema=ctx["schema"]; psl=schema.sl("position"); vsl=schema.sl("velocity"); msl=schema.sl("mass")
            accel=float(ctx["models"][spec["model"]])
            def rhs(t,y):
                dy=np.zeros_like(y); dy[psl]=y[vsl]; dy[vsl]=np.array([accel,0,0]); dy[msl]=0.0; return dy
            return rhs
        r.register("dynamics","dyn",dyn)
        r.register("output","vx",lambda spec,ctx: float(ctx["result"].final_vehicles[ctx["spec"]["vehicle"]].schema.unpack(ctx["result"].final_vehicles[ctx["spec"]["vehicle"]].state)["velocity"][0]))
        r.register("guard","time",lambda spec,ctx: (lambda t,y: float(t-float(spec["time"]))))
        r.register("event_action","remove",lambda spec,ctx: (lambda event_ctx: UniverseMutation(remove=(ctx["vehicle_id"],),note="fake remove")))
    return PluginDescriptor("fake.plugin",version,register,api_version=api,description="test plugin")


def _patch_eps(monkeypatch, *eps):
    monkeypatch.setattr(plugin_module.importlib_metadata,"entry_points",lambda: _EPs(eps))


def _mission(*, action=False):
    raw={
        "format_version":"1.0",
        "mission":{"id":"m-plugin-test","t_span":[0.0,1.0]},
        "plugins":[{"id":"fake.plugin","version":"1.0.0"}],
        "models":{"prop":{"category":"propulsion","type":"fake.plugin:accel","config":{"accel":2.0}}},
        "bodies":{"b":{"type":"spherical","mu":1e10,"radius":1e6}},
        "vehicles":{"v":{"body":"b","initial":{"dof":3,"state":{"position":[1e6,0,0],"velocity":[0,0,0],"mass":10}},
            "phases":[{"name":"p","dof":3,"dynamics":{"type":"fake.plugin:dyn","config":{"model":"prop"}}}] }},
        "outputs":[{"name":"vx","type":"fake.plugin:vx","vehicle":"v"}],
    }
    if action:
        raw["events"]=[{"name":"rm","vehicle":"v","guard":{"type":"fake.plugin:time","config":{"time":0.5},"direction":1},
                        "action":{"type":"fake.plugin:remove","config":{}}}]
        raw["outputs"]=[{"name":"count","type":"vehicle_count"}]
    return raw


def test_descriptor_and_registrar_namespace_capabilities():
    reg=MissionRegistry(); desc=_descriptor(); desc.register(PluginRegistrar(reg,desc))
    assert "fake.plugin:accel" in reg.available("propulsion")
    info=reg.registration("propulsion","fake.plugin:accel")
    assert info.owner=="fake.plugin" and info.owner_version=="1.0.0"


def test_registry_prevents_cross_owner_replacement():
    reg=MissionRegistry(); reg.register("body","x",lambda s,c:None,owner="core")
    with pytest.raises(KeyError,match="cross-owner"):
        reg.register("body","x",lambda s,c:None,replace=True,owner="other",owner_version="1")


def test_plugin_manager_lazy_discovery(monkeypatch):
    _patch_eps(monkeypatch,_EP("fake.plugin",lambda:_descriptor()))
    mgr=PluginManager(); found=mgr.discover()
    assert tuple(found)==("fake.plugin",) and not mgr.loaded


def test_plugin_manager_loads_exact_version(monkeypatch):
    _patch_eps(monkeypatch,_EP("fake.plugin",lambda:_descriptor()))
    mgr=PluginManager(); reg=MissionRegistry()
    loaded=mgr.load_requirements([PluginRequirement("fake.plugin","1.0.0")],reg)
    assert loaded[0].descriptor.api_version==PLUGIN_API_VERSION
    assert "fake.plugin:dyn" in reg.available("dynamics")


def test_plugin_version_mismatch_is_rejected(monkeypatch):
    _patch_eps(monkeypatch,_EP("fake.plugin",lambda:_descriptor("2.0.0")))
    with pytest.raises(PluginRequirementError,match="version mismatch"):
        PluginManager().load_requirements([PluginRequirement("fake.plugin","1.0.0")],MissionRegistry())


def test_plugin_api_mismatch_is_rejected(monkeypatch):
    _patch_eps(monkeypatch,_EP("fake.plugin",lambda:_descriptor(api="9.0")))
    with pytest.raises(PluginCompatibilityError,match="targets API"):
        PluginManager().load("fake.plugin",MissionRegistry())


def test_optional_missing_plugin_is_allowed(monkeypatch):
    _patch_eps(monkeypatch)
    loaded=PluginManager().load_requirements([PluginRequirement("missing","1",required=False)],MissionRegistry())
    assert loaded==()


def test_namespaced_mission_capability_requires_explicit_plugin_entry():
    raw=_mission(); raw.pop("plugins")
    with pytest.raises(MissionValidationError,match="explicit plugins"):
        MissionDocument(raw)


def test_compiler_runs_plugin_model_dynamics_and_output(monkeypatch):
    _patch_eps(monkeypatch,_EP("fake.plugin",lambda:_descriptor()))
    compiler=MissionCompiler(plugin_manager=PluginManager())
    rep=compiler.compile(MissionDocument(_mission())).run()
    assert rep.success and rep.outputs["vx"]==pytest.approx(2.0,rel=1e-8)
    assert rep.plugin_inventory==(("fake.plugin","1.0.0",PLUGIN_API_VERSION),)


def test_plugin_event_guard_and_action_can_change_topology(monkeypatch):
    _patch_eps(monkeypatch,_EP("fake.plugin",lambda:_descriptor()))
    rep=MissionCompiler(plugin_manager=PluginManager()).compile(MissionDocument(_mission(action=True))).run()
    assert rep.success and rep.outputs["count"]==0.0
    assert rep.events[0]["note"]=="fake remove"


def test_compiler_wraps_plugin_compatibility_errors(monkeypatch):
    _patch_eps(monkeypatch,_EP("fake.plugin",lambda:_descriptor(api="2.0")))
    with pytest.raises(MissionCompilationError,match="targets API"):
        MissionCompiler(plugin_manager=PluginManager()).compile(MissionDocument(_mission()))


def test_registry_inventory_is_deterministic():
    reg=MissionRegistry(); reg.register("output","b",lambda s,c:0); reg.register("body","a",lambda s,c:0)
    rows=reg.inventory()
    assert [(r["category"],r["type"]) for r in rows]==[("body","a"),("output","b")]
