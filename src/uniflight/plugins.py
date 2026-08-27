from __future__ import annotations

"""Milestone M stable public plugin architecture.

Plugins are trusted, in-process Python code. They are discovered through the
``uniflight.plugins`` importlib.metadata entry-point group.  Each entry point
must resolve to either a :class:`PluginDescriptor`, a callable returning one,
or an object exposing ``plugin_descriptor``.

Third-party capability names are always namespaced as ``<plugin-id>:<name>``.
This prevents a plugin from silently replacing core models or another plugin.
"""

from dataclasses import dataclass, field
from importlib import metadata as importlib_metadata
from types import MappingProxyType
from typing import Any, Callable, Iterable, Mapping
import re


PLUGIN_API_VERSION = "1.0"
PLUGIN_ENTRY_POINT_GROUP = "uniflight.plugins"

# Stable namespaces.  Model categories may be consumed directly by plugins or
# by future core assemblers; compiler-facing categories participate directly in
# the MDL runtime today.
PLUGIN_CAPABILITY_CATEGORIES = frozenset({
    "body", "atmosphere", "environment", "solver", "dynamics", "guard",
    "event_action", "output", "optimizer", "dataset_loader",
    "gravity", "aero", "aerothermal", "propulsion", "gnc", "sensor",
    "actuator", "subsystem", "terrain", "material", "chemistry",
})

_ID_RE = re.compile(r"^[a-z0-9][a-z0-9._-]*$")
_NAME_RE = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9._-]*$")


class PluginError(RuntimeError):
    pass


class PluginDiscoveryError(PluginError):
    pass


class PluginCompatibilityError(PluginError):
    pass


class PluginRequirementError(PluginError):
    pass


@dataclass(frozen=True, slots=True)
class PluginDescriptor:
    plugin_id: str
    version: str
    register: Callable[["PluginRegistrar"], None]
    api_version: str = PLUGIN_API_VERSION
    description: str = ""
    homepage: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        pid = str(self.plugin_id)
        ver = str(self.version)
        api = str(self.api_version)
        if not _ID_RE.fullmatch(pid):
            raise ValueError(f"invalid plugin_id {pid!r}")
        if not ver:
            raise ValueError("plugin version must be non-empty")
        if not callable(self.register):
            raise TypeError("plugin register must be callable")
        object.__setattr__(self, "plugin_id", pid)
        object.__setattr__(self, "version", ver)
        object.__setattr__(self, "api_version", api)
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))


@dataclass(frozen=True, slots=True)
class CapabilityRegistration:
    category: str
    type_name: str
    factory: Callable[..., Any]
    owner: str
    owner_version: str
    description: str = ""
    validator: Callable[[Mapping[str, Any]], None] | None = None


class PluginRegistrar:
    """Namespaced registration facade exposed to third-party plugins."""

    def __init__(self, registry: Any, descriptor: PluginDescriptor) -> None:
        self._registry = registry
        self.descriptor = descriptor

    def register(
        self,
        category: str,
        name: str,
        factory: Callable[..., Any],
        *,
        description: str = "",
        validator: Callable[[Mapping[str, Any]], None] | None = None,
    ) -> str:
        category = str(category)
        name = str(name)
        if category not in PLUGIN_CAPABILITY_CATEGORIES:
            raise PluginError(f"unsupported plugin capability category {category!r}")
        if not _NAME_RE.fullmatch(name):
            raise PluginError(f"invalid plugin capability name {name!r}")
        fq_name = f"{self.descriptor.plugin_id}:{name}"
        self._registry.register(
            category, fq_name, factory,
            owner=self.descriptor.plugin_id,
            owner_version=self.descriptor.version,
            description=description,
            validator=validator,
        )
        return fq_name


@dataclass(frozen=True, slots=True)
class PluginRequirement:
    plugin_id: str
    version: str
    required: bool = True


@dataclass(frozen=True, slots=True)
class LoadedPlugin:
    descriptor: PluginDescriptor
    entry_point: str = ""
    distribution: str = ""

    def inventory_tuple(self) -> tuple[str, str, str]:
        return (self.descriptor.plugin_id, self.descriptor.version, self.descriptor.api_version)


def _coerce_descriptor(obj: Any) -> PluginDescriptor:
    if isinstance(obj, PluginDescriptor):
        return obj
    if callable(obj):
        obj = obj()
        if isinstance(obj, PluginDescriptor):
            return obj
    desc = getattr(obj, "plugin_descriptor", None)
    if callable(desc):
        desc = desc()
    if isinstance(desc, PluginDescriptor):
        return desc
    raise PluginDiscoveryError(
        "plugin entry point must resolve to PluginDescriptor, callable returning one, "
        "or object exposing plugin_descriptor"
    )


class PluginManager:
    """Discover, compatibility-check and load UniFlight plugins.

    Discovery is lazy: installed entry points are enumerated without importing
    plugin code. Only requested plugins (or explicit ``load_all``) are imported.
    """

    def __init__(self, *, entry_point_group: str = PLUGIN_ENTRY_POINT_GROUP) -> None:
        self.entry_point_group = entry_point_group
        self._entry_points: dict[str, Any] | None = None
        self._loaded: dict[str, LoadedPlugin] = {}

    def discover(self, *, refresh: bool = False) -> Mapping[str, Any]:
        if self._entry_points is not None and not refresh:
            return MappingProxyType(dict(self._entry_points))
        eps = importlib_metadata.entry_points()
        selected = eps.select(group=self.entry_point_group) if hasattr(eps, "select") else eps.get(self.entry_point_group, [])
        found: dict[str, Any] = {}
        for ep in selected:
            pid = str(ep.name)
            if pid in found:
                raise PluginDiscoveryError(f"duplicate installed plugin entry point {pid!r}")
            found[pid] = ep
        self._entry_points = found
        return MappingProxyType(dict(found))

    @property
    def loaded(self) -> Mapping[str, LoadedPlugin]:
        return MappingProxyType(dict(self._loaded))

    def load(self, plugin_id: str, registry: Any) -> LoadedPlugin:
        plugin_id = str(plugin_id)
        if plugin_id in self._loaded:
            return self._loaded[plugin_id]
        eps = self.discover()
        if plugin_id not in eps:
            raise PluginRequirementError(f"required UniFlight plugin {plugin_id!r} is not installed")
        ep = eps[plugin_id]
        try:
            descriptor = _coerce_descriptor(ep.load())
        except PluginError:
            raise
        except Exception as exc:
            raise PluginDiscoveryError(f"failed loading plugin {plugin_id!r}: {exc}") from exc
        if descriptor.plugin_id != plugin_id:
            raise PluginDiscoveryError(
                f"entry point {plugin_id!r} returned descriptor for {descriptor.plugin_id!r}"
            )
        if descriptor.api_version != PLUGIN_API_VERSION:
            raise PluginCompatibilityError(
                f"plugin {plugin_id!r} targets API {descriptor.api_version!r}; "
                f"UniFlight requires {PLUGIN_API_VERSION!r}"
            )
        registrar = PluginRegistrar(registry, descriptor)
        try:
            descriptor.register(registrar)
        except Exception as exc:
            raise PluginDiscoveryError(f"plugin {plugin_id!r} registration failed: {exc}") from exc
        dist = ""
        try:
            if getattr(ep, "dist", None) is not None:
                dist = str(ep.dist.metadata.get("Name", ""))
        except Exception:
            pass
        loaded = LoadedPlugin(descriptor, str(getattr(ep, "value", "")), dist)
        self._loaded[plugin_id] = loaded
        return loaded

    def load_requirements(self, requirements: Iterable[PluginRequirement], registry: Any) -> tuple[LoadedPlugin, ...]:
        out: list[LoadedPlugin] = []
        for req in requirements:
            try:
                loaded = self.load(req.plugin_id, registry)
            except PluginRequirementError:
                if req.required:
                    raise
                continue
            if loaded.descriptor.version != str(req.version):
                raise PluginRequirementError(
                    f"plugin {req.plugin_id!r} version mismatch: mission requires {req.version!r}, "
                    f"installed {loaded.descriptor.version!r}"
                )
            out.append(loaded)
        return tuple(out)

    def load_all(self, registry: Any) -> tuple[LoadedPlugin, ...]:
        return tuple(self.load(pid, registry) for pid in sorted(self.discover()))

    def inventory(self) -> tuple[tuple[str, str, str], ...]:
        return tuple(self._loaded[k].inventory_tuple() for k in sorted(self._loaded))


def installed_plugin_summary(manager: PluginManager | None = None) -> tuple[Mapping[str, str], ...]:
    manager = manager or PluginManager()
    rows = []
    for pid, ep in sorted(manager.discover().items()):
        rows.append(MappingProxyType({
            "plugin_id": pid,
            "entry_point": str(getattr(ep, "value", "")),
            "distribution": str(getattr(getattr(ep, "dist", None), "name", "") or ""),
        }))
    return tuple(rows)
