from __future__ import annotations

import json
import os
from copy import deepcopy
from functools import lru_cache
from pathlib import Path
from typing import Any


def contract_root() -> Path:
    env = os.environ.get("UNIFLIGHT_MCP_CONTRACT_ROOT")
    if env:
        candidate = Path(env)
        if (candidate / "TOOL_POLICY.json").is_file():
            return candidate
    here = Path(__file__).resolve()
    for candidate in (
        here.parents[2] / "mcp",
        Path.cwd() / "mcp",
        here.parent / "contracts",
    ):
        if (candidate / "TOOL_POLICY.json").is_file():
            return candidate
    raise FileNotFoundError("UniFlight MCP contract directory not found")


@lru_cache(maxsize=None)
def load_json(rel: str) -> dict[str, Any]:
    return json.loads((contract_root() / rel).read_text(encoding="utf-8"))


def tool_schema(name: str, kind: str) -> dict[str, Any]:
    return load_json(f"schemas/tools/{name}.{kind}.json")


def _resolve_pointer(doc: dict[str, Any], pointer: str) -> Any:
    node: Any = doc
    for part in pointer.strip("/").split("/"):
        if not part:
            continue
        if part.startswith("$"):
            part = part
        node = node[part]
    return node


def resolve_schema(schema: dict[str, Any], *, base: Path | None = None) -> dict[str, Any]:
    """Inline local $ref values so FastMCP/jsonschema can validate oneOf envelopes."""
    origin = base or contract_root() / "schemas" / "tools" / "_tool.json"

    def walk(node: Any, current: Path) -> Any:
        if isinstance(node, list):
            return [walk(item, current) for item in node]
        if not isinstance(node, dict):
            return node
        ref = node.get("$ref")
        if isinstance(ref, str):
            file_part, _, fragment = ref.partition("#")
            target_path = (current.parent / file_part).resolve() if file_part else current
            target = json.loads(target_path.read_text(encoding="utf-8"))
            resolved = _resolve_pointer(target, fragment) if fragment else target
            return walk(deepcopy(resolved), target_path)
        return {key: walk(value, current) for key, value in node.items()}

    return walk(deepcopy(schema), origin)


@lru_cache(maxsize=None)
def tool_schema_resolved(name: str, kind: str) -> dict[str, Any]:
    return resolve_schema(tool_schema(name, kind))


def tool_policy() -> dict[str, Any]:
    return load_json("TOOL_POLICY.json")["tools"]


def tool_registry() -> dict[str, Any]:
    return load_json("TOOL_REGISTRY.json")["tools"]


def policy_for(name: str) -> dict[str, Any]:
    return tool_policy()[name]
