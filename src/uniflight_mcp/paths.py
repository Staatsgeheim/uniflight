from __future__ import annotations

from pathlib import Path

from .errors import DomainError


def resolve_under(path: str | Path, roots: tuple[Path, ...]) -> Path:
    candidate = Path(path).expanduser()
    try:
        resolved = candidate.resolve(strict=False)
    except OSError as exc:
        raise DomainError("INVALID_REQUEST", "path could not be resolved", path=str(path)) from exc
    if resolved.is_symlink():
        raise DomainError("INVALID_REQUEST", "symlinks are not accepted", path=str(path))
    for root in roots:
        root_r = root.resolve()
        if resolved == root_r or root_r in resolved.parents:
            return resolved
    raise DomainError("INVALID_REQUEST", "path is outside allowlisted roots", path=str(path))


def reject_traversal(text: str) -> None:
    if ".." in Path(text).parts or text.startswith(("~", "\\")):
        raise DomainError("INVALID_REQUEST", "path traversal is not allowed", path=text)
