from __future__ import annotations

"""Milestone N structured analysis result storage.

SQLite is used intentionally: it is part of the Python standard library,
portable, inspectable with common tools, and provides transactional checkpoint
semantics without inventing a custom binary format. Worker processes return
results to the parent; only the campaign coordinator writes the database.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping
import json
import sqlite3
import time


def _json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


@dataclass(frozen=True, slots=True)
class StoredCase:
    case_id: str
    index: int
    kind: str
    status: str
    parameters: Mapping[str, Any]
    metrics: Mapping[str, Any]
    error: str | None
    elapsed_seconds: float | None


class SQLiteResultStore:
    """Transactional campaign/result store and restart checkpoint."""

    def __init__(self, path: str | Path):
        self.path = Path(path).resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._connection = sqlite3.connect(self.path)
        self._connection.execute("PRAGMA journal_mode=WAL")
        self._connection.execute("PRAGMA synchronous=NORMAL")
        self._initialize()

    def _initialize(self) -> None:
        with self._connection:
            self._connection.execute("""
                CREATE TABLE IF NOT EXISTS campaigns (
                    campaign_id TEXT PRIMARY KEY,
                    kind TEXT NOT NULL,
                    mission_sha256 TEXT,
                    metadata_json TEXT NOT NULL,
                    created_unix REAL NOT NULL,
                    updated_unix REAL NOT NULL
                )
            """)
            self._connection.execute("""
                CREATE TABLE IF NOT EXISTS cases (
                    campaign_id TEXT NOT NULL,
                    case_id TEXT NOT NULL,
                    case_index INTEGER NOT NULL,
                    kind TEXT NOT NULL,
                    status TEXT NOT NULL,
                    parameters_json TEXT NOT NULL,
                    metrics_json TEXT NOT NULL,
                    error TEXT,
                    elapsed_seconds REAL,
                    updated_unix REAL NOT NULL,
                    PRIMARY KEY (campaign_id, case_id),
                    FOREIGN KEY (campaign_id) REFERENCES campaigns(campaign_id)
                )
            """)
            self._connection.execute(
                "CREATE INDEX IF NOT EXISTS idx_cases_campaign_status ON cases(campaign_id,status)"
            )

    def close(self) -> None:
        self._connection.close()

    def __enter__(self) -> "SQLiteResultStore":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def begin_campaign(self, campaign_id: str, kind: str, *, mission_sha256: str | None = None,
                       metadata: Mapping[str, Any] | None = None) -> None:
        now = time.time()
        payload = _json(dict(metadata or {}))
        existing = self._connection.execute(
            "SELECT kind, mission_sha256, metadata_json FROM campaigns WHERE campaign_id=?", (campaign_id,)
        ).fetchone()
        if existing is not None:
            old_kind, old_sha, _old_meta = existing
            if old_kind != kind or old_sha != mission_sha256:
                raise ValueError("campaign_id already exists with incompatible kind/mission")
            with self._connection:
                self._connection.execute(
                    "UPDATE campaigns SET updated_unix=? WHERE campaign_id=?", (now, campaign_id)
                )
            return
        with self._connection:
            self._connection.execute(
                "INSERT INTO campaigns VALUES (?,?,?,?,?,?)",
                (campaign_id, kind, mission_sha256, payload, now, now),
            )

    def completed_case_ids(self, campaign_id: str) -> set[str]:
        rows = self._connection.execute(
            "SELECT case_id FROM cases WHERE campaign_id=? AND status='completed'", (campaign_id,)
        )
        return {str(r[0]) for r in rows}

    def write_case(self, campaign_id: str, case: StoredCase) -> None:
        now = time.time()
        with self._connection:
            self._connection.execute("""
                INSERT INTO cases
                (campaign_id,case_id,case_index,kind,status,parameters_json,metrics_json,error,elapsed_seconds,updated_unix)
                VALUES (?,?,?,?,?,?,?,?,?,?)
                ON CONFLICT(campaign_id,case_id) DO UPDATE SET
                    case_index=excluded.case_index,
                    kind=excluded.kind,
                    status=excluded.status,
                    parameters_json=excluded.parameters_json,
                    metrics_json=excluded.metrics_json,
                    error=excluded.error,
                    elapsed_seconds=excluded.elapsed_seconds,
                    updated_unix=excluded.updated_unix
            """, (
                campaign_id, case.case_id, int(case.index), case.kind, case.status,
                _json(dict(case.parameters)), _json(dict(case.metrics)), case.error,
                case.elapsed_seconds, now,
            ))
            self._connection.execute(
                "UPDATE campaigns SET updated_unix=? WHERE campaign_id=?", (now, campaign_id)
            )

    def cases(self, campaign_id: str) -> tuple[StoredCase, ...]:
        rows = self._connection.execute("""
            SELECT case_id,case_index,kind,status,parameters_json,metrics_json,error,elapsed_seconds
            FROM cases WHERE campaign_id=? ORDER BY case_index,case_id
        """, (campaign_id,)).fetchall()
        return tuple(StoredCase(
            str(cid), int(idx), str(kind), str(status),
            json.loads(params), json.loads(metrics), error,
            None if elapsed is None else float(elapsed),
        ) for cid, idx, kind, status, params, metrics, error, elapsed in rows)

    def campaign_metadata(self, campaign_id: str) -> Mapping[str, Any]:
        row = self._connection.execute(
            "SELECT kind,mission_sha256,metadata_json,created_unix,updated_unix FROM campaigns WHERE campaign_id=?",
            (campaign_id,),
        ).fetchone()
        if row is None:
            raise KeyError(campaign_id)
        return {
            "campaign_id": campaign_id,
            "kind": row[0],
            "mission_sha256": row[1],
            "metadata": json.loads(row[2]),
            "created_unix": row[3],
            "updated_unix": row[4],
        }

    def summary(self, campaign_id: str) -> Mapping[str, Any]:
        rows = self._connection.execute(
            "SELECT status,COUNT(*) FROM cases WHERE campaign_id=? GROUP BY status", (campaign_id,)
        ).fetchall()
        counts = {str(k): int(v) for k, v in rows}
        return {**self.campaign_metadata(campaign_id), "case_counts": counts,
                "total_cases": int(sum(counts.values()))}

    def export_json(self, campaign_id: str, path: str | Path) -> Path:
        target = Path(path)
        payload = {
            "campaign": dict(self.campaign_metadata(campaign_id)),
            "cases": [
                {
                    "case_id": c.case_id, "index": c.index, "kind": c.kind,
                    "status": c.status, "parameters": dict(c.parameters),
                    "metrics": dict(c.metrics), "error": c.error,
                    "elapsed_seconds": c.elapsed_seconds,
                } for c in self.cases(campaign_id)
            ],
        }
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False)+"\n", encoding="utf-8")
        return target
