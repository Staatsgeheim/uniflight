from __future__ import annotations

import os
import time


_CROCKFORD = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"


def ulid() -> str:
    ts = int(time.time() * 1000)
    entropy = int.from_bytes(os.urandom(10), "big")
    value = (ts << 80) | entropy
    chars = ["0"] * 26
    for i in range(25, -1, -1):
        chars[i] = _CROCKFORD[value & 31]
        value >>= 5
    return "".join(chars)


def new_id(prefix: str) -> str:
    return f"{prefix}_{ulid()}"


def mission_id() -> str:
    return new_id("mis")


def run_id() -> str:
    return new_id("run")


def verification_id() -> str:
    return new_id("ver")


def artifact_id() -> str:
    return new_id("art")


def snapshot_id() -> str:
    return new_id("snap")
