# Milestone A Verification Matrix

| ID | Case | Implemented test | Status |
|---|---|---|---|
| 001 | Tsiolkovsky | `tests/test_rocket.py` | PASS |
| 002 | Kepler two-body | `tests/test_gravity.py::test_002_kepler_two_body_invariants` | PASS |
| 003 | Vacuum free fall | `tests/test_gravity.py::test_003_vacuum_radial_free_fall_short_time_against_constant_g_limit` | PASS |
| 004 | Quaternion constant rate | `tests/test_attitude.py` | PASS |
| 011 | Event root | `tests/test_events.py::test_011_event_root_time` | PASS |
| 012 | Frame round-trip | `tests/test_state_and_frames.py::test_012_frame_round_trip_near_machine_precision` | PASS |

Additional kernel tests cover state-schema round trips/immutability and non-terminal jump-map handling.

Verification command:

```bash
PYTHONPATH=src pytest
```

Current result: **8 passed**.
