# Archived Python planner imitation

`planner_imitate` is retained only as historical source material. The
`hope_planner` package is excluded from colcon by `COLCON_IGNORE`, and its
planner executables are not registered or supported.

Current simulation, Gate 3 and hardware workflows use the C++ runtime pair:

- `hope_ball_flight_packetizer`
- `hope_planner_cpp_node`

Use the current commands in [`../../SMOKE_TEST.md`](../../SMOKE_TEST.md)
for a transport/Planner smoke test, or
[`../../../docs/MODEL_21800.md`](../../../docs/MODEL_21800.md) for the
model_21800 MuJoCo closed loop. Do not use the historical Python imitation for
robot bring-up.
