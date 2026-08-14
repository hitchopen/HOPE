# Shadow-mode status

The former `planner_imitate + wbc_runner` shadow tutorial belonged to the
retired Python Planner and an older observation contract. It is not a supported
model_21800 production path and has been removed to prevent a new machine from
starting the wrong Planner.

Current safe, non-hardware alternatives are:

- C++ Planner transport smoke without any Runner:
  [`SMOKE_TEST.md`](SMOKE_TEST.md).
- Complete C++ Planner + model_21800 native Runner + MuJoCo closed loop:
  [`docs/MODEL_21800.md`](../docs/MODEL_21800.md).

Real hardware uses the native Runner under the Foxglove lifecycle described in
[`docs/operations/foxglove_first_hardware_test.md`](../docs/operations/foxglove_first_hardware_test.md).
Do not treat an ad-hoc ROS node that merely observes robot state as equivalent
to that Runner. A new shadow feature must explicitly implement the current
110-D observation contract, consume the C++ Planner's schema-2 flat command,
and prove that it creates no motor-command publisher before it is documented
as supported.
