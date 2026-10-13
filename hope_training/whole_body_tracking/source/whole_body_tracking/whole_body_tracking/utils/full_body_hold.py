"""v8 actual full-body HOLD imitation. Never a q_des-to-q objective.

One static, user-selected reference serves both swing sides. World translation
belongs to the current station command; it is not part of the pose reference.
All calculations are vectorized; cached references are mapped by joint name.
"""
import math
import torch

from .task_pose_reference import load_reference, reference_joints
from .ready_recovery import bounded_debt, recovery_gate, _log

GROUPS = {
    **{side + "_leg": [f"{side}_{j}_joint" for j in
       ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
       for side in ("left", "right")},
    "waist": [f"waist_{j}_joint" for j in ("yaw", "roll", "pitch")],
    **{side + "_arm": [f"{side}_{j}_joint" for j in
       ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")]
       for side in ("left", "right")},
}
ACTIVE_NAMES = tuple(n for names in GROUPS.values() for n in names)
BODY_NAMES = ("torso_Link", "left_ankle_roll_Link", "right_ankle_roll_Link",
              "left_wrist_yaw_Link", "right_wrist_yaw_Link")


def mean_and_worst(value):
    return .5 * value.mean(-1) + .5 * value.amax(-1)


def wrap_angle(value):
    return torch.atan2(torch.sin(value), torch.cos(value))


def rpy(quat):
    w, x, y, z = quat.unbind(-1)
    return torch.stack((torch.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                        torch.asin((2*(w*y-z*x)).clamp(-1, 1)),
                        torch.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))), -1)


def approach_factor(distance):
    # Never disable posture by standing far away. Legs retain 25% pressure while
    # travelling; full strength within 5 cm. Speed/contact cannot evade it.
    return .25 + .75 * ((.20-distance)/.15).clamp(0, 1)


def joint_costs(error):
    costs, rms, worst = [], {}, {}
    offset = 0
    for group, names in GROUPS.items():
        e = error[:, offset:offset+len(names)]
        margin = math.radians(1.5 if group == "waist" else 2.)
        scale = math.radians(6. if group == "waist" else 10.)
        costs.append(mean_and_worst(bounded_debt(e, margin, scale)))
        rms[group], worst[group] = e.square().mean(-1).sqrt(), e.abs().amax(-1)
        offset += len(names)
    return torch.stack(costs, -1), rms, worst


def quiet_gate(distance, max_joint_error, speed):
    # Quieting is a terminal settling goal, not a brake on necessary recovery.
    near = ((.10-distance)/.05).clamp(0, 1)
    pose = ((math.radians(15)-max_joint_error)/math.radians(10)).clamp(0, 1)
    slow = ((.20-speed)/.10).clamp(0, 1)
    return near * pose * slow


def phase_summaries(metrics):
    """Conditional snapshot statistics; absent phases are not zero-error poses."""
    result = {}
    for i, phase in enumerate(("hold", "strike", "recover")):
        mask = metrics["full_body_phase"] == i
        result[f"full_body_{phase}_fraction"] = mask.float().mean()
        if phase == "strike":
            continue  # static HOLD reference is deliberately inapplicable
        count = mask.sum()
        for group in GROUPS:
            value = metrics[f"full_body_{group}_error_rms_rad"]
            result[f"full_body_{phase}_{group}_error_rms_rad"] = torch.where(
                count > 0, ((value.square()*mask).sum()/count.clamp_min(1)).sqrt(),
                value.new_tensor(float("nan")))
    return result


def _state(env):
    cmd = env.command_manager.get_term("racket_target")
    robot = cmd.robot
    cache = getattr(env, "_full_body_hold_reference", None)
    if cache is None:
        ids, names = robot.find_joints(list(ACTIVE_NAMES), preserve_order=True)
        if list(names) != list(ACTIVE_NAMES):
            raise ValueError("full-body HOLD requires all 29 named active A3 joints")
        bodies = [cmd._resolve_body_index(n) for n in BODY_NAMES]
        if min(bodies) < 0:
            raise ValueError("full-body HOLD requires torso, feet and wrist bodies")
        pose = load_reference()
        ref = reference_joints(names, robot.data.joint_pos)
        limits = getattr(robot.data, "joint_pos_limits", None)
        if limits is not None:
            bounds = limits[0, ids]
            # Reject an unreachable task reference instead of rewarding hard stops.
            if bool(((ref <= bounds[:, 0]) | (ref >= bounds[:, 1])).any()):
                raise ValueError("HOLD reference must be strictly inside actual joint limits")
        cache = (ids, bodies, ref, pose)
        env._full_body_hold_reference = cache
    ids, bodies, ref, pose = cache
    return cmd, robot, ids, bodies, ref, pose


def _loaded_feet(env):
    sensor = env.scene.sensors["contact_forces"]
    feet = getattr(env, "_full_body_quiet_foot_ids", None)
    if feet is None:
        feet, _ = sensor.find_bodies(list(BODY_NAMES[1:3]), preserve_order=True)
        if len(feet) != 2:
            raise ValueError("full-body quiet requires both foot contact sensors")
        env._full_body_quiet_foot_ids = feet
    return (sensor.data.net_forces_w[:, feet, 2] > 20).all(-1)


def full_body_hold(env, start_steps=448000, ramp_steps=128000, cost_scale=1.8,
                   positive_kernel=False):
    if start_steps < 0 or ramp_steps <= 0:
        raise ValueError("invalid full-body HOLD curriculum")
    if not math.isfinite(cost_scale) or cost_scale <= 0:
        raise ValueError("full-body pose cost_scale must be finite and positive")
    cmd, robot, ids, bodies, ref, pose = _state(env)
    motion = env.command_manager.get_term("motion")
    gate = recovery_gate(motion.in_hold, cmd.time_to_strike)
    scale = min(1., max(0., (env.common_step_counter-start_steps)/ramp_steps))
    q = robot.data.joint_pos[:, ids]
    error = q-ref
    costs, rms, worst = joint_costs(error)
    distance = (cmd.base_target_pos_w-cmd.base_pos_w[:, :2]).norm(dim=-1)
    leg_factor = approach_factor(distance)
    factors = torch.stack((leg_factor, leg_factor, torch.ones_like(gate),
                           torch.ones_like(gate), torch.ones_like(gate)), -1)
    # Reducing leg stiffness during approach must NOT make retreating improve
    # reward. Add the complementary distance debt: d' = f*d + (1-f).
    joints = mean_and_worst(costs*factors + (1-factors))
    pos = robot.data.body_pos_w[:, bodies] - env.scene.env_origins[:, None, :]
    angles = rpy(robot.data.body_quat_w[:, bodies])
    metrics = pose["metrics"]
    torso_ref = q.new_tensor(metrics["torso_rpy_deg"]) * math.pi/180
    torso_error = wrap_angle(angles[:, 0]-torso_ref)
    torso = mean_and_worst(bounded_debt(torso_error, math.radians(1), math.radians(6)))
    foot_ref = q.new_tensor(metrics["foot_rpy_deg"])*math.pi/180
    foot_error = wrap_angle(angles[:, 1:3]-foot_ref)
    feet_angle = mean_and_worst(bounded_debt(foot_error.flatten(1), math.radians(3), math.radians(10)))
    # Signed lateral separation in the fixed table frame: crossed feet and a
    # rotated whole robot cannot masquerade as a correct stance width.
    width_error = pos[:, 1, 1]-pos[:, 2, 1]-metrics["foot_center_width_cm"]/100
    feet = mean_and_worst(torch.stack((feet_angle,
        bounded_debt(width_error, .015, .05),
        bounded_debt(pos[:, 1, 0]-pos[:, 2, 0], .025, .05)), -1))*leg_factor + (1-leg_factor)
    height_error = robot.data.root_pos_w[:, 2]-env.scene.env_origins[:, 2]-metrics["pelvis_height_cm"]/100
    height = bounded_debt(height_error, .015, .04)*leg_factor + (1-leg_factor)
    hand_ref = q.new_tensor([metrics[s+"_wrist_height_cm"]/100 for s in ("left", "right")])
    hand_error = pos[:, 3:5, 2]-hand_ref
    hands = mean_and_worst(bounded_debt(hand_error, .02, .05))
    geometry = mean_and_worst(torch.stack((torso, feet, height, hands), -1))
    cost = .6*joints + .4*geometry
    # The default preserves v8's [-.8, 1] budget. v9 raises only the late
    # error slope through cost_scale; reference bonus and early legacy stay fixed.
    upper = error[:, 12:]  # waist + both arms, exactly the previous set
    legacy = torch.exp(-upper.square().mean(-1)/.5**2)*motion.in_hold.float()
    # v10 retains pressure on every incorrect pose without making imperfect
    # recovery a persistent negative score. Unlike clamping a linear reward,
    # the exponential remains strictly monotone even on large pose errors.
    pose_score = torch.exp(-cost_scale*cost) if positive_kernel else 1-cost_scale*cost
    reward = (1-scale)*legacy + scale*gate*pose_score
    qd = robot.data.joint_vel[:, ids]
    speed = robot.data.root_lin_vel_w[:, :2].norm(dim=-1)
    eligible = gate*_loaded_feet(env)*quiet_gate(distance, error.abs().amax(-1), speed)
    phase = torch.where(motion.in_hold, 0, torch.where(gate > 0, 2, 1))
    values = {"gate": gate, "phase": phase.float(), "reference_applicable": (gate > 0).float(),
              "curriculum": torch.full_like(gate, scale), "cost": cost,
              "max_joint_error_rad": error.abs().amax(-1), "width_error_m": width_error,
              "height_error_m": height_error, "hand_height_error_max_m": hand_error.abs().amax(-1),
              "foot_angle_error_max_rad": foot_error.abs().flatten(1).amax(-1),
              "torso_error_max_rad": torso_error.abs().amax(-1), "quiet_gate": eligible,
              "max_joint_speed_rad_s": qd.abs().amax(-1)}
    for group in GROUPS:
        values[group+"_error_rms_rad"] = rms[group]
        values[group+"_error_max_rad"] = worst[group]
    # Conditional metrics cannot turn an absent HOLD into a successful pose.
    eligible_hold = motion.in_hold.float()
    full = ((error.abs().amax(-1) <= math.radians(5)) & (torso_error.abs().amax(-1) <= math.radians(3))
            & (foot_error.abs().flatten(1).amax(-1) <= math.radians(5)) & (width_error.abs() <= .02)
            & (height_error.abs() <= .02) & (hand_error.abs().amax(-1) <= .03))
    values["hold_fraction"] = eligible_hold
    values["hold_pose_pass_fraction"] = ((full*eligible_hold).sum()/eligible_hold.sum().clamp_min(1)).expand_as(gate)
    for name, value in values.items():
        _log(cmd, "full_body_"+name, value)
    trace = getattr(env, "_task_pose_chain_trace", None)
    if trace is not None and env.common_step_counter % 64000 < 512:
        action = env.action_manager.get_term("joint_pos")
        columns = getattr(env, "_full_body_trace_columns", None)
        if columns is None:
            columns = [action._joint_names.index(n) for n in ACTIVE_NAMES]
            env._full_body_trace_columns = columns
        trace_values = dict(values)
        for i, name in enumerate(ACTIVE_NAMES):
            trace_values[name+"_reference_rad"] = ref[i].expand_as(gate)
            trace_values[name+"_actual_rad"] = q[:, i]
            trace_values[name+"_velocity_rad_s"] = qd[:, i]
            trace_values[name+"_qdes_rad"] = action._qdes_executed[:, columns[i]]
            trace_values[name+"_pd_torque_estimate_Nm"] = robot.data.applied_torque[:, ids[i]]
        trace.record(int(env.common_step_counter), env.episode_length_buf, trace_values)
    return reward


def full_body_quiet(env, start_steps=448000, ramp_steps=128000, state_independent=False,
                    pose_independent=False):
    """Penalize actual velocity; v9 cannot disable it by moving or missing the pose.

    The exogenous phase still protects impact and the first 0.5 s follow-through.
    This is a bounded soft cost, never a station/pose release condition. Legacy
    behavior remains the default for replaying saved v8 configurations.
    """
    if start_steps < 0 or ramp_steps <= 0:
        raise ValueError("invalid full-body quiet curriculum")
    cmd, robot, ids, _, ref, _ = _state(env)
    motion = env.command_manager.get_term("motion")
    gate = recovery_gate(motion.in_hold, cmd.time_to_strike)
    distance = (cmd.base_target_pos_w-cmd.base_pos_w[:, :2]).norm(dim=-1)
    error = (robot.data.joint_pos[:, ids]-ref).abs().amax(-1)
    speed = robot.data.root_lin_vel_w[:, :2].norm(dim=-1)
    scale = min(1., max(0., (env.common_step_counter-start_steps)/ramp_steps))
    debt = mean_and_worst(bounded_debt(robot.data.joint_vel[:, ids], .08, .30))
    # v10: release necessary travelling/balance motion, without letting one
    # bad wrist erase quieting at an otherwise settled station. This does not
    # gate full_body_hold's actual pose objective.
    pose_error = torch.zeros_like(error) if pose_independent else error
    state_gate = _loaded_feet(env)*quiet_gate(distance, pose_error, speed)
    effective_gate = gate if state_independent else gate*state_gate
    # Separate actual eligibility from full_body_hold's legacy quiet_gate diagnostic.
    _log(cmd, "full_body_quiet_effective_gate", scale*effective_gate)
    _log(cmd, "full_body_quiet_velocity_debt", debt)
    return scale*effective_gate*debt
