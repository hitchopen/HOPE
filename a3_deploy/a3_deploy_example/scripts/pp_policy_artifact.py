"""Routing hints for existing observation/command ABIs, never training names.

The native Runner --inspect-policy owns full compatibility validation. These
small hints are also used by historical logs whose Runner predates inspection.
"""

SMALL_STATION_COMMAND = "small_station_external_schedule_precommit_settle_v1"
SMALL_STATION_PLANNER = "fresh_schema2_absolute_small_station_v1"


def small_station_mode(metadata, inputs, outputs):
    if (metadata.get("hitter_pingpong_command_contract") != SMALL_STATION_COMMAND
            or metadata.get("hitter_pingpong_planner_commit_contract") != SMALL_STATION_PLANNER
            or metadata.get("hitter_pure_runtime_contract") != "rally_final_v2"
            or metadata.get("qdes_action_contract") != "v12_affine_safe_slew_qdes_v1"
            or metadata.get("hitter_pingpong_optional_reach_training_enabled") != "false"
            or outputs.get("actions") != [1, 31]
            or "core_action_shadow_owned" in outputs):
        return None
    obs = metadata.get("actor_obs_contract")
    if obs == "hitter_pure_112_headslots_vxy_reach_v1" and inputs.get("obs") == [1, 112]:
        return "small_station112"
    if (obs == "hitter_compact_execution_324_v1" and inputs.get("obs") == [1, 324]
            and metadata.get("compact_history_contract") == "current114_past3x70_policy_tick_repeat_reset_v1"
            and metadata.get("compact_sent_feedback_contract") == "last_successfully_sent_absolute_action_order_rad_v1"):
        return "compact324"
    return None
