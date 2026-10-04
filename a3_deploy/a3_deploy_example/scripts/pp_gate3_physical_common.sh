#!/usr/bin/env bash
# Shared side-neutral physical Gate3 scenario.
#
# Each tuple is [x,y,z,vx,vy,vz] in the table-surface frame.  There is no
# forehand/backhand label.  With the production split/hysteresis and the live
# base pose, the planner autonomously selects six FH and six BH shots.  The
# lateral sequence asks for 0.19--0.25 m station changes in both directions
# while remaining inside the regulation table.
GATE3_PHYSICAL_SERVES_V1='[2.4,-1.2025,0.49,-3.0,0.0,2.2, 2.4,-0.8525,0.49,-3.0,0.0,2.2, 2.4,-1.4425,0.49,-3.0,0.0,2.2, 2.4,-0.7925,0.49,-3.0,0.0,2.2, 2.4,-1.4425,0.49,-3.0,0.0,2.2, 2.4,-0.7725,0.49,-3.0,0.0,2.2, 2.4,-1.4425,0.49,-3.0,0.0,2.2, 2.4,-0.8125,0.49,-3.0,0.0,2.2, 2.4,-1.4325,0.49,-3.0,0.0,2.2, 2.4,-0.7625,0.49,-3.0,0.0,2.2, 2.4,-1.4425,0.49,-3.0,0.0,2.2, 2.4,-0.7925,0.49,-3.0,0.0,2.2]'
GATE3_PHYSICAL_SERVES_FIRST_TWO='[2.4,-1.2025,0.49,-3.0,0.0,2.2, 2.4,-0.8525,0.49,-3.0,0.0,2.2]'
# Schema28 behavioral points sit inside the HOME-relative-y coarse axis
# prefilter. With HOME y=-0.7625 these yield FH offsets -.295/-.330 and BH
# offsets +.105/+.150, covering L1/L2 and both moving-foot signs without issuing
# the old, untrained FH offset -.440 target.  This does NOT certify a correlated
# complete question. Formal qualification separately requires each emitted
# position/velocity/normal/TTS tuple to match one exact receipt-bound bank row.
GATE3_HITTER_FH_L0='2.4,-1.0225,0.49,-3.0,0.0,2.2'
GATE3_HITTER_FH_L1='2.4,-1.0575,0.49,-3.0,0.0,2.2'
GATE3_HITTER_FH_L2='2.4,-1.0925,0.49,-3.0,0.0,2.2'
GATE3_HITTER_BH_L0='2.4,-0.7125,0.49,-3.0,0.0,2.2'
GATE3_HITTER_BH_L1='2.4,-0.6575,0.49,-3.0,0.0,2.2'
GATE3_HITTER_BH_L2='2.4,-0.6125,0.49,-3.0,0.0,2.2'
# The low variants use the same lateral cells but a different physical arc.
# With the Gate3/Planner physics tuple (drag=0.1220, e_h=0.64,
# e_v=0.9215), z_table=0.16 and vz=2.94 cross the fixed strike plane at
# world z ~=1.010 m after exactly one table bounce.  That is inside v5's H0
# support (FH <=1.057 m, BH <=1.049 m), about 8 cm below the old lane, and is
# still far enough above the 0.980 m lower box edge for live-fit noise.
GATE3_HITTER_FH_L0_LOW='2.4,-1.0225,0.16,-3.0,0.0,2.94'
GATE3_HITTER_FH_L1_LOW='2.4,-1.0575,0.16,-3.0,0.0,2.94'
GATE3_HITTER_FH_L2_LOW='2.4,-1.0925,0.16,-3.0,0.0,2.94'
GATE3_HITTER_BH_L0_LOW='2.4,-0.7125,0.16,-3.0,0.0,2.94'
GATE3_HITTER_BH_L1_LOW='2.4,-0.6575,0.16,-3.0,0.0,2.94'
GATE3_HITTER_BH_L2_LOW='2.4,-0.6125,0.16,-3.0,0.0,2.94'
# Fixed-HOME qualification has no commanded locomotion lane. Rows remain
# side-neutral; the Planner derives side and support intent from immutable HOME.
GATE3_PHYSICAL_SERVES_HITTER_CLEAN_FIXED_HOME_6="[$GATE3_HITTER_FH_L0, $GATE3_HITTER_BH_L0, $GATE3_HITTER_FH_L1, $GATE3_HITTER_BH_L1, $GATE3_HITTER_FH_L2, $GATE3_HITTER_BH_L2]"
# Twenty no-reset HOME shots retain FF/FB/BB/BF transitions while preserving
# the ordinary L0 ability that optional-reach training is required not to
# regress. Across all 26 shots each side has five L0 and four L1/L2 questions.
GATE3_PHYSICAL_SERVES_HITTER_RAPID_20="[$GATE3_HITTER_FH_L0, $GATE3_HITTER_FH_L1, $GATE3_HITTER_BH_L0, $GATE3_HITTER_BH_L1, $GATE3_HITTER_FH_L2, $GATE3_HITTER_BH_L2, $GATE3_HITTER_FH_L0, $GATE3_HITTER_BH_L0, $GATE3_HITTER_FH_L1, $GATE3_HITTER_BH_L1, $GATE3_HITTER_FH_L2, $GATE3_HITTER_BH_L2, $GATE3_HITTER_FH_L0, $GATE3_HITTER_BH_L0, $GATE3_HITTER_FH_L1, $GATE3_HITTER_BH_L1, $GATE3_HITTER_FH_L2, $GATE3_HITTER_BH_L2, $GATE3_HITTER_FH_L0, $GATE3_HITTER_BH_L0]"
GATE3_PHYSICAL_SERVES_HITTER_FIXED_HOME="${GATE3_PHYSICAL_SERVES_HITTER_CLEAN_FIXED_HOME_6%\]}, ${GATE3_PHYSICAL_SERVES_HITTER_RAPID_20#\[}"

# Six clean low balls establish both sides and all reach levels.  The 12-ball
# torture cycle then alternates low/nominal height, inner/outer reach, both
# sides, and includes FH->FH plus BH->BH transitions.  Repeating this cycle in
# one MOTION session accumulates recovery debt without resetting the plant.
GATE3_PHYSICAL_SERVES_HITTER_TORTURE_CLEAN_LOW_6="[$GATE3_HITTER_FH_L0_LOW, $GATE3_HITTER_BH_L0_LOW, $GATE3_HITTER_FH_L1_LOW, $GATE3_HITTER_BH_L1_LOW, $GATE3_HITTER_FH_L2_LOW, $GATE3_HITTER_BH_L2_LOW]"
GATE3_PHYSICAL_SERVES_HITTER_TORTURE_CYCLE_12="[$GATE3_HITTER_FH_L2_LOW, $GATE3_HITTER_BH_L2_LOW, $GATE3_HITTER_FH_L1, $GATE3_HITTER_BH_L1, $GATE3_HITTER_FH_L0_LOW, $GATE3_HITTER_FH_L2, $GATE3_HITTER_BH_L0_LOW, $GATE3_HITTER_BH_L2, $GATE3_HITTER_FH_L1_LOW, $GATE3_HITTER_BH_L1_LOW, $GATE3_HITTER_FH_L0, $GATE3_HITTER_BH_L0]"
# The task-complete phase is the same autonomous Gate3, extended to 26 shots:
# two full 12-shot station/side cycles plus the first FH/BH pair.
GATE3_PHYSICAL_SERVES_V17_TASK="${GATE3_PHYSICAL_SERVES_V1%\]}, ${GATE3_PHYSICAL_SERVES_V1#\[}"
GATE3_PHYSICAL_SERVES_V17_TASK="${GATE3_PHYSICAL_SERVES_V17_TASK%\]}, ${GATE3_PHYSICAL_SERVES_FIRST_TWO#\[}"

# RallyV8 has a different per-side strike plane and, especially, a much
# flatter backhand racket-velocity cone.  These are still side-neutral ball
# states: no row contains or commands a side.  The planner's live
# split/hysteresis chooses the side from the measured trajectory and base pose.
# The alternating vertical speeds make both naturally selected sides feasible
# under V8's exported boxes without --demo velocity substitution.
GATE3_PHYSICAL_SERVES_V8='[2.4,-1.2025,0.49,-3.0,0.0,2.0, 2.4,-0.8525,0.49,-3.0,0.0,4.0, 2.4,-1.4425,0.49,-3.0,0.0,2.0, 2.4,-0.7925,0.49,-3.0,0.0,4.0, 2.4,-1.4425,0.49,-3.0,0.0,2.0, 2.4,-0.7725,0.49,-3.0,0.0,4.0, 2.4,-1.4425,0.49,-3.0,0.0,2.0, 2.4,-0.8125,0.49,-3.0,0.0,4.0, 2.4,-1.4325,0.49,-3.0,0.0,2.0, 2.4,-0.7625,0.49,-3.0,0.0,4.0, 2.4,-1.4425,0.49,-3.0,0.0,2.0, 2.4,-0.7925,0.49,-3.0,0.0,4.0]'

gate3_apply_physical_arena_contract() {
  export PP_SERVES=12
  export PP_SERVES_LIST="$GATE3_PHYSICAL_SERVES_V1"
  export PP_RESET_Y=-0.7625
  export PP_LAND_X=2.055
  export PP_LAND_Y_FH=-0.7625
  export PP_LAND_Y_BH=-0.7625
  export PP_DTF_FH=0.50
  export PP_DTF_BH=0.50
  export PP_SPLIT_Y=-0.25
  export PP_SPLIT_HYST=0.04
  export PP_GATE3_VERDICT=certification
  export PP_ALLOW_RESCUE=0
  export PP_MAX_RESCUES=0
  export PP_EXTRA_ARGS=''
}

gate3_apply_v8_physical_arena_contract() {
  gate3_apply_physical_arena_contract
  export PP_SERVES_LIST="$GATE3_PHYSICAL_SERVES_V8"
  export PP_DTF_FH=0.45
  export PP_DTF_BH=0.65
}

gate3_apply_v17_task_contract() {
  gate3_apply_physical_arena_contract
  export PP_SERVES=26
  export PP_SERVES_LIST="$GATE3_PHYSICAL_SERVES_V17_TASK"
}

gate3_apply_hitter_fixed_home_rapid_1150ms_v1_contract() {
  local requested_serves="${PP_SERVES:-26}"
  if ! [[ "$requested_serves" =~ ^[1-9][0-9]*$ ]] || \
     (( requested_serves < 26 || requested_serves % 2 != 0 )); then
    echo "[gate3-common] fixed-HOME PP_SERVES must be an even integer >= 26"
    return 2
  fi
  gate3_apply_physical_arena_contract
  export PP_SERVES=26
  export PP_SERVES_LIST="$GATE3_PHYSICAL_SERVES_HITTER_FIXED_HOME"
  export PP_GATE3_LANE_CONTRACT=hitter_pingpong_fixed_home_rapid_1150ms_v1
  # Rapid questions release the incoming slot after 1.15 s. Previous balls
  # continue in independent return slots until ground; none is truncated.
  export PP_FLIGHT_S_LIST='[2.5,2.5,2.5,2.5,2.5,2.5,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,1.15,2.5]'
  export PP_PAUSE_S_LIST='[1.0,1.0,1.0,1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]'
  if [ "$requested_serves" -ne 26 ]; then
    # Extend the existing rapid sequence without an intermediate reset or tail.
    # Retain the six clean flights and give only the final flight a full tail.
    local expanded
    expanded="$(python3 - "$requested_serves" "$GATE3_PHYSICAL_SERVES_HITTER_FIXED_HOME" <<'PY'
import json, sys
n = int(sys.argv[1])
flat = json.loads(sys.argv[2])
rows = [flat[i:i+6] for i in range(0, len(flat), 6)]
ledger = rows[:6] + [rows[6 + i % 20] for i in range(n - 6)]
print(json.dumps([v for row in ledger for v in row]))
print(json.dumps([2.5] * 6 + [1.15] * (n - 7) + [2.5]))
print(json.dumps([1.0] * 6 + [0.0] * (n - 6)))
PY
)" || return 2
    local -a expanded_lines
    mapfile -t expanded_lines <<< "$expanded"
    export PP_SERVES="$requested_serves"
    export PP_SERVES_LIST="${expanded_lines[0]}"
    export PP_FLIGHT_S_LIST="${expanded_lines[1]}"
    export PP_PAUSE_S_LIST="${expanded_lines[2]}"
  fi
}

gate3_apply_hitter_low_corner_torture_v1_contract() {
  gate3_apply_physical_arena_contract
  # Default to a 102-flight soak; callers may raise this arbitrarily for a
  # practical run-until-fall session while retaining an exact report ledger.
  local cycles="${PP_TORTURE_CYCLES:-8}"
  if ! [[ "$cycles" =~ ^[1-9][0-9]*$ ]]; then
    echo "[gate3-common] PP_TORTURE_CYCLES must be a positive integer"
    return 2
  fi

  local serves="$GATE3_PHYSICAL_SERVES_HITTER_TORTURE_CLEAN_LOW_6"
  local cycle
  for ((cycle = 0; cycle < cycles; ++cycle)); do
    serves="${serves%\]}, ${GATE3_PHYSICAL_SERVES_HITTER_TORTURE_CYCLE_12#\[}"
  done
  export PP_SERVES=$((6 + 12 * cycles))
  export PP_SERVES_LIST="$serves"
  export PP_CLEAN_FIXED_HOME_SERVES=6
  export PP_GATE3_LANE_CONTRACT=hitter_pingpong_low_corner_torture_v1

  # Clean questions retain a complete outcome window.  All but the final
  # torture question use the shortest already-proven physical launch cadence;
  # the final one retains the complete recovery tail.
  local flight_values=""
  local pause_values=""
  local index value separator
  for ((index = 1; index <= PP_SERVES; ++index)); do
    separator=","
    if [ "$index" -eq 1 ]; then
      separator=""
    fi
    if [ "$index" -le "$PP_CLEAN_FIXED_HOME_SERVES" ] || \
       [ "$index" -eq "$PP_SERVES" ]; then
      value="2.5"
    else
      value="1.15"
    fi
    flight_values="${flight_values}${separator}${value}"
    if [ "$index" -le "$PP_CLEAN_FIXED_HOME_SERVES" ]; then
      value="1.0"
    else
      value="0.0"
    fi
    pause_values="${pause_values}${separator}${value}"
  done
  export PP_FLIGHT_S_LIST="[$flight_values]"
  export PP_PAUSE_S_LIST="[$pause_values]"
}
