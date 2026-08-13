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
