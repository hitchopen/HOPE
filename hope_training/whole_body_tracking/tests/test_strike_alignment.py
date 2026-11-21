import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from reference_gru_diagnostic import planner_target
from diagnose_strike_alignment import bracket
from planner_rally_protocol import decode_command


def test_frozen_training_countdown_is_not_capped_or_shifted():
    target=planner_target([.58,-.26,1.051],[1.73,.178,1.193],21.115060091,20.44)
    assert target.time_to_strike==pytest.approx(.675060091)
    assert planner_target(target.pos_w,target.vel_w,21.115060091,21.12).time_to_strike==pytest.approx(-.004939909)
    np.testing.assert_array_equal(target.pos_w,[.58,-.26,1.051])


def test_absolute_clock_and_height_offset_applied_once():
    unix=1793955061.
    wire=[2,1,1,.08,-1.0225,1.05129664686,1.73,.178,1.193,.6945913,unix+1.178896,0,unix,484304700,1,1,1,37,.18]
    cmd,reason=decode_command(wire,int((unix-1)*1e9),int((unix+.489)*1e9))
    assert reason=='accepted'
    np.testing.assert_allclose(cmd['position_w'],[.58,-.26,1.05129664686])
    anchor=cmd['deadline_unix_s']-21.115060091
    target=planner_target(cmd['position_w'],cmd['velocity_w'],cmd['deadline_unix_s']-anchor,20.44)
    assert target.time_to_strike==pytest.approx(.675060091,abs=3e-7)


def test_deadline_interpolation_refuses_unobserved_time():
    rows=[{'sim_t':1.,'qpos':[1.,2.]},{'sim_t':1.001,'qpos':[2.,4.]}]
    value,interval=bracket(rows,1.00025,'qpos')
    np.testing.assert_allclose(value,[1.25,2.5]);assert interval==[1.,1.001]
    with pytest.raises(ValueError):bracket(rows,2.,'qpos')
