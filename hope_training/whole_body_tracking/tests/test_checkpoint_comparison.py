import sys,json
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from render_checkpoint_comparison import overlay,clock
from batch_recurrent_comparison import classify,valid_done,sha

def summary():return {'error':None,'rally_failure':None,'groups':{'all':{'launched':100,'unlaunched':0,'unresolved':0}}}
def test_policy_fall_and_infrastructure_are_distinct():
 s=summary();assert classify(s)=='COMPLETE_100'
 s['rally_failure']='fall';assert classify(s)=='EARLY_TERMINATION_FALL'
 s['error']='ROS transport';assert 'INFRASTRUCTURE' in classify(s)

def test_overlay_does_not_reveal_future_plan_or_contacts():
 row={'launched':True,'launched_at_s':20.,'shot_id':1,'planner_side':'backhand','commands':[{'applied':True,'sim_t':20.425}],
  'events':[{'kind':'racket_contact','episode_t':21.1}],'qualified_contact':False,'legal_landing':False}
 s=summary()
 assert 'Serve 0/100' in overlay(19.,s,[row],1000)[1]
 assert 'PENDING' in overlay(20.1,s,[row],1000)[1]
 assert 'BACKHAND' in overlay(20.5,s,[row],1000)[1]
 assert 'Raw contact balls 0' in overlay(20.5,s,[row],1000)[2]
 assert 'Raw contact balls 1' in overlay(21.2,s,[row],1000)[2]
 assert 'legal returns 0' in overlay(21.2,s,[row],1000)[2]
 assert clock(20)=='0:00:20.00'

def test_resume_never_accepts_changed_video(tmp_path):
 movie=tmp_path/'comparison.mp4';movie.write_bytes(b'verified')
 (tmp_path/'completed.json').write_text(json.dumps({'protocol_signature':'x','sha256':{str(movie):sha(movie)}}))
 assert valid_done(tmp_path,'x')
 movie.write_bytes(b'partial');assert not valid_done(tmp_path,'x')
 assert not valid_done(tmp_path,'other protocol')
