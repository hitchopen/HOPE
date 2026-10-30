"""Replay identical inputs/clocks through native code and unchanged Python.

Compare every scalar observer output, errors, integer identities, last-good
receipts, validity and START player verdicts. No DDS or hardware is exercised.
"""
import ast
from collections import deque
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import random
import subprocess
import threading
from types import SimpleNamespace
from unittest.mock import patch

import pytest

pytestmark = pytest.mark.skipif(not os.environ.get("HOPE_NATIVE_BIN_DIR"), reason="requires native observer replay driver")
ROOT = Path(__file__).resolve().parents[1]
BASE = [2, 1, 1, 1700000000, 123456789, -.5, .2, .8, 1, 0, 0, 0, 1, 47, 123, 234]
RUNNER = [2, 42, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 123]
COMMAND = [2, 1, -1, .3, -.4, .8, 4, 1, 2, .3, 1700000000.8, 0, 1700000000, 123456789, 5, 6, 7, 14, .04]


def sanitize(value):
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else "Infinity" if value > 0 else "-Infinity"
    if isinstance(value, dict):
        return {k: sanitize(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [sanitize(v) for v in value]
    return value


def case(events=(), **kw):
    return dict(events=list(events), now=100_000_000_000, ros=1700000001000000000,
                runtime=dict(fresh=True, session_id="model21800_20261029T000000Z", attempt="planner_attempt_001",
                    planner_alive=True, x_hit=dict(request_id="1",success=True,message="x_hit=.58",x_hit_m=.58),
                    x_hit_text="request=1 success=1 x_hit=.58", error=""), **kw)


def event(kind, data, now=99_990_000_000):
    return dict(kind=kind, data=data, now=now)


def reference(request):
    from hope_observer import ObserverModule
    from hope_observer_core import decode_racket_packet, DecodeError, XHitStatus
    from hope_operations_core import OperationsState, CacheSnapshot
    from hope_player_positions import PlayerPositions
    shared = OperationsState()
    obj = object.__new__(ObserverModule)
    obj._shared_state, obj._input_lock = shared, threading.Lock()
    obj._command_packet, obj._command_error, obj._command_receipt_ns = None, "NO RACKET PACKET", 0
    obj._ball_receipts_ns = deque(maxlen=512)
    obj._base_stale_after_s, obj._ball_stale_after_s, obj._runner_stale_after_s = .5, .1, 1.
    for e in request["events"]:
        with patch("time.monotonic_ns", return_value=e["now"]):
            if e["kind"] == "ball":
                obj._ball_receipts_ns.append(e["now"])
            elif e["kind"] == "command":
                obj._command_receipt_ns = e["now"]
                try:
                    obj._command_packet = decode_racket_packet(e["data"])
                    obj._command_error = obj._command_packet.reason
                except DecodeError as exc:
                    obj._command_packet = None
                    obj._command_error = f"MALFORMED RACKET PACKET: {exc}"
            else:
                getattr(shared, e["kind"]).update(e["data"])
    runtime = request["runtime"]
    obj._runtime_snapshot = lambda: CacheSnapshot((runtime.get("session_id", ""),runtime.get("attempt", ""),
        runtime.get("planner_alive",False),XHitStatus(**runtime["x_hit"]) if runtime.get("x_hit") else None,
        runtime.get("x_hit_text","NO X_HIT STATUS")),runtime.get("fresh",False),runtime.get("error","RUNTIME SNAPSHOT PENDING"),0.,False)
    obj.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=request["ros"]))
    outputs = {}
    tree = ast.parse((ROOT / "a3/hope_observer.py").read_text())
    keys = next(n.value.keys for n in ast.walk(tree) if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Attribute)
                and n.targets[0].attr == "_topic_publishers" and isinstance(n.value, ast.Dict))
    obj._topic_publishers = {k.value:SimpleNamespace(publish=lambda m,k=k.value: outputs.__setitem__(k,m.data)) for k in keys}
    with patch("time.monotonic_ns",return_value=request["now"]):
        ObserverModule._publish_snapshot.__wrapped__(obj)
    result = {"view":outputs}
    for name in ("base", "runner"):
        s=getattr(shared,name).snapshot()
        result[name]=dict(value=asdict(s.value) if s.value else None, received_ns=s.received_ns,error=s.error,decode_error=s.decode_error)
    result["command"]=dict(value=asdict(obj._command_packet) if obj._command_packet else None,
        received_ns=obj._command_receipt_ns,error=obj._command_error)
    players = PlayerPositions()
    for p in request.get("players",[]):
        players.update(p["name"],p["p"],p["q"],p["stamp"],p["frame"],p["now"])
    result["players"]=players.status(request["now"]*1e-9)
    return result


def compare(actual, expected, path=""):
    if isinstance(expected, dict):
        for k, v in expected.items():
            compare(actual[k],v,path+"."+k)
    elif isinstance(expected, (list,tuple)):
        assert len(actual)==len(expected),path
        for i,(a,b) in enumerate(zip(actual,expected)):compare(a,b,path+f"[{i}]")
    elif isinstance(expected,float):
        if math.isnan(expected):assert actual is None,path
        else:assert actual == pytest.approx(expected,rel=1e-14,abs=1e-12),(path,actual,expected)
    elif isinstance(expected,int) and isinstance(actual,str):
        assert int(actual)==expected,path  # timestamps beyond int64
    else:
        assert actual==expected,(path,actual,expected)


def replay(cases):
    binary=Path(os.environ["HOPE_NATIVE_BIN_DIR"])/"hope-operations-observer-driver"
    result=subprocess.run([str(binary)],input="".join(json.dumps(sanitize(c))+"\n" for c in cases),
                          text=True,capture_output=True,check=True)
    lines=result.stdout.splitlines()
    assert len(lines)==len(cases)
    for i,(raw,c) in enumerate(zip(lines,cases)):
        compare(json.loads(raw), reference(c),str(i))


def test_wire_decoders_and_every_observer_output_match_python():
    cases=[case()]
    for kind, baseline in (("base",BASE),("runner",RUNNER),("command",COMMAND)):
        prior=event(kind,baseline,99_000_000_000)
        for length in (0,len(baseline)-1,len(baseline),len(baseline)+1):
            cases.append(case([prior,event(kind,(baseline+[0])[:length])]))
        for i in range(len(baseline)):
            for value in (-1.,0.,.5,1.,2.,99.,float(2**52),float(2**52+1),math.nan,math.inf,-math.inf):
                data=baseline.copy();data[i]=value
                cases.append(case([prior,event(kind,data)]))
    for schema,extra in ((2,[]),(4,[1]),(5,[1,.58,1])):
        data=COMMAND.copy();data[0]=schema;data+=extra
        for valid in (0,1):
            data=data.copy();data[1]=valid
            cases.append(case([event("command",data)]))
    from hope_runner_control_core import (MODE_NAMES, ROLE_NAMES, RESULT_NAMES, REASON_NAMES, SERVE_STATE_NAMES, GRIPPER_STATE_NAMES,ACTION_NAMES)
    for index,names in ((3,MODE_NAMES),(7,ROLE_NAMES),(10,RESULT_NAMES),(11,REASON_NAMES),(13,SERVE_STATE_NAMES),(14,GRIPPER_STATE_NAMES),(17,ACTION_NAMES),(18,RESULT_NAMES),(19,REASON_NAMES)):
        for code in names:
            data=RUNNER.copy();data[index]=code
            cases.append(case([event("runner",data)]))
    replay(cases)


def test_staleness_clock_jump_identity_and_rate_match_python():
    from hope_runner_control_core import runner_session_fingerprint
    rng=random.Random(1291);cases=[]
    for rate in (150,300,600):
        for age_ms in (0,99.999999,100,100.000001,250,500,500.000001,1000,1000.000001,3000):
            end=100_000_000_000-round(age_ms*1e6)
            events=[dict(kind="ball",now=end-int((599-i)*1e9/rate)) for i in range(600)]
            runner=RUNNER.copy();runner[20]=runner_session_fingerprint("model21800_20261029T000000Z")
            events += [event("base",BASE,end),event("runner",runner,end),event("command",COMMAND,end)]
            c=case(events);c["ros"]+=rng.randint(-10000000000,10000000000);cases.append(c)
    for fresh,error in ((False,"snapshot refresh pending"),(False,"snapshot refresh overdue"),(False,"OSError: failed"),(True,"")):
        c=case([event("runner",RUNNER)]);c["runtime"]["fresh"]=fresh;c["runtime"]["error"]=error;cases.append(c)
    replay(cases)


def test_oversized_timestamp_age_matches_without_platform_long_double_precision():
    # Keep exact text/integer comparisons: do not hide the Apple ARM64
    # long-double rounding difference by loosening the production parity test.
    cases = []
    for seconds in (10_000_000_000, 2**51, 2**52):
        for ros in (0, 1700000001000000000, 2**63-1):
            data = BASE.copy()
            data[3] = float(seconds)
            c = case([event("base", data)])
            c["ros"] = ros
            cases.append(c)
    replay(cases)


def test_player_eligibility_replays_invalidations_and_boundaries_match():
    cases=[]
    def p(x,stamp,now=99.99,**kw):
        return dict(name="UCB_P2",p=[x,0.,1.],q=[0.,0.,0.,1.],stamp=stamp,frame="hope_table_global",now=now,**kw)
    for x in (-1.,0.,2.74,2.740001,math.nan,math.inf):
        for now in (99.,99.75,99.749999,100.,100.01):
            cases.append(case(players=[p(x,1,now)]))
    cases += [case(players=[p(-1,2,99.),p(-1,2)]),case(players=[p(-1,2,99.),p(3,1)]),
              case(players=[p(-1,2),p(math.nan,3)]),case(players=[p(-1,2),p(math.nan,3),p(3,1)])]
    replay(cases)
