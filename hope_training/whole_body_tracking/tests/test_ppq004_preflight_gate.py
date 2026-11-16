import sys,os,json,shutil
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from validate_ppq004_preflight import validate

@pytest.fixture
def session(tmp_path):
    location=os.environ.get('HOPE_GRU_DIAGNOSTIC_SESSION')
    if not location:pytest.skip('set HOPE_GRU_DIAGNOSTIC_SESSION to a local archived preflight session')
    original=Path(location)
    if not (original/'attempt-8aUCdxAm/preflight/summary.json').exists():pytest.skip('requires archived actual planner preflight')
    for name in ['snapshot.json','bundle','fixed-home-100.yaml']:(tmp_path/name).symlink_to(original/name)
    names=['preflight/summary.json','preflight/configuration.json','preflight/per-ball.json','preflight/physical-evidence.json','preflight/planner-commands.jsonl','preflight/policy-trace.jsonl','preflight-ros/planner-audit.csv','preflight-ros/sensor-and-planner.jsonl','preflight-ros/planner.log','launcher.log']
    for name in names:
        dest=tmp_path/'attempt-8aUCdxAm'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(original/'attempt-8aUCdxAm'/name,dest)
    return tmp_path


def test_real_preflight_infrastructure_is_independent_of_hitting_or_pass_flag(session):
    path=session/'attempt-8aUCdxAm/preflight/summary.json';r=json.loads(path.read_text());r['integration_preflight_pass']=False;path.write_text(json.dumps(r))
    r=validate(session)
    assert r['infrastructure_gate']=='PASS'
    assert r['hitting_observed_not_a_gate']['raw_contacts']==5
    assert r['hitting_observed_not_a_gate']['qualified_contacts']==0
    assert r['hitting_observed_not_a_gate']['legal_landings']==0
    assert r['scheduled_diagnostic_balls']==100
    assert len(r['actor_deliveries'])==5


@pytest.mark.parametrize('problem',['soft','actor','undrained','planner_error'])
def test_fail_closed_on_infrastructure_error(session,problem):
    base=session/'attempt-8aUCdxAm'
    if problem=='soft':
        path=base/'preflight/configuration.json';r=json.loads(path.read_text());r['table_impact_model']='soft';path.write_text(json.dumps(r))
    elif problem=='actor':
        path=base/'preflight/policy-trace.jsonl';rows=[json.loads(x) for x in path.read_text().splitlines()]
        for row in rows:row['target_source']='idle_no_plan'
        path.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    elif problem=='undrained':
        path=base/'preflight/summary.json';r=json.loads(path.read_text());r['remaining_live_ids']=[9];path.write_text(json.dumps(r))
    else:
        path=base/'preflight-ros/planner.log';path.write_text(path.read_text()+'\n[ERROR] transport failure\n')
    with pytest.raises(ValueError):validate(session)
