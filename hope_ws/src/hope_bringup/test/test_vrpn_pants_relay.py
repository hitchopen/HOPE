"""Real ROS callbacks, capture-only publishers; no hardware action clients."""
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import pytest

rclpy=pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from vrpn_pants_contract import Selection,load_profiles,observation

def load(name):
    loader=importlib.machinery.SourceFileLoader(name,str(SCRIPTS/name))
    spec=importlib.util.spec_from_loader(name,loader)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    loader.exec_module(module)
    return module


@pytest.fixture
def ros():
    rclpy.init(args=['--ros-args','-p','localization_source:=vrpn_ground_truth',
        '-p','world_frame_calibrated:=true','-p','world_frame_sha256:='+'a'*64,
        '-p','position_scale:=0.001',
        '-p','follow_lifecycle_config:=true','-p','server:=192.168.100.99',
        '-p','ball_tracking_mode:=rigid_body','-p','ball_object:=Ball'])
    yield
    rclpy.shutdown()


def pose(node,x=-460):
    msg=PoseStamped()
    msg.header.frame_id='world'
    msg.header.stamp=rclpy.clock.Clock().now().to_msg()
    msg.pose.position.x=float(x)
    msg.pose.position.y=-1100.0
    msg.pose.position.z=200.0
    msg.pose.orientation.w=1.0
    return msg


def test_switch_flows_from_snapshot_through_vrpn_into_guarded_base(ros):
    relay=load('avatar_pro_vrpn_relay').AvatarProVrpnRelay()
    base=load('hope_base_pose_flat_relay').BasePoseFlatRelay()
    outputs=[]; pelvis=[]; observations=[]; balls=[]; raw_players=[]
    for pub in relay._player_position_publishers.values():
        pub.publish=raw_players.append
    base._pub.publish=outputs.append
    base._pelvis_pub.publish=pelvis.append
    relay._observation_pub.publish=lambda msg:(observations.append(msg),base._vrpn_pose_cb(msg))
    relay._ball_pub.publish=balls.append
    relay._on_pose('/vrpn_mocap/UCB_P1/pose','UCB_P1',pose(relay))
    assert not observations  # config required
    for revision,profile,side in [(1,'v2','P1'),(2,'v4','P2')]:
        relay._config_cb(String(data=json.dumps(dict(revision=revision,pants_profile=profile,table_side=side,state='STOPPED',busy=False,
            mocap_source='vrpn',vrpn_ip='192.168.100.99',vrpn_port='3883',vrpn_units='mm'))))
        relay._on_pose('/vrpn_mocap/UCB_'+side+'/pose','UCB_'+side,pose(relay))
        assert raw_players[-1].pose.position.x == pytest.approx(-.46)
        assert raw_players[-1].header.frame_id == 'hope_table_global'
        assert outputs[-1].data[1]==1
        assert base._vrpn_selection==Selection(profile,side,revision)
        assert outputs[-1].data[5:8]==pytest.approx([-.46,-1.1,.96])
        assert pelvis[-1].pose.position.z==pytest.approx(.2)  # policy offset only on flat ABI
        ball_pose=pose(relay,2000)
        ball_pose.pose.orientation.w=0.0  # a point-only ball has no useful orientation
        relay._on_pose('/vrpn_mocap/Ball/pose','ball',ball_pose)
        assert balls[-1].point.x==pytest.approx(2)
    base._vrpn_pose_cb(observations[0])
    assert outputs[-1].data[1]==0  # prior configuration cannot roll back
    relay._config_seen=time.monotonic()-4
    count=len(observations)
    relay._on_pose('unused','UCB_P2',pose(relay))
    assert len(observations)==count
    relay.destroy_node();base.destroy_node()


def test_bad_timestamp_and_jump_do_not_produce_valid_base(ros):
    base=load('hope_base_pose_flat_relay').BasePoseFlatRelay()
    output=[];base._pub.publish=output.append
    stamp=base.get_clock().now().nanoseconds
    def send(ns,p=(0.,0.,0.)):
        doc=observation(Selection('v4','P1',2),load_profiles(),divmod(ns,10**9),p,(0.,0.,0.,1.))
        base._vrpn_pose_cb(String(data=json.dumps(doc)))
    send(stamp-10**9);assert not output  # stale input is dropped, never restamped
    send(stamp);assert output[-1].data[1]==1
    accepted_count = len(output)
    accepted_stamp = base._last_source_ns
    send(stamp)
    send(stamp+1_000_000,(1.,0.,0.))
    assert len(output) == accepted_count
    assert base._last_source_ns == accepted_stamp
    assert base._reordered_dropped == 1 and base._jump_dropped == 1
    base.destroy_node()


def test_burst_pose_noise_stays_valid_and_large_jump_is_dropped(ros):
    import math
    base = load('hope_base_pose_flat_relay').BasePoseFlatRelay()
    output = []
    base._pub.publish = output.append
    base._pelvis_pub.publish = lambda msg: None
    stamp = base.get_clock().now().nanoseconds
    def send(delta_ns, x, degrees):
        half_angle = math.radians(degrees) / 2
        doc = observation(Selection('v4', 'P1', 2), load_profiles(),
                          divmod(stamp + delta_ns, 10**9), (x, 0., 0.),
                          (0., 0., math.sin(half_angle), math.cos(half_angle)))
        base._vrpn_pose_cb(String(data=json.dumps(doc)))
        return output[-1].data[1]
    try:
        assert send(0, 0., 0.) == 1
        assert send(25_000, .0000357, .0137) == 1
        assert send(50_000, .009, 2.9) == 1
        accepted_count = len(output)
        accepted_stamp = base._last_source_ns
        send(75_000, .2, 2.9)
        assert len(output) == accepted_count
        assert base._last_source_ns == accepted_stamp
        assert base._jump_dropped == 1
        assert send(3_333_333, .001, .2) == 1
    finally:
        base.destroy_node()


@pytest.mark.parametrize('hz', [150, 300])
def test_player_start_evidence_is_30hz_while_base_observations_remain_full_rate(ros, hz, monkeypatch):
    relay = load('avatar_pro_vrpn_relay').AvatarProVrpnRelay()
    raw, observations = [], []
    relay._player_position_publishers['UCB_P1'].publish = raw.append
    relay._observation_pub.publish = observations.append
    relay._config_cb(String(data=json.dumps(dict(revision=1, pants_profile='v2', table_side='P1',
        state='STOPPED', busy=False, mocap_source='vrpn', vrpn_ip='192.168.100.99',
        vrpn_port='3883', vrpn_units='mm'))))
    try:
        for i in range(hz):
            msg = pose(relay)
            ns = 100_000_000_000 + round(i*1e9/hz)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
            monkeypatch.setattr(relay, "get_clock", lambda ns=ns: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=ns)))
            relay._on_pose('/vrpn_mocap/UCB_P1/pose', 'UCB_P1', msg)
        assert len(raw) == 30
        assert len(observations) == hz
    finally:
        relay.destroy_node()


def test_rigid_ball_orientation_reaches_pose_array_and_invalid_spin_stays_invalid(ros):
    relay=load('avatar_pro_vrpn_relay').AvatarProVrpnRelay()
    arrays=[];relay._pose_array_pub.publish=arrays.append
    for revision,side in [(1,'P1'),(2,'P2')]:
        relay._config_cb(String(data=json.dumps(dict(revision=revision,pants_profile='v2',table_side=side,
            state='STOPPED',busy=False,mocap_source='vrpn',vrpn_ip='192.168.100.99',vrpn_port='3883',vrpn_units='mm'))))
        relay._on_pose("body", "UCB_" + side, pose(relay))
        msg=pose(relay,2000)
        # 90 degrees about X, unchanged by the pants identifier.
        msg.pose.orientation.x=2**-.5;msg.pose.orientation.w=2**-.5
        relay._on_pose('/vrpn_mocap/Ball/pose_id_4','ball',msg)
        q=arrays[-1].poses[0].orientation
        expected=[2**-.5,0,0,2**-.5]
        assert [q.x,q.y,q.z,q.w]==pytest.approx(expected)
        for bad in [0.,float('nan')]:
            msg.pose.orientation.x=0.;msg.pose.orientation.w=bad
            relay._on_pose('/vrpn_mocap/Ball/pose_id_4','ball',msg)
            q=arrays[-1].poses[0].orientation
            assert [q.x,q.y,q.z,q.w]==[0,0,0,0]
            assert arrays[-1].poses[0].position.x==pytest.approx(2)
    relay.destroy_node()


@pytest.mark.parametrize("side", ["P1", "P2"])
def test_planner_full_stream_preserves_all_300hz_ball_exposures(ros, side, monkeypatch):
    relay = load('avatar_pro_vrpn_relay').AvatarProVrpnRelay()
    full = []
    relay._planner_pose_array_pub.publish = full.append
    relay._config_cb(String(data=json.dumps(dict(
        revision=1, pants_profile='v2', table_side=side, state='STOPPED', busy=False,
        mocap_source='vrpn', vrpn_ip='192.168.100.99', vrpn_port='3883', vrpn_units='mm'))))
    for i in range(300):
        msg = pose(relay, 2000)
        ns = 100_000_000_000 + round(i * 1e9 / 300)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
        monkeypatch.setattr(relay, 'get_clock', lambda ns=ns: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=ns)))
        body = PoseStamped()
        body.header.frame_id = "world"
        body.header.stamp = msg.header.stamp
        body.pose.position.x = -460.
        body.pose.orientation.w = 1.
        relay._on_pose('body', 'UCB_' + side, body)
        relay._on_pose('/vrpn_mocap/Ball/pose', 'ball', msg)
    assert len(full) == 300
    for i, msg in enumerate(full):
        assert msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec == (
            100_000_000_000 + round(i * 1e9 / 300))
        assert len(msg.poses) == 1
        assert msg.poses[0].position.x == pytest.approx(2.0)
    relay.destroy_node()

@pytest.mark.parametrize('hz',[150,300])
@pytest.mark.parametrize('side',['P1','P2'])
def test_pelvis_flat_publishes_in_same_callback_without_any_ball(ros,hz,side):
    relay=load('avatar_pro_vrpn_relay').AvatarProVrpnRelay()
    base=load('hope_base_pose_flat_relay').BasePoseFlatRelay()
    output=[];base._pub.publish=output.append
    relay._observation_pub.publish=base._vrpn_pose_cb
    relay._config_cb(String(data=json.dumps(dict(revision=1,pants_profile='v4',table_side=side,
        state='STOPPED',busy=False,mocap_source='vrpn',vrpn_ip='192.168.100.99',
        vrpn_port='3883',vrpn_units='mm'))))
    try:
        for i in range(30):
            source=pose(relay)
            relay._on_pose('/vrpn_mocap/UCB_'+side+'/pose','UCB_'+side,source)
            assert len(output)==i+1  # Synchronous end-to-end into flat base wire.
            assert output[-1].data[1]==1
            time.sleep(1/hz)
    finally:
        relay.destroy_node();base.destroy_node()


@pytest.mark.parametrize('world_confirmed', [False, True])
def test_catalog_asset_requires_only_world_confirmation_not_pelvis_checkbox(ros, world_confirmed):
    base = load('hope_base_pose_flat_relay').BasePoseFlatRelay()
    output = []
    base._pub.publish = output.append
    base._pelvis_pub.publish = lambda msg: None
    base._world_calibrated = world_confirmed
    try:
        assert not base.has_parameter('vrpn_pelvis_asset_confirmed')
        stamp = base.get_clock().now().nanoseconds
        doc = observation(Selection('v4', 'P2', 2), load_profiles(),
                          divmod(stamp, 10**9), (3.2, -.7, .1), (0., 0., 0., 1.))
        base._vrpn_pose_cb(String(data=json.dumps(doc)))
        assert output[-1].data[1] == int(world_confirmed)
        if world_confirmed:
            assert output[-1].data[5:8] == pytest.approx([3.2, -.7, .86])
        doc['asset_sha256'] = '0' * 64
        base._vrpn_pose_cb(String(data=json.dumps(doc)))
        assert output[-1].data[1] == 0  # catalog integrity remains mandatory
    finally:
        base.destroy_node()


@pytest.mark.parametrize('body_side', ['P1', 'P2'])
@pytest.mark.parametrize('physical_side', ['P1', 'P2'])
def test_either_pants_at_either_end_has_same_policy_pose_and_ball(ros, body_side, physical_side):
    import math
    relay = load('avatar_pro_vrpn_relay').AvatarProVrpnRelay()
    base = load('hope_base_pose_flat_relay').BasePoseFlatRelay()
    output, balls, raw = [], [], []
    base._pub.publish = output.append
    relay._observation_pub.publish = base._vrpn_pose_cb
    relay._planner_pose_array_pub.publish = balls.append
    relay._player_position_publishers['UCB_' + body_side].publish = raw.append
    relay._config_cb(String(data=json.dumps(dict(revision=1, pants_profile='v4', table_side=body_side,
        state='STOPPED', busy=False, mocap_source='vrpn', vrpn_ip='192.168.100.99',
        vrpn_port='3883', vrpn_units='mm'))))
    try:
        body = pose(relay, -691 if physical_side == 'P1' else 3431)
        body.pose.position.y = -859.9 if physical_side == 'P1' else -665.1
        yaw = math.radians(-10 if physical_side == 'P1' else 170)
        body.pose.orientation.z, body.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        relay._on_pose('body', 'UCB_' + body_side, body)
        assert output[-1].data[1] == 1
        assert output[-1].data[5:8] == pytest.approx([-.691, -.8599, .96])
        w, x, y, z = output[-1].data[8:12]
        assert math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))) == pytest.approx(-10)
        assert raw[-1].pose.position.x == pytest.approx(body.pose.position.x * .001)
        # The fixed policy plane now points towards the table at either end.
        plane_x = output[-1].data[5] + .58
        assert plane_x == pytest.approx(-.111)
        ball = pose(relay, 1000 if physical_side == 'P1' else 1740)
        ball.pose.position.y = -500 if physical_side == 'P1' else -1025
        ball.pose.orientation.w = 0.  # unknown spin must remain unknown
        relay._on_pose('ball', 'ball', ball)
        p, q = balls[-1].poses[0].position, balls[-1].poses[0].orientation
        assert [p.x,p.y,p.z] == pytest.approx([1.,-.5,.2])
        assert [q.x,q.y,q.z,q.w] == [0.,0.,0.,0.]
        assert balls[-1].header.stamp == ball.header.stamp
        assert base._vrpn_physical_side == physical_side
    finally:
        relay.destroy_node(); base.destroy_node()


def test_no_frame_from_ball_other_body_or_stale_pose_and_no_live_flip(ros):
    relay = load('avatar_pro_vrpn_relay').AvatarProVrpnRelay()
    observations, balls, raw = [], [], []
    relay._observation_pub.publish = observations.append
    relay._ball_pub.publish = balls.append
    relay._player_position_publishers['UCB_P2'].publish = raw.append
    relay._config_cb(String(data=json.dumps(dict(revision=1, pants_profile='v4', table_side='P1',
        state='STOPPED', busy=False, mocap_source='vrpn', vrpn_ip='192.168.100.99',
        vrpn_port='3883', vrpn_units='mm'))))
    try:
        relay._on_pose('ball', 'ball', pose(relay, 3000))
        relay._on_pose('other', 'UCB_P2', pose(relay, 3400))
        stale = pose(relay, 3400); stale.header.stamp.sec -= 2
        relay._on_pose('selected', 'UCB_P1', stale)
        assert raw and not observations and not balls and relay._table_frame.side is None
        relay._on_pose('selected', 'UCB_P1', pose(relay, -500))
        assert relay._table_frame.side == 'P1' and len(observations) == 1
        relay._on_pose('selected', 'UCB_P1', pose(relay, 3400))
        relay._on_pose('ball', 'ball', pose(relay))
        relay._on_pose('selected', 'UCB_P1', pose(relay, -500))
        assert relay._table_frame.side == 'P1' and relay._table_frame.fault
        assert len(observations) == 1 and not balls
    finally:
        relay.destroy_node()


def test_base_rejects_same_revision_physical_frame_change(ros):
    base = load('hope_base_pose_flat_relay').BasePoseFlatRelay()
    output = []; base._pub.publish = output.append
    try:
        selection = Selection('v4', 'P1', 1)
        stamp = base.get_clock().now().nanoseconds
        for side in ['P1', 'P2']:
            doc = observation(selection, load_profiles(), divmod(stamp, 10**9),
                              (-.5, -.7, .2), (0.,0.,0.,1.), side)
            base._vrpn_pose_cb(String(data=json.dumps(doc)))
        assert output[0].data[1] == 1 and output[-1].data[1] == 0
    finally:
        base.destroy_node()
