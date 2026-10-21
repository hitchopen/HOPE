"""Real ROS callbacks, capture-only publishers; no hardware action clients."""
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys
import time
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
        '-p','vrpn_pelvis_asset_confirmed:=true','-p','position_scale:=0.001',
        '-p','follow_lifecycle_config:=true','-p','server:=192.168.100.99',
        '-p','ball_tracking_mode:=rigid_body','-p','ball_object:=Ball'])
    yield
    rclpy.shutdown()


def pose(node,x=2100):
    msg=PoseStamped()
    msg.header.frame_id='world'
    msg.header.stamp=node.get_clock().now().to_msg()
    msg.pose.position.x=float(x)
    msg.pose.position.y=-1100.0
    msg.pose.position.z=200.0
    msg.pose.orientation.w=1.0
    return msg


def test_switch_flows_from_snapshot_through_vrpn_into_guarded_base(ros):
    relay=load('avatar_pro_vrpn_relay').AvatarProVrpnRelay()
    base=load('hope_base_pose_flat_relay').BasePoseFlatRelay()
    outputs=[]; pelvis=[]; observations=[]; balls=[]
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
        assert outputs[-1].data[1]==1
        assert base._vrpn_selection==Selection(profile,side,revision)
        assert outputs[-1].data[5:8]==pytest.approx([2.1,-1.1,.96] if side=='P1' else [.64,-.425,.96])
        assert pelvis[-1].pose.position.z==pytest.approx(.2)  # policy offset only on flat ABI
        ball_pose=pose(relay,2000)
        ball_pose.pose.orientation.w=0.0  # a point-only ball has no useful orientation
        relay._on_pose('/vrpn_mocap/Ball/pose','ball',ball_pose)
        assert balls[-1].point.x==pytest.approx(2 if side=='P1' else .74)
    base._vrpn_pose_cb(observations[0])
    assert outputs[-1].data[1]==0  # prior configuration cannot roll back
    relay._config_seen=time.monotonic()-4
    count=len(observations)
    relay._on_pose('unused','UCB_P2',pose(relay))
    assert len(observations)==count
    relay.destroy_node();base.destroy_node()


def test_unconfirmed_import_bad_timestamp_and_jump_do_not_produce_valid_base(ros):
    base=load('hope_base_pose_flat_relay').BasePoseFlatRelay()
    output=[];base._pub.publish=output.append
    stamp=base.get_clock().now().nanoseconds
    def send(ns,p=(0.,0.,0.)):
        doc=observation(Selection('v4','P1',2),load_profiles(),divmod(ns,10**9),p,(0.,0.,0.,1.))
        base._vrpn_pose_cb(String(data=json.dumps(doc)))
    base._vrpn_confirmed=False
    send(stamp);assert output[-1].data[1]==0
    base._vrpn_confirmed=True
    send(stamp-10**9);assert output[-1].data[1]==0
    send(stamp);assert output[-1].data[1]==1
    send(stamp);assert output[-1].data[1]==0
    send(stamp+1_000_000,(1.,0.,0.));assert output[-1].data[1]==0
    base.destroy_node()


def test_rigid_ball_orientation_reaches_pose_array_and_invalid_spin_stays_invalid(ros):
    relay=load('avatar_pro_vrpn_relay').AvatarProVrpnRelay()
    arrays=[];relay._pose_array_pub.publish=arrays.append
    for revision,side in [(1,'P1'),(2,'P2')]:
        relay._config_cb(String(data=json.dumps(dict(revision=revision,pants_profile='v2',table_side=side,
            state='STOPPED',busy=False,mocap_source='vrpn',vrpn_ip='192.168.100.99',vrpn_port='3883',vrpn_units='mm'))))
        msg=pose(relay,2000)
        # 90 degrees about X; P2 also rotates the world by pi about Z.
        msg.pose.orientation.x=2**-.5;msg.pose.orientation.w=2**-.5
        relay._on_pose('/vrpn_mocap/Ball/pose_id_4','ball',msg)
        q=arrays[-1].poses[0].orientation
        expected=[2**-.5,0,0,2**-.5] if side=='P1' else [0,2**-.5,2**-.5,0]
        assert [q.x,q.y,q.z,q.w]==pytest.approx(expected)
        for bad in [0.,float('nan')]:
            msg.pose.orientation.x=0.;msg.pose.orientation.w=bad
            relay._on_pose('/vrpn_mocap/Ball/pose_id_4','ball',msg)
            q=arrays[-1].poses[0].orientation
            assert [q.x,q.y,q.z,q.w]==[0,0,0,0]
            assert arrays[-1].poses[0].position.x==pytest.approx(2 if side=='P1' else .74)
    relay.destroy_node()


@pytest.mark.parametrize("side", ["P1", "P2"])
def test_planner_full_stream_preserves_all_300hz_ball_exposures(ros, side):
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
        relay._on_pose('/vrpn_mocap/Ball/pose', 'ball', msg)
        # A body callback must never re-publish the cached ball.
        relay._on_pose('/vrpn_mocap/UCB_P1/pose', 'UCB_P1', msg)
    assert len(full) == 300
    for i, msg in enumerate(full):
        assert msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec == (
            100_000_000_000 + round(i * 1e9 / 300))
        assert len(msg.poses) == 1
        assert msg.poses[0].position.x == pytest.approx(2.0 if side == 'P1' else .74)
    relay.destroy_node()
