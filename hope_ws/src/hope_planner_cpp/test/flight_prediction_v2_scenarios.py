#!/usr/bin/env python3
"""Raw-observation forecast scenarios; synthetic registration is not live qualification."""
import argparse
import json
import math
import subprocess
import time

import numpy as np
from scipy.integrate import solve_ivp
import rclpy
from rclpy.qos import qos_profile_sensor_data
from hope_msgs.msg import BallObservationV2, FlightContextV2, FlightPredictionV2, FlightPacketV2
from std_msgs.msg import Float64MultiArray

KD, KM, REST, GRIP, MU, RADIUS = .12861635416914574, .00379260504480258, .9313775047019153, .38744443855065164, .16706204822993975, .02


def independent_trace(direction, duration=1.2):
    """Piecewise DOP853 fixture, independent of the C++ algorithm under test."""
    state = np.array([.15 if direction > 0 else 2.59, -.2, .30, direction*4.5, 0., -.5])
    omega = np.array([0., direction*30., 0.])
    pieces, contacts = [], []
    start = 0.
    while start < duration:
        held = omega.copy()
        def rhs(_, value):
            velocity = value[3:]
            return np.r_[velocity, [0., 0., -9.81] - KD*np.linalg.norm(velocity)*velocity + KM*np.cross(held, velocity)]
        def contact(_, value):
            return value[2]-RADIUS
        contact.terminal, contact.direction = True, -1
        solution = solve_ivp(rhs, (start, duration), state, method='DOP853',
                             rtol=2e-12, atol=2e-14, max_step=.001,
                             dense_output=True, events=contact)
        stop = solution.t[-1]
        pieces.append((start, stop, solution.sol, held))
        state = solution.y[:, -1].copy()
        if not len(solution.t_events[0]) or not 0 <= state[0] <= 2.74:
            break
        slip = np.array([state[3]-RADIUS*omega[1], state[4]+RADIUS*omega[0], 0.])
        norm = np.linalg.norm(slip)
        dv = -min(GRIP*norm, MU*(1+REST)*abs(state[5]))*slip/norm if norm else np.zeros(3)
        state[3:5] += dv[:2]
        state[5] *= -REST
        omega += np.array([dv[1], -dv[0], 0.])/((2/3)*RADIUS)
        contacts.append(stop)
        start = stop
    def at(t):
        for lower, upper, dense, spin in reversed(pieces):
            if lower <= t <= upper + 1e-8:
                return dense(min(t, upper)), spin
        raise ValueError(('fixture outside trajectory', t))
    # The intended legal serve has B1 on the striker half, B2 on the
    # receiver half, and exits the table before a third table contact.
    assert len(contacts) == 2, contacts
    assert direction*(at(contacts[0])[0][0]-1.37) < 0
    assert direction*(at(contacts[1])[0][0]-1.37) > 0
    return at, contacts


def scenario(executable, direction, rate, policy):
    session = f'scoped_{direction}_{rate}_{policy}'
    process = subprocess.Popen([executable, '--ros-args', '-p', 'session_id:='+session,
        '-p', 'registration_id:=synthetic_registered_table', '-p', 'plane_policy:='+policy,
        '-p', 'full_output_rate_hz:=150.0', '-p', 'test.worker_delay_ms:=12',
        '-p', 'test.worker_delay_every:=40'], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    node = rclpy.create_node('scenario_'+str(abs(hash(session))))
    predictions, packets = [], []
    node.create_subscription(FlightPredictionV2, '/hope/shadow/flight_v2/prediction',
        lambda m: predictions.append((m, time.time_ns())) if m.session_id == session else None, qos_profile_sensor_data)
    node.create_subscription(FlightPacketV2, '/hope/shadow/flight_v2/packet', packets.append, qos_profile_sensor_data)
    observation = node.create_publisher(BallObservationV2, '/hope/ball/observation_v2', qos_profile_sensor_data)
    context = node.create_publisher(FlightContextV2, '/hope/shadow/flight_v2/context', 32)
    base = node.create_publisher(Float64MultiArray, '/hope/robot/base_pose', qos_profile_sensor_data)
    sequence = 0
    def spin(duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.001)
    def pose(x=-.43, world=1):
        nonlocal sequence
        sequence += 1
        ns = time.time_ns()
        message = Float64MultiArray()
        message.data = [2., 1., float(sequence), float(ns//10**9), float(ns%10**9),
                        x, -.2, 0., 0., 0., 0., 1., 1., 47., 7., float(world)]
        base.publish(message)
    def evidence(operation, **fields):
        message = FlightContextV2()
        message.session_id, message.authority, message.operation = session, 'fixture', operation
        message.source_time_ns = time.time_ns()
        for name, value in fields.items():
            setattr(message, name, value)
        context.publish(message)
    def measurement(ns, xyz, omega=None, clock='ros_system', frame='table', mapping_valid=True):
        message = BallObservationV2()
        message.source_time_ns, message.source_time_valid = ns, True
        message.clock_domain, message.clock_mapping_id, message.clock_mapping_valid = clock, 'fixture_wall', mapping_valid
        message.frame_id, message.ball_id, message.position_valid = frame, 'fixture', True
        message.position.x, message.position.y, message.position.z = [float(v) for v in xyz]
        message.orientation.w = 1.
        if omega is not None:
            message.omega_valid = True
            message.omega_rad_s.x, message.omega_rad_s.y, message.omega_rad_s.z = [float(v) for v in omega]
        observation.publish(message)
    try:
        spin(.8)
        assert process.poll() is None, process.stderr.read()
        for _ in range(12):
            pose()
            spin(.002)
        if policy == 'ON_REFRESH':
            evidence('REFRESH_PLANE')
            spin(.015)
        at, contacts = independent_trace(direction)
        start, schedule = time.time_ns(), time.monotonic()
        context_sent = continuity_sent = False
        sent_stamps = []
        for i in range(round(.82*rate)):
            if round(.32*rate) <= i < round(.32*rate)+2:
                continue
            deadline = schedule+i/rate+(.00015 if i%3 == 0 else 0)
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.001)
            ns = time.time_ns()
            t = (ns-start)*1e-9
            state, omega = at(t)
            raw_spin = omega if i == 0 or contacts[1]+.035 < t < contacts[1]+.055 else None
            if .075 < t < .10:
                raw_spin = np.zeros(3)  # Missing legacy sentinel.
            if .10 < t < .12:
                raw_spin = np.array([math.nan, 99., -88.])  # Rejected whole vector.
            measurement(ns, state[:3], raw_spin)
            sent_stamps.append(ns)
            if i % max(1, rate//40) == 0:
                pose(-.43 if t < .35 else -.33)
            current = [p for p, _ in predictions if p.physical_available]
            if current and .10 < t < contacts[0] and not context_sent:
                p = current[-1]
                evidence('CONTEXT', source_time_ns=p.prediction_reference_time_ns,
                         shot_generation=p.shot_generation, match_context='SERVE')
                context_sent = True
            if current and t > contacts[0]+.07 and not continuity_sent:
                p = current[-1]
                evidence('CONTEXT', source_time_ns=p.prediction_reference_time_ns,
                         shot_generation=p.shot_generation, match_context='SERVE', history_continuity='complete')
                continuity_sent = True
            rclpy.spin_once(node, timeout_sec=0)
        spin(.045)
        physical = [p for p, _ in predictions if p.physical_available]
        assert physical and context_sent and continuity_sent
        generations = {p.shot_generation for p in physical}
        assert len(generations) == 1, [(p.play_state, p.reason) for p, _ in predictions]
        generation = physical[0].shot_generation
        assert any(p.match_context == 'SERVE' and p.classification_status == 'PROVISIONAL'
                   and p.play_state == ('RALLY' if direction > 0 else 'OPPONENT_RALLY') for p in physical), 'premature serve confirmation'
        assert any(p.classification_status == 'CONFIRMED' and p.play_state == ('SERVE' if direction > 0 else 'OPPONENT_SERVE') for p in physical), 'observed B1 did not confirm serve'
        confirmed = {e.event_id: e for p in physical for e in p.events
                     if e.kind == 'table_bounce' and e.status == 'CONFIRMED'}
        assert len(confirmed) >= 2, (direction,rate,policy,contacts,sorted({(e.event_id,e.status,round((e.event_time_ns-start)*1e-9,4),round(e.position.x,3)) for p in physical for e in p.events if e.kind=='table_bounce'}),[p.reason for p,_ in predictions if not p.physical_available])
        b1 = min(confirmed.values(), key=lambda e:e.event_time_ns)
        assert any(e.event_id == b1.event_id and e.status == 'PREDICTED'
                   for p in physical for e in p.events), ('B1 identity changed on confirmation',b1.event_id,(b1.event_time_ns-start)*1e-9, sorted({(e.event_id,e.status,round((e.event_time_ns-start)*1e-9,4)) for p in physical for e in p.events if e.kind=='table_bounce' and e.event_time_ns<start+round(contacts[0]*1e9)+50_000_000}))
        assert b1.confirmed_at_ns >= b1.event_time_ns
        modes = {p.current_state.spin_mode for p in physical}
        assert {'HELD', 'CONTACT_PROPAGATED', 'MEASURED'} <= modes, modes
        assert any(p.current_state.spin_measurement_time_ns == sent_stamps[0] and
                   p.current_state.spin_mode == 'CONTACT_PROPAGATED' for p in physical)
        assert any(p.target_valid and not p.target_historical for p in physical), ('no supported ordered ball target',direction,rate,policy,[(p.reason,p.plane_available,p.history_continuity,p.target_kind,round(p.current_state.position.x,3)) for p in physical[-8:]],sorted({p.reason for p in physical}))
        planes = {round(p.plane_x, 4) for p in physical if p.plane_available}
        assert .15 in planes
        assert (.25 in planes) == (policy == 'FOLLOW_POSE'), planes
        assert packets and any(len(p.event_ledger) and p.checkpoint_spin_available for p in packets)
        for p in physical:
            assert p.shot_id == generation
            assert p.current_state.position.z < .76
            if p.target_valid and not p.target_historical:
                # X05: -.2 m lies outside the removed planner Y gate [-1.2,-.3].
                # A physical forecast remains usable without robot admission.
                assert p.target_position.y > -.3
                assert abs(p.target_position.y+.2) < 1e-6
                assert p.predicted_arrival_time_ns == p.prediction_reference_time_ns+p.arrival_dt_ns
        # A real new same-direction boundary clears spin and retires pending work.
        boundary = time.time_ns()
        evidence('START', source_time_ns=boundary, boundary_evidence='RACKET_CONTACT',
                 direction=direction, history_continuity='complete')
        spin(.015)
        for i in range(12):
            ns = time.time_ns()
            t = (ns-boundary)*1e-9
            measurement(ns, (1.8 if direction < 0 else .3, -.2, 1.-4.905*t*t))
            spin(1/rate)
        resets=[p for p,_ in predictions if p.forecast_status=='REVOKED' and p.shot_generation>generation and p.published_at_ns>=boundary]
        assert resets, 'new-shot revocation/ack missing'
        fence=min(p.published_at_ns for p in resets)
        new = [p for p, _ in predictions if p.published_at_ns > fence and p.physical_available]
        assert new and all(p.shot_generation != generation for p in new)
        assert all(not p.current_state.spin_available for p in new)
        measurement(time.time_ns(), (1.8, -.2, .8), mapping_valid=False)
        spin(.02)
        assert any(p.reason == 'clock_mapping_unavailable' for p, _ in predictions)
        measurement(time.time_ns(), (1.8, -.2, .8), clock='unregistered')
        spin(.02)
        assert any(p.reason=='source_clock_domain_mismatch' and p.play_state=='INVALID' for p,_ in predictions)
        if policy == 'FOLLOW_POSE':
            # Fresh ball data with stale pose must retain physical forecasts.
            spin(.22)
            begin = time.time_ns()
            evidence('START', source_time_ns=begin, boundary_evidence='RACKET_CONTACT',
                     direction=-1, history_continuity='complete')
            spin(.01)
            for i in range(16):
                ns=time.time_ns(); t=(ns-begin)*1e-9
                measurement(ns, (2.-3*t, -.2, 1.-4.905*t*t))
                spin(1/rate)
            recent=[p for p,_ in predictions if p.prediction_reference_time_ns>=begin and p.physical_available]
            assert recent and all(not p.plane_available and not p.target_valid for p in recent)
            assert all(not any(e.kind=='receiving_plane' and e.status=='PREDICTED' for e in p.events) for p in recent)
            stable_generation=recent[-1].shot_generation
            for _ in range(12):
                pose(-.33)
                spin(.002)
            for i in range(4):
                pose(-.33);spin(.002)
                ns=time.time_ns()-5_000_000;t=(ns-begin)*1e-9
                measurement(ns,(2.-3*t,-.2,1.-4.905*t*t));spin(1/rate)
            assert any(p.plane_available and p.shot_generation==stable_generation for p,_ in predictions if p.prediction_reference_time_ns>=begin)
            assert any(p.plane_available and p.plane_pose_source_time_ns>p.prediction_reference_time_ns for p,_ in predictions if p.prediction_reference_time_ns>=begin), 'fresh pose rejected against an older ball timestamp'
            pose(-.33, world=9)
            spin(.02)
            assert any(p.reason == 'semantic_context_superseded' and p.play_state=='INVALID' for p,_ in predictions)
        return dict(direction=direction, rate_hz=rate, policy=policy, physical_forecasts=len(physical),
                    confirmed_bounces=len(confirmed), spin_modes=sorted(modes), planes=sorted(planes),
                    generation=generation, automatic_acquisition=True, source_stamps_preserved=True, outside_legacy_robot_y_gate=True)
    finally:
        node.destroy_node()
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--executable',required=True)
    parser.add_argument('--output')
    parser.add_argument('--case',nargs=3,metavar=('DIRECTION','RATE','POLICY'))
    args=parser.parse_args()
    rclpy.init()
    try:
        if args.case:
            results=[scenario(args.executable,int(args.case[0]),int(args.case[1]),args.case[2])]
        else:
            results=[scenario(args.executable,direction,rate,'FOLLOW_POSE')
                     for direction in (-1,1) for rate in (150,300)]
            results += [scenario(args.executable,direction,150,'ON_REFRESH') for direction in (-1,1)]
        report=dict(status='PASS',truth='synthetic independent DOP853 fixture; not physical qualification',scenarios=results)
        print(json.dumps(report))
        if args.output:
            with open(args.output,'w',encoding='utf-8') as output:
                json.dump(report,output,indent=2)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
