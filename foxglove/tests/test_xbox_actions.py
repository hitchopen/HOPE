import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parents[1] / 'laptop'))
from hope_xbox_actions import ActionSequence, ButtonEdges, EmergencyChord, face_button_names
from hope_xbox_preview import TriggerLatch


def state(mode='PD_STAND', role='UNASSIGNED', serve='IDLE', **kw):
    values = dict(run_mode=mode, local_role=role, serve_state=serve, boot_id=42,
                  command_fault_latched=False, serve_capability='AVAILABLE',
                  last_action='NONE', last_action_id=0)
    values.update(kw)
    return SimpleNamespace(**values)


class XboxActionTests(unittest.TestCase):
    def test_xpad_letter_codes_do_not_swap_x_and_y(self):
        self.assertEqual(face_button_names([304]), {'A'})
        self.assertEqual(face_button_names([305]), {'B'})
        self.assertEqual(face_button_names([307]), {'X'})
        self.assertEqual(face_button_names([308]), {'Y'})
        self.assertEqual(face_button_names([310, 311]), set())

    def test_lt_range_and_hysteresis(self):
        lt = TriggerLatch()
        for raw, pressed in [(0, False), (511, False), (600, True), (500, True), (300, False)]:
            self.assertEqual(lt.sample(raw, 0, 1023)[0], pressed)
        self.assertTrue(lt.sample(255, 0, 255)[0])
        with self.assertRaises(ValueError):
            lt.sample(0, 1, 1)

    def test_face_keys_are_edges_not_hold_or_reconnect_replay(self):
        keys = ButtonEdges()
        self.assertIsNone(keys.sample(True, {'A'}))
        keys.sample(True, set())
        self.assertEqual(keys.sample(True, {'A'}), 'A')
        for _ in range(100):
            self.assertIsNone(keys.sample(True, {'A'}))
        keys.sample(False, {'A'})
        self.assertIsNone(keys.sample(True, {'A'}))
        keys.sample(True, set())
        self.assertIsNone(keys.sample(True, {'A','B'}))
        self.assertIsNone(keys.sample(True, {'B'}))
        keys.sample(True, set())
        self.assertEqual(keys.sample(True, {'B'}), 'B')

    def test_a_from_teleop_waits_for_actual_stand_and_role_ack(self):
        seq = ActionSequence()
        self.assertTrue(seq.request('A', state('TELEOP'), 0))
        self.assertEqual(seq.advance(state('TELEOP'), .01), 'enter_pd_stand')
        seq.acknowledge(True, 'ACCEPTED_PENDING')
        self.assertIsNone(seq.advance(state('TELEOP'), 1))
        self.assertEqual(seq.advance(state(), 2), 'set_server')
        self.assertIsNone(seq.advance(state(role='SERVER'), 2.1))  # RPC not acknowledged
        seq.acknowledge(True)
        self.assertEqual(seq.advance(state(role='SERVER'), 2.2), 'prepare_serve')
        seq.acknowledge(True)
        seq.advance(state('SERVE','SERVER','PREPARING_STAND'), 2.3)
        self.assertFalse(seq.busy)

    def test_a_from_server_ready_preserves_direct_smooth_loop(self):
        seq = ActionSequence()
        seq.request('A', state('MOTION','SERVER','COMPLETE'), 0)
        self.assertEqual(seq.advance(state('MOTION','SERVER','COMPLETE'), .1), 'prepare_serve')

    def test_b_does_not_queue_a_future_ball_release(self):
        seq = ActionSequence()
        self.assertFalse(seq.request('B', state('SERVE','SERVER','TRANSITION_TO_LOAD'), 0))
        self.assertIsNone(seq.advance(state('SERVE','SERVER','WAIT_READY_TO_SERVE'), 6))
        self.assertTrue(seq.request('B', state('SERVE','SERVER','WAIT_READY_TO_SERVE'), 6))
        self.assertEqual(seq.advance(state('SERVE','SERVER','WAIT_READY_TO_SERVE'), 6), 'ready_to_serve')
        seq.acknowledge(True)
        seq.advance(state('SERVE','SERVER','PLAYING_PRE_RELEASE'), 6.1)
        self.assertFalse(seq.busy)
        self.assertIn('automatically', seq.status)

    def test_b_acknowledges_completed_kernel_serve_in_stand(self):
        seq = ActionSequence()
        self.assertTrue(seq.request('B', state('SERVE', 'SERVER', 'WAIT_READY_TO_SERVE'), 0))
        self.assertEqual(seq.advance(state('SERVE', 'SERVER', 'WAIT_READY_TO_SERVE'), 0), 'ready_to_serve')
        seq.acknowledge(True)
        self.assertIsNone(seq.advance(state('PD_STAND', 'SERVER', 'COMPLETE'), 5))
        self.assertFalse(seq.busy)

    def test_x_receiver_ready_and_y_only_from_stand(self):
        seq = ActionSequence()
        seq.request('X', state('MOTION','SERVER','COMPLETE'), 0)
        self.assertEqual(seq.advance(state('MOTION','SERVER','COMPLETE'), 0), 'enter_pd_stand')
        seq.acknowledge(True)
        self.assertEqual(seq.advance(state(role='SERVER'), .1), 'set_receiver')
        seq.acknowledge(True)
        self.assertEqual(seq.advance(state(role='RECEIVER'), .2), 'enter_motion')
        seq.acknowledge(True)
        seq.advance(state('MOTION','RECEIVER'), .3)
        self.assertFalse(seq.busy)
        self.assertFalse(seq.request('Y', state('MOTION','RECEIVER'), .4))
        self.assertIsNone(seq.advance(state(role='RECEIVER'), .5))
        self.assertTrue(seq.request('Y', state(role='RECEIVER'), .5))
        self.assertEqual(seq.advance(state(role='RECEIVER'), .5), 'enter_teleop')
        seq.acknowledge(True)
        seq.advance(state('TELEOP','RECEIVER'), .6)
        self.assertFalse(seq.busy)

    def test_y_during_serve_never_queues_locomotion_after_recovery(self):
        for phase in ('PREPARING_STAND', 'WAIT_READY_TO_SERVE', 'STRIKE', 'RECOVERY'):
            seq = ActionSequence()
            keys = ButtonEdges()
            keys.sample(True, set())
            self.assertFalse(seq.request(keys.sample(True, {'Y'}), state('SERVE', 'SERVER', phase), 0))
            for now in (1, 2, 10):
                self.assertIsNone(keys.sample(True, {'Y'}))
                self.assertIsNone(seq.advance(state('PD_STAND', 'SERVER', 'COMPLETE'), now))
            self.assertFalse(seq.busy)
            keys.sample(True, set())
            self.assertTrue(seq.request(keys.sample(True, {'Y'}), state(role='SERVER'), 11))
            self.assertEqual(seq.advance(state(role='SERVER'), 11), 'enter_teleop')

    def test_y_is_cancelled_if_stand_is_left_before_dispatch(self):
        seq = ActionSequence()
        self.assertTrue(seq.request('Y', state(), 0))
        self.assertIsNone(seq.advance(state('SERVE', 'SERVER', 'PREPARING_STAND'), .01))
        self.assertFalse(seq.busy)
        self.assertIsNone(seq.advance(state(), 10))

    def test_reconnect_with_held_y_never_enters_locomotion(self):
        keys, seq = ButtonEdges(), ActionSequence()
        keys.sample(False, set())
        self.assertIsNone(keys.sample(True, {'Y'}))
        self.assertIsNone(seq.advance(state(), 1))
        keys.sample(True, set())
        self.assertTrue(seq.request(keys.sample(True, {'Y'}), state(), 2))
        self.assertEqual(seq.advance(state(), 2), 'enter_teleop')

    def test_disconnect_boot_fault_timeout_and_rejection_cancel_tail(self):
        for bad, now, connected in [(None,1,True), (state(boot_id=43),1,True),
             (state(command_fault_latched=True),1,True), (state(),21,True), (state(),1,False),
             (state(last_action='EMERGENCY_PASSIVE',last_action_id=2),1,True)]:
            seq = ActionSequence()
            seq.request('A', state('TELEOP'), 0)
            seq.advance(state('TELEOP'), 0)
            seq.acknowledge(True)
            self.assertIsNone(seq.advance(bad, now, connected))
            self.assertFalse(seq.busy)
            self.assertIsNone(seq.advance(state(), now+1))
        seq.request('Y', state(), 30)
        seq.advance(state(), 30)
        seq.acknowledge(False, 'not ready')
        self.assertFalse(seq.busy)

    def test_existing_passive_receipt_does_not_block_explicit_recovery(self):
        s = state('PASSIVE', last_action='EMERGENCY_PASSIVE', last_action_id=4)
        seq = ActionSequence()
        seq.request('A', s, 0)
        self.assertEqual(seq.advance(s, 0), 'enter_pd_stand')

    def test_busy_button_is_not_replayed_later(self):
        seq = ActionSequence()
        seq.request('Y', state(), 0)
        self.assertFalse(seq.request('B', state('SERVE','SERVER','WAIT_READY_TO_SERVE'), .1))
        self.assertEqual(seq.advance(state(), .1), 'enter_teleop')

    def test_lb_rb_emergency_latches_without_lt_or_runner_and_release_does_not_clear(self):
        e = EmergencyChord()
        e.sample(True, False)
        self.assertFalse(e.pending)
        e.sample(True, True)
        self.assertTrue(e.pending)
        e.pending = False
        e.sample(True, True)
        self.assertFalse(e.pending)
        e.observe_latch(False)  # old false before remote processes E-stop
        self.assertTrue(e.latched)
        e.sample(False, False)
        self.assertTrue(e.latched)
        e.observe_latch(True)
        e.status = 'E-STOP incomplete: vendor unavailable'
        e.observe_latch(True)
        self.assertIn('incomplete', e.status)
        e.observe_latch(False)  # only an authoritative recovery clears it
        self.assertFalse(e.latched)


if __name__ == '__main__':
    unittest.main()
