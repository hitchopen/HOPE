from pathlib import Path
import subprocess
import sys
import unittest


A3_DIR = Path(__file__).resolve().parents[1] / "a3"
sys.path.insert(0, str(A3_DIR))

from hope_runner_adapter_core import (  # noqa: E402
    COMMAND_PASSIVE,
    COMMAND_POLICY,
    COMMAND_PREPARE,
    MODE_PD_STAND,
    RunnerStatus,
    SshRunnerClient,
    decode_mode_command,
    execute_mode_command,
    parse_helper_status,
)


def status_line(
    *,
    state="STOPPED",
    request_seq=0,
    applied_seq=0,
    result=0,
    fault=0,
    reason="NONE",
    pd_ticks=0,
):
    return (
        f"A3CTL_V1 state={state} run_id=none pid=0 start_ticks=0 "
        f"pd_ticks={pd_ticks} request_seq={request_seq} "
        f"applied_seq={applied_seq} result={result} fault={fault} "
        f"reason={reason}\n"
    )


def make_status(**kwargs):
    return parse_helper_status(status_line(**kwargs))


class FakeClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def invoke(self, *args, **_kwargs):
        self.calls.append(args)
        if not self.responses:
            raise AssertionError(f"unexpected helper call: {args}")
        return self.responses.pop(0)


class ContractTests(unittest.TestCase):
    def test_status_parser_is_exact_and_wire_never_claims_base_consumption(self):
        parsed = make_status(
            state="PD_READY",
            request_seq=41,
            applied_seq=41,
            result=1,
            pd_ticks=175,
        )
        self.assertTrue(parsed.pd_ready)
        self.assertEqual(parsed.mode, MODE_PD_STAND)
        self.assertEqual(parsed.wire_data()[6], 0.0)

        with self.assertRaisesRegex(ValueError, "exactly one"):
            parse_helper_status("")
        with self.assertRaisesRegex(ValueError, "fields"):
            parse_helper_status(status_line().replace(" reason=NONE", ""))
        with self.assertRaisesRegex(ValueError, "out of range"):
            parse_helper_status(status_line(result=2))

    def test_mode_command_rejects_malformed_or_unsafe_values(self):
        self.assertEqual(decode_mode_command([1.0, 12.0, 0.0]), (12, 0))
        for values in (
            [1.0, 12.5, 0.0],
            [1.0, float("nan"), 0.0],
            [1.0, 12.0, 3.0],
            [2.0, 12.0, 0.0],
            [1.0, 12.0],
        ):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    decode_mode_command(values)

    def test_prepare_starts_stock_runner_then_sends_existing_prepare_key(self):
        sequence = 101
        client = FakeClient(
            [
                make_status(state="STOPPED", request_seq=sequence),
                make_status(state="STARTING", request_seq=sequence),
                make_status(state="IDLE", request_seq=sequence),
                make_status(
                    state="PD_RAMP",
                    request_seq=sequence,
                    applied_seq=sequence,
                    result=1,
                ),
            ]
        )
        result = execute_mode_command(
            client,
            sequence,
            COMMAND_PREPARE,
            sleep=lambda _seconds: None,
        )
        self.assertEqual(result.state, "PD_RAMP")
        self.assertEqual(
            client.calls,
            [
                ("claim", "101", "prepare"),
                ("start", "101"),
                ("status",),
                ("key", "prepare", "101"),
            ],
        )

    def test_newer_remote_claim_prevents_old_command_from_sending_a_key(self):
        client = FakeClient(
            [make_status(state="STOPPED", request_seq=202, result=0)]
        )
        result = execute_mode_command(client, 201, COMMAND_PREPARE)
        self.assertEqual(result.request_seq, 202)
        self.assertEqual(client.calls, [("claim", "201", "prepare")])

    def test_passive_when_stopped_is_idempotent_and_policy_uses_m_key(self):
        passive_client = FakeClient(
            [
                make_status(
                    state="STOPPED",
                    request_seq=301,
                    applied_seq=301,
                    result=1,
                    reason="ALREADY_STOPPED",
                )
            ]
        )
        passive = execute_mode_command(
            passive_client, 301, COMMAND_PASSIVE
        )
        self.assertEqual(passive.state, "STOPPED")
        self.assertEqual(
            passive_client.calls, [("claim", "301", "passive")]
        )

        policy_client = FakeClient(
            [
                make_status(state="PD_READY", request_seq=302, pd_ticks=175),
                make_status(
                    state="MOTION",
                    request_seq=302,
                    applied_seq=302,
                    result=1,
                ),
            ]
        )
        policy = execute_mode_command(policy_client, 302, COMMAND_POLICY)
        self.assertEqual(policy.state, "MOTION")
        self.assertEqual(
            policy_client.calls,
            [("claim", "302", "policy"), ("key", "policy", "302")],
        )

    def test_ssh_client_uses_fixed_host_helper_and_no_shell(self):
        observed = {}

        def fake_run(argv, **kwargs):
            observed["argv"] = argv
            observed["kwargs"] = kwargs
            return subprocess.CompletedProcess(
                argv, 0, stdout=status_line(), stderr=""
            )

        client = SshRunnerClient(run=fake_run)
        client.invoke("status")
        self.assertEqual(observed["argv"][0], "/usr/bin/ssh")
        self.assertIn("agi@10.42.10.12", observed["argv"])
        self.assertIn(
            "/agibot/a3_deploy_model21800/hope_model21800_runner.sh",
            observed["argv"],
        )
        self.assertNotIn("shell", observed["kwargs"])
        with self.assertRaises(ValueError):
            client.invoke("key", "x", "1")


class StaticHelperSafetyTests(unittest.TestCase):
    def test_helper_is_pinned_and_has_no_broad_kill_or_auto_start(self):
        helper = (A3_DIR / "hope_model21800_runner.sh").read_text()
        self.assertIn('DEPLOY_DIR="/agibot/a3_deploy_model21800"', helper)
        self.assertIn("./run_a3.sh --frame-log-interval=25", helper)
        self.assertNotIn("--auto-start", helper)
        self.assertNotIn("pkill", helper)
        self.assertNotIn("killall", helper)
        self.assertIn('kill -KILL "${exact_pid}"', helper)
        self.assertIn("vendor_motion_control_running", helper)
        self.assertIn('tmux_ctl send-keys -t "${TMUX_PANE}" -l "${key}"', helper)
        self.assertNotIn("send-keys -t \"${TMUX_PANE}\" Enter", helper)


if __name__ == "__main__":
    unittest.main()
