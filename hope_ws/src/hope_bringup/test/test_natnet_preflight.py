import importlib.util
import struct
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "natnet_preflight.py"
OPTITRACK_SOURCE = (
    SCRIPT.parents[4]
    / "NatNet2ROS2/src/motion_capture_tracking/deps/libmotioncapture/src/optitrack.cpp"
)
SPEC = importlib.util.spec_from_file_location("natnet_preflight", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class NatNetPreflightTest(unittest.TestCase):
    def test_packet_validation_rejects_short_or_truncated_packets(self):
        self.assertIsNone(MODULE.packet_message_id(b""))
        self.assertFalse(MODULE.packet_is_complete(b"\x05\x00\x04\x00"))
        self.assertTrue(
            MODULE.packet_is_complete(
                struct.pack("<HHI", MODULE.NAT_MODELDEF, 4, 0), 8
            )
        )

    def test_masked_modeldef_is_required_by_driver_contract(self):
        packet = struct.pack("<HHI", MODULE.NAT_MODELDEF, 4, 0)
        self.assertIsNone(MODULE.modeldef_contract_blocker(packet, None))
        self.assertIn(
            "answers only payload-less",
            MODULE.modeldef_contract_blocker(None, packet),
        )

    def test_modeldef_rejects_unframed_asset_name_scan(self):
        packet = b"\x05\x00\x00\x00Ball\x00all\x00UCB_P1\x00Ball_Marker1\x00"
        with self.assertRaises(ValueError):
            MODULE.modeldef_assets(packet)

    def test_generic_asset_aliases_preserve_canonical_contract(self):
        aliases = MODULE.parse_asset_aliases(
            ["SITE_A=CANONICAL_A", "SITE_B=CANONICAL_B"]
        )
        self.assertEqual(
            MODULE.canonical_assets(["Ball", "SITE_A"], aliases),
            ["Ball", "CANONICAL_A"],
        )
        self.assertEqual(
            MODULE.canonical_assets(["Ball", "UCB_P1"], aliases),
            ["Ball", "UCB_P1"],
        )
        with self.assertRaises(ValueError):
            MODULE.parse_asset_aliases(
                ["SITE_A=CANONICAL_A", "SITE_B=CANONICAL_A"]
            )

    def test_default_ball_alias_accepts_both_spellings(self):
        aliases = MODULE.parse_asset_aliases(
            list(MODULE.DEFAULT_ASSET_ALIAS_ENTRIES)
        )
        self.assertEqual(
            MODULE.canonical_assets(["ball", "UCB_P1"], aliases),
            ["Ball", "UCB_P1"],
        )
        self.assertEqual(
            MODULE.canonical_assets(["Ball", "UCB_P1"], aliases),
            ["Ball", "UCB_P1"],
        )
        runtime_config = (
            SCRIPT.parents[1] / "config" / "optitrack_mct.yaml"
        ).read_text()
        self.assertIn(
            'rigid_body_name_aliases: ["ball=Ball", "P1=UCB_P1", "P2=UCB_P2"]', runtime_config
        )

    def test_modeldef_exact_ball_wins_if_both_spellings_exist(self):
        def modeldef_body(name, rigid_body_id):
            return (
                struct.pack("<i", 1)
                + name.encode()
                + b"\x00"
                + struct.pack("<ii3fi", rigid_body_id, -1, 0, 0, 0, 1)
                + struct.pack("<3fi", 0.1, 0.2, 0.3, 0)
                + b"Marker 001\x00"
            )

        aliases = MODULE.parse_asset_aliases(
            list(MODULE.DEFAULT_ASSET_ALIAS_ENTRIES)
        )
        lower_description = modeldef_body("ball", 11)
        lower_payload = struct.pack("<i", 1) + lower_description
        lower_packet = struct.pack(
            "<HH", MODULE.NAT_MODELDEF, len(lower_payload)
        ) + lower_payload
        self.assertEqual(
            MODULE.modeldef_rigid_body_ids(lower_packet, 4, 5, aliases),
            {"Ball": 11},
        )

        both_descriptions = modeldef_body("ball", 11) + modeldef_body(
            "Ball", 10
        )
        both_payload = struct.pack("<i", 2) + both_descriptions
        both_packet = struct.pack(
            "<HH", MODULE.NAT_MODELDEF, len(both_payload)
        ) + both_payload
        self.assertEqual(
            MODULE.modeldef_rigid_body_ids(both_packet, 4, 5, aliases),
            {"Ball": 10},
        )

    def test_table_side_maps_to_ucb_robot_asset(self):
        self.assertEqual(MODULE.robot_asset_for_table_side("P1"), "UCB_P1")
        self.assertEqual(MODULE.robot_asset_for_table_side("P2"), "UCB_P2")

    def test_natnet_45_modeldef_ids_and_live_tracking_flags(self):
        def modeldef_body(name, rigid_body_id):
            return (
                struct.pack("<i", 1)
                + name.encode()
                + b"\x00"
                + struct.pack(
                    "<ii3fi", rigid_body_id, -1, 0, 0, 0, 1
                )
                + struct.pack("<3fi", 0.1, 0.2, 0.3, 0)
                + b"Marker 001\x00"
            )

        descriptions = modeldef_body("UCB_P1", 17) + modeldef_body("szu_P1", 23)
        modeldef_payload = struct.pack("<i", 2) + descriptions
        modeldef = struct.pack(
            "<HH", MODULE.NAT_MODELDEF, len(modeldef_payload)
        ) + modeldef_payload
        self.assertEqual(
            MODULE.modeldef_rigid_body_ids(modeldef, 4, 5, {}),
            {"UCB_P1": 17, "szu_P1": 23},
        )

        rigid_bodies = (
            struct.pack("<I8fH", 17, 0, 0, 0, 0, 0, 0, 1, 0.001, 1)
            + struct.pack("<I8fH", 23, 0, 0, 0, 0, 0, 0, 1, 0.001, 0)
        )
        frame_payload = (
            struct.pack("<Ii", 42, 1)
            + b"all\x00"
            + struct.pack("<i", 0)
            + struct.pack("<i", 0)
            + struct.pack("<i", 2)
            + rigid_bodies
        )
        frame = struct.pack(
            "<HH", MODULE.NAT_FRAMEOFDATA, len(frame_payload)
        ) + frame_payload
        self.assertEqual(
            MODULE.frame_rigid_body_tracking(frame, 4, 5),
            {17: True, 23: False},
        )

    def test_natnet_45_labeled_marker_counts_are_model_specific(self):
        labeled = b"".join(
            struct.pack(
                "<I4fHf",
                (17 << 16) | member_id,
                0.0,
                0.0,
                0.0,
                0.01,
                0x0A,
                0.0001,
            )
            for member_id in range(10)
        )
        frame_payload = (
            struct.pack("<I", 43)
            + struct.pack("<i", 1)  # marker-set count
            + b"all\x00"
            + struct.pack("<i", 0)  # markers in set
            + struct.pack("<i", 0)  # unlabeled markers
            + struct.pack("<i", 0)  # rigid bodies
            + struct.pack("<i", 0)  # skeletons
            + struct.pack("<i", 10)  # labeled markers
            + labeled
        )
        frame = struct.pack(
            "<HH", MODULE.NAT_FRAMEOFDATA, len(frame_payload)
        ) + frame_payload
        self.assertEqual(
            MODULE.frame_labeled_marker_model_counts(frame, 4, 5),
            (10, {17: 10}),
        )

    def test_multicast_uses_motive_route_not_group_default_route(self):
        source = SCRIPT.read_text()
        self.assertIn("selected by Motive route", source)
        self.assertIn("joins on the Motive interface explicitly", source)
        self.assertNotIn("switch Motive to Unicast", source)

        driver = OPTITRACK_SOURCE.read_text()
        self.assertIn("make_address_v4(interface_ip)", driver)
        self.assertIn("multicast_address_boost, listen_address_boost", driver)
        self.assertIn("Using unicast from server", driver)
        self.assertIn("connect_from_data", driver)
        self.assertIn("NAT_KEEPALIVE", driver)

    def test_natnet_45_uses_bounded_public_parsers(self):
        driver = OPTITRACK_SOURCE.read_text()
        self.assertIn("parseNatNetSizedFrame", driver)
        self.assertIn("parseNatNetModelDef", driver)
        self.assertIn("parseModelDef(modelDef.data(), modelDef.size())", driver)


if __name__ == "__main__":
    unittest.main()
