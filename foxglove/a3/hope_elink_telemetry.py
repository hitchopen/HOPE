"""Passive vendor-message freshness; never an ELink probe or recovery client.

Fresh ROS envelopes are NOT automatically fresh hardware measurements. An
audited, build-specific acquisition contract is required before this helper
can report functional_ready. Even then it establishes only the contracted
telemetry conditions, not actuator safety or safe native-stack recovery.
These diagnostics have no lifecycle or motion-admission authority.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
from typing import Mapping


CONFIG_PATH = Path("/etc/hope-foxglove/elink_telemetry.json")
CHANNELS = ("left_hand", "right_hand", "bms", "emergency")
HAND_VARIANTS = frozenset(("inspire", "claw", "agi_claw_state", "agi_hand", "agi_o10_hand"))
DEFAULT_TOPICS = {
    "left_hand": "/body_drive/hand_joint_state/pb_3Aaimdk_2Eprotocol_2EHandStateChannel",
    "right_hand": "/body_drive/hand_joint_state/pb_3Aaimdk_2Eprotocol_2EHandStateChannel",
    "bms": "/aima/bms/data/pb_3Aaimdk_2Eprotocol_2EBmsStateChannel",
    "emergency": "/hal_state/emergency/pb_3Aaimdk_2Eprotocol_2EEmergencyStateChannel",
}
SHA256 = re.compile(r"[0-9a-f]{64}")
BOOT_ID = re.compile(r"[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}")
MAX_UTC_NS = 253402300800_000000000
MAX_DIAGNOSTICS = 17  # 16 battery slots plus an adapter-level advisory.
MAX_DIAGNOSTIC_LENGTH = 256  # Keep combined health below its 16 KiB IPC bound.


def _number(value, *, positive=False):
    try:
        return type(value) in (int, float) and math.isfinite(value) and (value > 0 if positive else value >= 0)
    except OverflowError:
        return False


def _text(value, max_length=256):
    return isinstance(value, str) and 0 < len(value) <= max_length and not any(ord(c) < 32 for c in value)


def _utc_ns(value):
    return type(value) is int and 0 < value < MAX_UTC_NS


def _bounded_diagnostics(values, *, bms_device_fault=False):
    """Keep advisory diagnostics bounded without changing freshness results.

    Preserve printable ASCII only so json.dumps cannot expand Unicode/control
    characters beyond the budget. Avoid traversing an unbounded collection.
    """
    result = ["BMS_DEVICE_FAULT_ADVISORY"] if bms_device_fault else []
    if not isinstance(values, tuple):
        return tuple(result + ["TELEMETRY_DIAGNOSTICS_FORMAT_ADVISORY"])
    remaining = MAX_DIAGNOSTICS - len(result)
    truncated = len(values) > remaining
    count = remaining - 1 if truncated else remaining
    for value in values[:count]:
        if not isinstance(value, str) or not value:
            result.append("TELEMETRY_DIAGNOSTICS_FORMAT_ADVISORY")
            continue
        shortened = len(value) > MAX_DIAGNOSTIC_LENGTH
        limit = MAX_DIAGNOSTIC_LENGTH - 14 if shortened else MAX_DIAGNOSTIC_LENGTH
        value = "".join(c if 32 <= ord(c) < 127 and c not in ('"', "\\") else "?" for c in value[:limit])
        result.append(value + ("...[truncated]" if shortened else ""))
    if truncated:
        result.append("TELEMETRY_DIAGNOSTICS_TRUNCATED_ADVISORY")
    return tuple(result)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate contract field")
        result[key] = value
    return result


@dataclass(frozen=True)
class ChannelContract:
    required: bool
    topic: str
    publisher_node: str
    stale_after_s: float
    max_source_age_s: float
    future_tolerance_s: float
    expected_variant: str = ""
    acquisition_evidence: str = ""
    fault_semantics_evidence: str = ""
    not_required_reason: str = ""


@dataclass(frozen=True)
class TelemetryContract:
    contract_id: str
    sha256: str
    audited: bool
    hardware_profile: str
    hal_library_sha256: str
    channels: Mapping[str, ChannelContract]
    error: str = ""

    @classmethod
    def unverified(cls, reason="ELINK_TELEMETRY_CONTRACT_UNVERIFIED"):
        channels = {key: ChannelContract(True, DEFAULT_TOPICS[key], "", 1.0, 1.0, 0.05)
                    for key in CHANNELS}
        return cls("unverified", "", False, "", "", channels, reason)

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != {
                "schema_version", "contract_id", "audited", "hardware_profile",
                "hal_library_sha256", "channels"}:
            raise ValueError("ELINK_TELEMETRY_CONTRACT_FIELDS_INVALID")
        if (type(value["schema_version"]) is not int or value["schema_version"] != 1
                or type(value["audited"]) is not bool or not _text(value["contract_id"])
                or not isinstance(value["channels"], dict) or set(value["channels"]) != set(CHANNELS)):
            raise ValueError("ELINK_TELEMETRY_CONTRACT_SCHEMA_INVALID")
        audited = value["audited"]
        profile, library = value["hardware_profile"], value["hal_library_sha256"]
        if not isinstance(profile, str) or not isinstance(library, str):
            raise ValueError("ELINK_TELEMETRY_CONTRACT_IDENTITY_INVALID")
        if audited and (not _text(profile) or not SHA256.fullmatch(library)):
            raise ValueError("ELINK_TELEMETRY_AUDIT_IDENTITY_REQUIRED")
        channels = {}
        fields = {"required", "topic", "publisher_node", "stale_after_s", "max_source_age_s",
                  "future_tolerance_s", "expected_variant", "acquisition_evidence",
                  "fault_semantics_evidence", "not_required_reason"}
        for key in CHANNELS:
            spec = value["channels"][key]
            if not isinstance(spec, dict) or set(spec) != fields or type(spec["required"]) is not bool:
                raise ValueError("ELINK_TELEMETRY_CHANNEL_FIELDS_INVALID:" + key)
            if (not _text(spec["topic"]) or not spec["topic"].startswith("/")
                    or not isinstance(spec["publisher_node"], str)
                    or not all(_number(spec[k], positive=True) and spec[k] <= 60
                               for k in ("stale_after_s", "max_source_age_s"))
                    or not _number(spec["future_tolerance_s"]) or spec["future_tolerance_s"] > 1
                    or not all(isinstance(spec[k], str) and len(spec[k]) <= 1024 for k in
                               ("expected_variant", "acquisition_evidence", "fault_semantics_evidence", "not_required_reason"))):
                raise ValueError("ELINK_TELEMETRY_CHANNEL_INVALID:" + key)
            if key in ("bms", "emergency") and not spec["required"]:
                raise ValueError("ELINK_TELEMETRY_CORE_CHANNEL_REQUIRED:" + key)
            if not spec["required"] and not _text(spec["not_required_reason"], 1024):
                raise ValueError("ELINK_TELEMETRY_APPLICABILITY_REASON_REQUIRED:" + key)
            if audited and spec["required"]:
                if (not _text(spec["publisher_node"]) or not spec["publisher_node"].startswith("/")
                        or not _text(spec["acquisition_evidence"], 1024)
                        or not _text(spec["fault_semantics_evidence"], 1024)):
                    raise ValueError("ELINK_TELEMETRY_CHANNEL_AUDIT_REQUIRED:" + key)
                if key.endswith("hand") and spec["expected_variant"] not in HAND_VARIANTS:
                    raise ValueError("ELINK_TELEMETRY_HAND_VARIANT_REQUIRED:" + key)
            channels[key] = ChannelContract(**spec)
        canonical = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return cls(value["contract_id"], hashlib.sha256(canonical).hexdigest(), audited,
                   profile, library, channels)


def load_telemetry_contract(path=CONFIG_PATH, *, require_root=True):
    """Bounded trusted config; unavailable means diagnostic readiness is false.

    require_root=False exists only for isolated tests/developer diagnostics.
    Deployment must use the default. The contract is configuration evidence,
    not proof by itself that a hardware verification was actually performed.
    """
    try:
        path = Path(path)
        parent = path.parent.stat()
        if require_root and (parent.st_uid != 0 or parent.st_mode & 0o022):
            raise ValueError("untrusted contract directory")
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
        with os.fdopen(fd, "rb") as stream:
            meta = os.fstat(stream.fileno())
            if not stat.S_ISREG(meta.st_mode) or meta.st_mode & 0o022 or (require_root and meta.st_uid != 0):
                raise ValueError("untrusted contract file")
            raw = stream.read(65537)
        if len(raw) > 65536:
            raise ValueError("oversized contract")
        return TelemetryContract.from_dict(json.loads(raw, object_pairs_hook=_unique_object))
    except (OSError, ValueError, TypeError, UnicodeError, RecursionError, OverflowError):
        return TelemetryContract.unverified("ELINK_TELEMETRY_CONTRACT_UNAVAILABLE_OR_INVALID")


@dataclass(frozen=True)
class TelemetrySample:
    seq: int
    source_ns: int
    publisher_node: str
    publisher_gid: str
    variant: str = ""
    payload_valid: bool = True
    device_fault: bool = False
    diagnostics: tuple[str, ...] = ()


def _identity(value):
    if hasattr(value, "to_dict"):
        value = value.to_dict()
    if (not isinstance(value, dict) or set(value) != {"boot_id", "pid", "start_ticks"}
            or not isinstance(value["boot_id"], str) or not BOOT_ID.fullmatch(value["boot_id"])
            or type(value["pid"]) is not int or value["pid"] <= 0
            or type(value["start_ticks"]) is not int or value["start_ticks"] < 0):
        raise ValueError("ELINK_TELEMETRY_GENERATION_INVALID")
    return dict(value)


class ElinkTelemetryTracker:
    """Diagnostic-only tracker; retains evidence within a publisher generation.

    Incoming callbacks must use the binding token obtained under the caller's
    shared lock. Old tokens are rejected; source-time checks separately reject
    pre-bind messages that were already queued in a middleware executor.
    """
    def __init__(self, contract=None):
        self.contract = contract if contract is not None else TelemetryContract.unverified()
        self._session = ""
        self._generation = ""
        self._last_generation = ""
        self._bound_at = None
        self._bound_wall_ns = None
        self._build_matches = False
        self._records = {}
        self._fault = ""

    @property
    def generation_id(self):
        return self._generation

    def unbind_generation(self):
        """Suspend diagnostic sampling during an intentional HAL transition.

        Retain fault evidence without persisting or enforcing it. Late
        old-generation callbacks cannot claim current freshness; a subsequent
        bind needs new data. No lifecycle or motion action depends on this.
        """
        self._session = ""
        self._generation = ""
        self._bound_at = None
        self._bound_wall_ns = None
        self._build_matches = False
        self._records = {}

    def bind_generation(self, session_id, transfer, hal, *, now, wall_ns,
                        hardware_profile="", hal_library_sha256=""):
        if (not _text(session_id, 128) or not _number(now) or not _utc_ns(wall_ns)):
            raise ValueError("ELINK_TELEMETRY_BIND_INVALID")
        transfer, hal = _identity(transfer), _identity(hal)
        if transfer["boot_id"] != hal["boot_id"]:
            raise ValueError("ELINK_TELEMETRY_GENERATION_HOST_MISMATCH")
        value = {"session_id": session_id, "transfer": transfer, "hal": hal,
                 "contract_sha256": self.contract.sha256}
        generation = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
        build_matches = (self.contract.audited and hardware_profile == self.contract.hardware_profile
                         and hal_library_sha256 == self.contract.hal_library_sha256)
        if generation != self._generation:
            self._session, self._generation = session_id, generation
            self._bound_at, self._bound_wall_ns = float(now), wall_ns
            self._records = {}
            if generation != self._last_generation:
                self._fault = ""
            self._last_generation = generation
        self._build_matches = bool(build_matches)
        return generation

    def invalidate(self, channel, reason):
        if channel not in CHANNELS:
            raise ValueError("unknown telemetry channel")
        record = self._records.setdefault(channel, {})
        record["reason"] = str(reason)[:256]
        record["progress"] = 0

    def update(self, channel, sample, *, now, wall_ns, generation_id):
        if channel not in CHANNELS:
            raise ValueError("unknown telemetry channel")
        if generation_id != self._generation or not self._generation:
            return False  # A late old-generation callback cannot affect the new session.
        spec = self.contract.channels[channel]
        if not spec.required:
            return False
        def reject(reason):
            self.invalidate(channel, reason)
            return False
        if not isinstance(sample, TelemetrySample):
            return reject("ELINK_TELEMETRY_SAMPLE_INVALID")
        if (not _number(now) or now < self._bound_at or not _utc_ns(wall_ns)
                or type(sample.seq) is not int or not 0 <= sample.seq <= 0xffffffff
                or (sample.seq == 0 and channel != "bms")
                or not _utc_ns(sample.source_ns)
                or type(sample.payload_valid) is not bool or not sample.payload_valid
                or type(sample.device_fault) is not bool
                or not _text(sample.publisher_node) or not _text(sample.publisher_gid, 128)
                or not isinstance(sample.variant, str)):
            return reject("ELINK_TELEMETRY_SAMPLE_INVALID")
        if spec.publisher_node and sample.publisher_node != spec.publisher_node:
            return reject("ELINK_TELEMETRY_PUBLISHER_MISMATCH")
        if spec.expected_variant and sample.variant != spec.expected_variant:
            return reject("ELINK_TELEMETRY_VARIANT_MISMATCH")
        age = (wall_ns - sample.source_ns) / 1e9
        if age < -spec.future_tolerance_s or age >= spec.max_source_age_s:
            return reject("ELINK_TELEMETRY_SOURCE_TIME_INVALID")
        # Exclude queued pre-bind publications even with the allowed source
        # clock lead. Wait for genuinely post-bind progress, not a fresh latch.
        if sample.source_ns <= self._bound_wall_ns + int(spec.future_tolerance_s * 1e9):
            return reject("ELINK_TELEMETRY_PREVIOUS_GENERATION_SAMPLE")
        record = self._records.setdefault(channel, {})
        if record.get("gid") and record["gid"] != sample.publisher_gid:
            self._fault = "ELINK_TELEMETRY_PUBLISHER_GENERATION_CHANGED:" + channel
            return reject(self._fault)
        # Vendor BMS publications can leave the proto3 sequence unset (zero).
        # In that case only the sequence comparison is inapplicable: source
        # timestamps and monotonic receipt times must still strictly advance.
        # Once a positive sequence is accepted, require sequence progression
        # too; a reset to zero cannot silently downgrade that check. Rejection
        # retains the last accepted seq/source/receipt high-water marks.
        bms_sequence_unset = channel == "bms" and sample.seq == 0 and record.get("seq", 0) == 0
        if ("seq" in record and (sample.source_ns <= record["source_ns"]
                                or (not bms_sequence_unset and sample.seq <= record["seq"]))
                or "received_at" in record and now <= record["received_at"]):
            return reject("ELINK_TELEMETRY_NONPROGRESSING_SAMPLE")
        if "received_at" in record:
            elapsed = now - record["received_at"]
            # UTC qualification can itself be cached briefly. A simultaneous
            # receiver/source clock step must not make every envelope fresh.
            # This tolerance is the configured age window, not a new claim
            # about safe control timing. Small clock motion still requires
            # the external clock qualification contract.
            tolerance = spec.max_source_age_s + spec.future_tolerance_s
            wall_elapsed = (wall_ns - record["receipt_wall_ns"]) / 1e9
            source_elapsed = (sample.source_ns - record["source_ns"]) / 1e9
            if abs(wall_elapsed - elapsed) > tolerance or abs(source_elapsed - elapsed) > tolerance:
                self._fault = "ELINK_TELEMETRY_CLOCK_DISCONTINUITY:" + channel
                return reject(self._fault)
        # Battery supervision belongs to the vendor system. Record BMS
        # transport freshness, but keep battery values/status advisory here
        # as well as in the decoder. No tracker result is a motion veto.
        diagnostics = _bounded_diagnostics(sample.diagnostics,
                                           bms_device_fault=channel == "bms" and sample.device_fault)
        if sample.device_fault and channel != "bms":
            self._fault = "ELINK_TELEMETRY_DEVICE_FAULT:" + channel
            return reject(self._fault)
        record.update(seq=sample.seq, source_ns=sample.source_ns, received_at=float(now),
                      receipt_wall_ns=wall_ns, gid=sample.publisher_gid,
                      publisher_node=sample.publisher_node,
                      diagnostics=diagnostics,
                      progress=min(record.get("progress", 0) + 1, 2), reason="")
        return True

    def status(self, now, wall_ns, clock_qualified):
        clock_ok = (_number(now) and _utc_ns(wall_ns)
                    and type(clock_qualified) is bool and clock_qualified)
        channels, all_fresh = {}, bool(self._generation) and clock_ok and not self._fault
        expiries = []
        for channel, spec in self.contract.channels.items():
            record = self._records.get(channel, {})
            received = record.get("received_at")
            source = record.get("source_ns")
            age = now - received if _number(now) and received is not None else None
            source_age = (wall_ns - source) / 1e9 if _utc_ns(wall_ns) and source is not None else None
            reason = record.get("reason", "ELINK_TELEMETRY_NOT_RECEIVED")
            fresh = bool(clock_ok and not reason and record.get("progress", 0) >= 2
                         and age is not None and 0 <= age < spec.stale_after_s
                         and source_age is not None and -spec.future_tolerance_s <= source_age < spec.max_source_age_s)
            if not clock_ok:
                reason = "ELINK_TELEMETRY_CLOCK_UNQUALIFIED"
            elif not reason and not fresh:
                reason = "ELINK_TELEMETRY_NEEDS_PROGRESS" if record.get("progress", 0) < 2 else "ELINK_TELEMETRY_STALE"
            channels[channel] = {"required": spec.required, "message_fresh": fresh,
                                 "reason": reason, "receipt_age_s": age,
                                 "source_age_s": source_age, "sequence": record.get("seq")}
            channels[channel]["warnings"] = list(record.get("diagnostics", ()))
            if spec.required and not fresh:
                all_fresh = False
            if spec.required and fresh:
                expiries.append(min(received + spec.stale_after_s,
                                    now + spec.max_source_age_s - source_age))
        ready = bool(all_fresh and self.contract.audited and self._build_matches and not self.contract.error)
        if self._fault:
            reason = self._fault
        elif not self._generation:
            reason = "ELINK_TELEMETRY_GENERATION_UNBOUND"
        elif not all_fresh:
            reason = "ELINK_TELEMETRY_REQUIRED_CHANNEL_UNAVAILABLE"
        elif not self.contract.audited or self.contract.error:
            reason = self.contract.error or "ELINK_TELEMETRY_CONTRACT_UNVERIFIED"
        elif not self._build_matches:
            reason = "ELINK_TELEMETRY_AUDITED_BUILD_MISMATCH"
        else:
            reason = "ELINK_TELEMETRY_AUDITED_CHANNELS_FRESH"
        return {"schema_version": 1, "session_id": self._session,
                "generation_id": self._generation, "contract_id": self.contract.contract_id,
                "contract_sha256": self.contract.sha256, "contract_audited": self.contract.audited,
                "message_fresh": bool(all_fresh), "functional_ready": ready,
                "valid_until_monotonic_s": min(expiries) if ready and expiries else None,
                "fault_latched": bool(self._fault), "reason": reason, "channels": channels}


def decode_vendor_channel(channel, message, *, publisher_node, publisher_gid):
    """Decode generated protobuf objects, not guessed wire layouts.

    No source-progress semantics are inferred here. Those belong to the
    independently reviewed acquisition contract and tracker.
    """
    expected = {"left_hand": "aimdk.protocol.HandStateChannel", "right_hand": "aimdk.protocol.HandStateChannel",
                "bms": "aimdk.protocol.BmsStateChannel", "emergency": "aimdk.protocol.EmergencyStateChannel"}
    if channel not in expected or message.DESCRIPTOR.full_name != expected[channel]:
        raise ValueError("ELINK_TELEMETRY_MESSAGE_TYPE_MISMATCH")
    if not message.HasField("header") or not message.header.HasField("timestamp"):
        raise ValueError("ELINK_TELEMETRY_HEADER_MISSING")
    stamp = message.header.timestamp
    if stamp.seconds <= 0 or not 0 <= stamp.nanos < 1_000_000_000:
        raise ValueError("ELINK_TELEMETRY_TIMESTAMP_INVALID")
    source_ns = stamp.seconds * 1_000_000_000 + stamp.nanos
    if stamp.ms_since_epoch and stamp.ms_since_epoch != source_ns // 1_000_000:
        raise ValueError("ELINK_TELEMETRY_TIMESTAMP_INCONSISTENT")
    variant, fault, diagnostics = "", False, []
    if channel.endswith("hand"):
        side = "left" if channel == "left_hand" else "right"
        if not message.HasField("data") or not message.data.HasField(side):
            raise ValueError("ELINK_TELEMETRY_HAND_MISSING")
        hand = getattr(message.data, side)
        variant = hand.WhichOneof("hand")
        if variant not in HAND_VARIANTS:
            raise ValueError("ELINK_TELEMETRY_HAND_VARIANT_MISSING")
        if variant == "agi_claw_state":
            if hand.agi_claw_state.state not in (0x11, 0x12, 0x13, 0x14, 0x21, 0x22, 0x23):
                raise ValueError("ELINK_TELEMETRY_CLAW_STATUS_UNDEFINED")
            fault = hand.agi_claw_state.state in (0x13, 0x14, 0x23)
            if not 0 <= hand.agi_claw_state.pos <= 100:
                raise ValueError("ELINK_TELEMETRY_CLAW_POSITION_INVALID")
        if variant in ("agi_hand", "agi_o10_hand"):
            finger = getattr(hand, variant)
            if not finger.HasField("finger") or not finger.finger.HasField("pos"):
                raise ValueError("ELINK_TELEMETRY_HAND_POSITION_MISSING")
            upper = 2000 if variant == "agi_hand" else 4096
            if any(not 0 <= getattr(finger.finger.pos, field.name) <= upper
                   for field in finger.finger.pos.DESCRIPTOR.fields):
                raise ValueError("ELINK_TELEMETRY_HAND_POSITION_INVALID")
            if finger.finger.HasField("toq"):
                upper = 5700 if variant == "agi_hand" else 255
                if any(not 0 <= getattr(finger.finger.toq, field.name) <= upper
                       for field in finger.finger.toq.DESCRIPTOR.fields):
                    raise ValueError("ELINK_TELEMETRY_HAND_TORQUE_INVALID")
            if variant == "agi_o10_hand" and finger.finger.HasField("pressure"):
                for field in finger.finger.pressure.DESCRIPTOR.fields:
                    values = getattr(finger.finger.pressure, field.name)
                    count = 25 if field.name in ("palm_pressures", "back_of_hand_pressures") else 16
                    if values and (len(values) != count or any(not 0 <= value <= 255 for value in values)):
                        raise ValueError("ELINK_TELEMETRY_HAND_PRESSURE_INVALID")
        if variant == "inspire" and not hand.inspire.HasField("finger"):
            raise ValueError("ELINK_TELEMETRY_HAND_POSITION_MISSING")
    elif channel == "bms":
        batteries = [("bms_datas[" + str(index) + "]", battery)
                     for index, battery in enumerate(message.bms_datas)]
        if message.HasField("data"):
            batteries.append(("data", message.data))
        if not batteries or len(batteries) > 16:
            raise ValueError("ELINK_TELEMETRY_BMS_MISSING")
        for slot, battery in batteries:
            warnings = []
            priority = {name: index for index, name in enumerate(
                ("abnormal_state", "battery_pack_state", "battery_comm_state", "bms_state", "voltage"))}
            for field in sorted(battery.DESCRIPTOR.fields, key=lambda item: priority.get(item.name, 5)):
                value = getattr(battery, field.name)
                if (field.type in (field.TYPE_FLOAT, field.TYPE_DOUBLE) and not math.isfinite(value)
                        or field.name == "voltage" and value <= 0
                        or field.name in ("abnormal_state", "battery_pack_state", "battery_comm_state") and value != 0
                        or field.name == "bms_state" and value not in (0, 1)
                        or field.name == "power_supply_health" and value not in (0, 1)
                        or field.name in ("battery_slow_start_output_state", "battery_slow_start_recharge_state") and value in (1, 4)
                        or field.type == field.TYPE_ENUM and value not in field.enum_type.values_by_number):
                    # String formatting makes NaN/Inf and unknown enum codes
                    # JSON-safe; do not copy non-finite floats into status.
                    rendered = str(value)
                    if field.type == field.TYPE_ENUM:
                        enum = field.enum_type.values_by_number.get(value)
                        if enum is not None:
                            rendered += "(" + enum.name.removeprefix(field.enum_type.name + "_") + ")"
                    warnings.append(field.name + "=" + rendered)
            if warnings:
                # One diagnostic per slot, at most 16. Future schema growth
                # is truncated deterministically, never an admission veto.
                diagnostic = "BMS_ADVISORY:" + slot + ":" + ";".join(warnings)
                if len(diagnostic) > MAX_DIAGNOSTIC_LENGTH:
                    diagnostic = diagnostic[:MAX_DIAGNOSTIC_LENGTH - 14] + "...[truncated]"
                diagnostics.append(diagnostic)
        # Zero/absent/startup values, unknown statuses and asserted battery
        # fault codes remain advisory. This is not a claim that a battery is
        # healthy. Vendor protection retains battery authority; HOPE still
        # verifies the envelope, identity, audit, clocks and sample progress.
        # Positive legacy data with omitted proto3 bms_state=0 stays accepted.
    else:
        if not message.HasField("data"):
            raise ValueError("ELINK_TELEMETRY_EMERGENCY_MISSING")
        data = message.data
        # Treat asserted alarm bits or an unexplained nonzero reason as a
        # reported fault; a missing proto3 default is not independent proof of
        # normal hardware status (the acquisition audit remains mandatory).
        fault = bool(data.reason != 0 or any(
            value for descriptor, value in data.ListFields() if descriptor.type == descriptor.TYPE_BOOL))
    return TelemetrySample(int(message.header.seq), source_ns, publisher_node, publisher_gid,
                           variant or "", True, bool(fault), tuple(diagnostics))
