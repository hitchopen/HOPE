# Recording and display CPU controls

Runner skips trace and observation CSV row formatting while recording is OFF.
Recording ON resumes the existing segment writer; switching OFF clears the
previous-command comparison so the next take cannot compare against an old row.
Capture metadata includes selected ball physics and Planner diagnostics.

Public startup remains manual. Set `HOPE_AUTO_RECORD_ON_START=1` explicitly to
start capture after authoritative Runner readiness. A supervisor restart or a
failed START never initiates capture. Recording errors do not change control mode.

Display-only publishers suppress automatic QoS warning callbacks by default.
`HOPE_DISPLAY_QOS_EVENTS=1` restores them. Topic types, QoS policies, rates,
subscriptions and command publishers are unchanged. Any later deployment must
install `foxglove/a3/hope_display_publishers.py` alongside monitor and observer.

Offline tests cover recording segments, OFF flooding, formatting gates, capture
failure, manual defaults and mocked display publisher construction. They do not
measure CPU savings on a physical robot.
