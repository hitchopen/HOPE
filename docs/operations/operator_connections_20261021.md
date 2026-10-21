# Confirmed configuration and robot connections

The Console backfills each confirmed field as it arrives, without replacing an
operator's edited draft. A robot IP opens a new Foxglove connection and locks the
old panel until the matching target reports its identity; it never changes the
old robot's persisted HDU address. Saved connection targets survive panel reload.
Missing operator-profile topics remain compatible with older supervisors.

CONFIRM CONFIG may reconnect mocap while STOPPED. Changing the operator profile,
Laptop address, source, endpoint, units, pants or table side reconnects the
selected receiver. A saved configuration is retained if reconnection fails, and
another confirmation retries it. An existing receiver owned by another robot
is not stopped. Pelvis/world confirmations survive only an exact configuration
match. Mac operator dispatch permits the same explicit reconnect action.

Receiver startup alone is not evidence of streaming: the Console waits for fresh
pose data, and for marker data when using Motive. VRPN has no marker-count feed.
The manual setup command remains available as a fallback.

Validation: 49 Python tests and 23 frontend tests passed; Console build and shell
syntax checks passed. Tests use temporary files and mocks for receiver operations;
this PR preparation did not reconnect mocap, deploy software or move a robot.

This batch depends on the START recovery branch. Ball-model and trajectory data
remain in their separate data PR.
