# Continuous flight fitting and 405 ms packet deadline

The packetizer continuously evaluates the latest clean flight segment and emits
at predicted contact minus 350 ms policy + 30 ms transport + 5 ms solve + 20 ms
policy tick. That is the producer's reserve; it is not 405 ms remaining after
network delivery to Runner. Samples keep camera source timestamps. The final
field guard settings are 16 direction frames, 100 ms maximum saved-fit age and
20 ms maximum gap between fit samples. Contact/epoch boundaries never get fitted
as one smooth free flight; an already committed flight is not rescheduled.

Both NatNet and VRPN expose `/poses/planner_full`, one ball-only message per
received exposure. These profiles consume that raw stream instead of the
visualization stream. Legacy pose topics remain. Install Planner and packetizer
as a matched pair; physical coefficients and schema/HOME defaults are preserved
from main. PPQ004 selection is a separate PR.

The historical October 20 deployment still subscribed to `/poses`; this PR's
full-rate topic wiring is source-tested and is not claimed as that deployment's
live state. Historical replay uses PPQ002 coefficients explicitly and simulates
a source-time 1 ms timer, with no network or policy actuation.

The 27 complete Friday input trajectories and per-flight expected outputs are in
`hope_ws/src/hope_planner_cpp/test/data/deadline_20261020`. The replay retains the
57 ms missing-data case with only about 287 ms lead. Across the 12 trajectories
with measured crossing comparisons, median absolute timing error changed from
11.05 to 13.08 ms; this is not evidence of universal precision or hit-rate gains.
The incorrect-ball October 20 trials are excluded from acceptance.

Build the normal ROS package and run its CTest suite. The standalone replay uses
the built core (replace BUILD with the package build directory):

```bash
g++ -std=c++17 -O2 -Ihope_ws/src/hope_planner_cpp/include -I/usr/include/eigen3 hope_ws/src/hope_planner_cpp/scripts/replay_hit_deadline.cpp BUILD/libhope_planner_core.a -o /tmp/replay_hit_deadline
python3 hope_ws/src/hope_planner_cpp/scripts/check_deadline_replay.py --executable /tmp/replay_hit_deadline
```

ROS integration tests must run on localhost-only isolated DDS domains. No robot
restart or physical qualification is performed by these tests.
