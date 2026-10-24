# Failed START recovery and bounded parameter discovery

A new START request in FAILED first cleans and collects the exact previous
managed session, then starts once under the same operation lock. Cleanup failure
retains the old identity and lock; a second click cannot launch another worker.
The Console labels this path RETRY START and shows CLEANING while recovery runs.
Preflight errors retain the last diagnostic line instead of truncating away it.

Ball-parameter reads keep one DDS participant for up to four discovery/reply
attempts (20 seconds maximum). Pending requests are cancelled on timeout;
duplicate nodes still fail validation. The Laptop unicast discovery peer list
includes loopback, preserving access by the local parameter checker.

Validation: 28 Python tests passed, including a real ROS parameter service that
appears after the old two-second window and a bounded-timeout test, on localhost
DDS domain 218. Console build and all 18 frontend tests passed. This PR only
changes source; historical field repairs do not imply a new deployment here.

Depends on the ball-model software PR. It does not contain trajectory archives,
change ball coefficients, or change automatic recording defaults.
