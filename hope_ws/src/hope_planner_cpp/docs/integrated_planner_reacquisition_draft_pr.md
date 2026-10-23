# Recover rejected flights and qualify causal spin in integrated V2

A rejected flight permanently latched automatic acquisition, repeated angular
qualification discarded the history needed for quaternion-only spin, and the
integrated base consumer decoded the producer's WXYZ quaternion in the wrong
order. This continuation introduces evidence-fenced rearm with fresh ownership,
bounded causal candidate spin, truthful readiness diagnostics, correct rotated
base admission and preservation of explicit valid zero through ROS decoding.
Legacy behavior, physics coefficients, policy artifacts and wire schemas remain
pinned; output stays shadow-only.

Validation: 18 native and 18 ASan/UBSan groups, 26 historical review cases,
RA01–RA08/SP01–SP09/BP01–BP05, 701 exact legacy comparisons, 144 explicitly
DOP853 outgoing cases, two policy artifacts and real producer/decoder checks.
All 140 initial A/B pairs recover with decoded B commands. Full ROS initially
passed 96/100 planner groups; two instrumentation issues were corrected and
passed targeted retests. Two 300 Hz coverage cases failed at 92.5% and 91.25%;
each exact retry passed 100%, but timing qualification remains open. Bringup's
two lint failures exactly match baseline. G6/G7 remain NOT_RUN.

See `integrated_planner_reacquisition_spin_qualification.md` and its JSON for
source pins, denominators, failed runs and remaining work. This is a local draft,
not an upstream submission or deployment request. PR64 is closed and unmerged.
