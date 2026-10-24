# Add single-node V2 physical-return planning in shadow mode

The minimal V2 engine only forecasts flights and its public target contract
requires game-history qualification. Add an opt-in path inside the existing
C++ planner that selects physically supported receiver-bounce-to-plane
contacts with UNKNOWN/INCOMPLETE history, solves the return with the shared
forward engine, reconstructs the proposed paddle action and checks its first
outgoing landing before encoding a shadow schema-2 command.

Legacy remains the default and its numerical implementations/configurations
stay pinned. The standalone forecaster remains command-free. The new profile
uses the real typed mocap observation and registered base streams, with bounded
latest-work scheduling, semantic shot IDs, exact deadlines, physical/source
fences, turn/reach checks and input readiness diagnostics. No referee producer
is required. Ignored context no longer spuriously revokes standalone forecasts.

Validation includes native and sanitizer regressions, 144 independent outgoing
reference cases, actual runner decoding/110-D observation construction, and
installed raw-MCT -> real adapters -> integrated node -> C++ receiver tests at
150/300 Hz with both plane policies. Both available policies load with the
actual runner. See `integrated_planner_acceptance.md` and its JSON for exact
results, pins, inherited lint and requirement-level limits.

This is a **draft, shadow-only implementation**, not the full V3 qualification.
Normal-runner three-arm closed-loop outcomes, the V2+legacy-return ablation,
expanded lifecycle/load profiles, live authority and independent physical
evidence remain outstanding. Live mode is unavailable. No robot activation,
existing PR64 change, push or merge is part of this change. PR64's historical
forecast-only downstream exclusions do not describe this branch's application.

Branch: `feature/integrated-racket-planner-v2`, based on minimal
`4dc0959348f2aa777330d177d16eb70f778a79e6`. A same-fork PR against minimal isolates
this integration delta; a PR against upstream `nightly_built` includes the
minimal engine commits until those land upstream.
