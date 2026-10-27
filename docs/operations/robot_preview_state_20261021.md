# Robot-scoped Y-range preview drafts

The local Y-range preview saves the authoritative robot address with its draft.
A draft is restored only after the same robot reports that address. Changing
robots remounts the range control, avoiding reuse of another robot's local draft.
Saving a preview preserves other saved panel fields. This only changes local
preview state and never applies a Planner parameter or publishes a robot command.

This is a narrow follow-up to the main integration of optional Y filtering on
`codex/hope-y-filter-main-20261022`. It shares the existing `savedPanel` reference
and `configHost` binding with operator connections.

Validation: Console build and all 38 frontend tests passed against the current
main integration. No deployment or hardware operation was performed.
