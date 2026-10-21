# Robot-scoped Y-range preview drafts

The local Y-range preview saves the authoritative robot address with its draft.
A draft is restored only after the same robot reports that address. Changing
robots remounts the range control, avoiding reuse of another robot's local draft.
Saving a preview preserves other saved panel fields. This only changes local
preview state and never applies a Planner parameter or publishes a robot command.

This is a narrow follow-up to the existing field/site options branch containing
BallYFilterControls. Keep that feature dependency when integrating into main.
When combining with operator-connections, use its single savedPanel reference
and configHost binding rather than declaring duplicates.

Validation: Console build and all 21 frontend tests passed on the nohope site
options parent. No deployment or hardware operation was performed.
