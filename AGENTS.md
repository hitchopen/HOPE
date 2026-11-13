# Repository work rules

## Deployment, robot changeover and alignment claims

Before planning or reporting a deployment, read
`docs/operations/new_robot_commissioning.md` and
`docs/operations/deployment_coverage.md`. Use the versioned inventory in
`foxglove/deployment_profiles/coverage.json` for full-machine/changeover scope.
Keep this inventory and installation/rollback guidance current when adding a
deployable component. An operations bundle manifest or model catalog is not a
whole-machine manifest.

Report source pushed, installed files, loaded runtime, reboot persistence and
physical qualification separately. Match machine ID, boot ID, selected slot,
process identity and actual hashes at the layer claimed. Missing evidence stays
unverified; do not infer it from a historical receipt or matching Git branches.
For full-machine coverage, retain the offline coverage check output and review
the underlying evidence; the tool does not verify the truth of human assertions.

When auditing whether field work was pushed, inspect the deployment task's
staging manifest and final receipt as well as repository status. Account for
field-only helper/profile changes outside the checkout. Preserve their exact
provenance and reconcile behavior against current source before reuse; a
historical patch is not an approved installer. Do not replace a newer helper
wholesale or assume two robots with the same model use the same BSP.

Keep dirty field changes intact and use a suitable isolated worktree for source
reconciliation. Deployment/restart/mocap/robot actions remain subject to the
user's current authorization and the existing hardware procedure; source work
and offline coverage checks do not authorize those actions.
