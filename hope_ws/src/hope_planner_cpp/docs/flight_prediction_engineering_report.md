# V2 minimal-branch engineering report

This report replaces source-scaffold receipts as a description of the new minimal branch. Historical reports/PDFs for 9118a491 or 2bb6cc56 are not current implementation evidence. The uploaded implementation specification supplies the acceptance contract, not passing results.

Architecture, physical assumptions, compatibility policy, complete file inventory and test evidence are linked from the adjacent implementation, physics, scope, parity and acceptance documents. All 50 A/B/C/T/X identifiers remain visible. Mixed requirements retain separate forecast/software, downstream and physical-data dispositions.

Scope decisions: isolate FlightSpinEstimator; retain the small physical history/config-hash helper; keep the original production estimator/predictor/receiver/packetizer/Python code; use the original one-shot trigger; mark unsupported historical events rather than substituting V2 output; reject unmatched moving-plane datasets. No physical coefficients were tuned.

Report refresh checklist:

- [x] Refresh upstream/source pins and inspect both repository histories.
- [x] Preserve source branches and original production bytes.
- [x] Selectively port physical contracts; exclude PR49 command/selector graph.
- [ ] Attach final acceptance/config/code identities and exact current test receipts.
- [ ] Report selected 150/300 Hz and optional-profile load results individually.
- [ ] Publish a PDF only from this updated Markdown when document tooling is available.
- [ ] Collect independent real-flight data with clock/frame/registered-plane receipts before physical qualification.

Do not forward earlier PDFs as if they describe this new branch. No original review PDF was supplied in this task, so exact PDF page citations are unavailable; the complete appendix requirement text was supplied in the implementation plan.
