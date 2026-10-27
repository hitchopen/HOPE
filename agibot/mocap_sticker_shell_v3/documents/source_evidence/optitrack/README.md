# Original measurement evidence — not player imports

`A3.motive` and `A3.csv` are byte-for-byte copies of the original user exports.
They contain the historical 16-point measured body, **not the current P1/P2
12-marker patterns**. Do not import them for the current installation.

`asset_correspondence_20261011.json` and `station_mapping_20261011.csv` document
the earlier correspondence/17-point reconstruction. Only paths and the report's
historical-status label changed during release cleanup; the measurements and
fit results are unchanged. JSON paths are relative to the repository root.
The old generated 17-point files now live only in maintained regression
fixtures under `hope_ws/src/hope_bringup/test/fixtures/motive_legacy_17/`.

Current imports are [P1.motive](../../../motive_asset/P1.motive) and
[P2.motive](../../../motive_asset/P2.motive); their
[manifest](../../../motive_asset/asset_manifest.json) supplies all authoring
coordinates. Original measurements are provenance, not disposable test output
and not proof of current physical pelvis alignment or visibility.
