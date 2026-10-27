# Historical 17-marker regression fixtures — not deployment assets

These files preserve tests of measured-point correspondence, frame conversion,
variable-count live ModelDef acceptance and rejection of unapproved calibration
references. They are byte-identical to their former release locations and are
not installed by CMake.

Do not import these into the current two-player Motive project. The current
P1/P2 release has 12 markers each in
`agibot/mocap_sticker_shell_v3/motive_asset/P1.motive` and `P2.motive`.
Original user exports remain under the shell's `documents/source_evidence/optitrack/`.
