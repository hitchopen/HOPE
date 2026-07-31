# Validated PR #18 reference assets

This directory preserves the exact fixed-motion reference associated with
PR #18. The project developer confirms that the runtime and
`serve_policy.csv` were fully tested on Agibot A3 and are executable and safe
for that application.

| Asset | Identity and purpose |
|---|---|
| `serve_policy.csv` | Exact 3,878-frame, 200 Hz SDK source CSV; SHA-256 `2a7de3f1c97a300069899c139c9eb96e94fd61d3419701d5e44ef37b2bf6641d`. |
| `serve_vendor_arm.json` | Runtime, timing, transport, limit and evidence contract for the CSV. |
| `pr18_a3_serve_demo.mp4` | Developer-supplied PR #18 demo reference; H.264/AAC, 720×788, 30 fps, 6.525 s; SHA-256 `5e476beb7671491d2288fd9dcd5206a12f41717d2a65253313ce1f5505d36ac8`. |

The video is retained as a visual reference source. The manifest-bound CSV and
runtime remain the machine-verifiable execution artifacts. A newly generated
CSV is a different artifact and must complete its own A3 qualification before
real execution.
