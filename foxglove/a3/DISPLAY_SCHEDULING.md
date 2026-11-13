# Display scheduling

The read-only laptop `hope-live-scene` launcher runs at nice +10. Scene and TF
output are capped at 5 Hz and telemetry is 5 Hz by default. `--scene-hz` (1–20)
and `--telemetry-hz` (1–10) configure those rates. Measurement consumption remains
20 Hz and the remote trace reader retains every fresh 50 Hz sample; the remote
reader also runs at nice +10. Target/frozen-shot updates remain event driven.

The pinned bridge build applies `patches/foxglove_bridge-display-priority.patch`
through `apply_foxglove_display_patch.py`, validating original or patched source
hashes before proceeding. Both bridge parameter files set:

```yaml
display_subscription_nice: 10
display_ball_marker_hz: 10.0
```

The `fg-display` thread has a separate single-threaded executor for ROS-to-browser
subscriptions. `/hope/runner/operator_challenge_hdu_flat` remains on the normal
executor. Browser-to-ROS publishes, service callbacks, DDS receive threads and
WebSocket infrastructure keep their original priority. This does not change the
Runner, localization transport, recording or operator watchdog configuration.
Do not apply `Nice=10` to the entire control bridge: that also lowers heartbeat
and service work. Nice is scheduling preference under contention, not a CPU cap.

The ball-marker setting limits display forwarding only; its upstream observer
still publishes at its configured rate. It saves WebSocket encoding/traffic,
not upstream DDS reception. Scene/telemetry reductions happen at their source.
No raw mocap, planner or control topic is decimated.

After building, activate only the display endpoints while the robot is not in
MOTION, and verify `fg-display` is NI=10 while other bridge threads remain NI=0
with `ps -eLo pid,tid,ni,cls,comm`. Verify message rates at the browser endpoint,
challenge freshness, and the unchanged Runner PID/mode. Never infer physical
hitting improvement or total HDU CPU reduction from message-count savings.

For rollback, restore the backed-up bridge library/executable and parameter
files, then restart the two bridge services. Restore the saved laptop live-scene
files and restart its user service. `display_subscription_nice: 0` disables the
separate executor and the marker forwarding limiter after a bridge restart.
