# Time-calibration coordinator network recovery

The time-calibration coordinator waits up to 30 seconds for the saved HDU IPv4
address to be present on a link with carrier before initializing DDS. A missing
venue network does not block local diagnostics indefinitely. Install the matching
`hope_time_calibration.py` and `hope_time_calibration_core.py` together, including
the `/usr/local/bin` entrypoint used by the service.

The coordinator samples interface identity, address and carrier once per second.
It remembers observed down/up transitions even when the address is unchanged.
After a changed signature has settled for three seconds, it rebuilds DDS only
when all of these hold:

- Calibration is IDLE, COMPLETE or REJECTED, with no owned maintenance lock.
- The local lifecycle owner reports STOPPED and not busy, in this boot, with a
  monotonic timestamp no more than 1.5 seconds old.
- The shared hardware-operation lock can be acquired, and owner evidence still
  passes after acquisition.

Request admission is sealed before teardown. The coordinator exits with code 75;
`hope-time-calibration.service` already has `Restart=on-failure` and `RestartSec=2`.
No calibration request is replayed, no clock is stepped, and no robot process is
started by network recovery. RUNNING, INTERRUPTED and FAILED_SAFE_STOP retain
their existing behavior. Changes observed while busy remain pending until safe.

Verification: the coordinator log contains `TIME_CALIBRATION_NETWORK_WAIT` at
startup and `NETWORK_DDS_RECYCLE` on recovery. Check fresh
`/hope/lifecycle/time_calibration/state` and `busy` at both the HDU and Laptop
Foxglove endpoints, plus fresh APPLIED configuration and STOPPED lifecycle.
A clickable button proves admission readiness, not completed clock calibration.

Regression coverage includes delayed address arrival, bounded offline startup,
same-IP link recovery, debounce, unavailable probes, stale owner evidence,
maintenance interlocks and request/recycle races. The isolated ROS test requires
Jazzy with ROS_DOMAIN_ID=218 and ROS_LOCALHOST_ONLY=1; it exercises the real timer,
executor teardown, exit code and replacement process using synthetic network
inputs. It does not disconnect a robot or qualify physical cold-boot recovery.
