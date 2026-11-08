"""Visible, bounded diagnostics for contained telemetry callback failures."""
import threading

from std_msgs.msg import String, UInt64


class TelemetryFaults:
    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._count = 0
        self._active = {}
        self._last_error = "NONE"
        self._count_pub = node.create_publisher(UInt64, "/hope/operations/telemetry_fault_count", 10)
        self._status_pub = node.create_publisher(String, "/hope/operations/telemetry_fault", 10)
        self._timer = node.create_timer(1.0, self.publish)

    def failed(self, source, exc):
        detail = f"{source}: {type(exc).__name__}: {exc}"
        with self._lock:
            self._count += 1
            self._active[source] = detail
            self._last_error = detail
        # Reporting a broken telemetry publisher must not become a second
        # exception that terminates lifecycle ownership.
        try:
            self._node.get_logger().error(detail, throttle_duration_sec=5.0)
        except Exception:
            pass
        self.publish()

    def recovered(self, source):
        with self._lock:
            changed = self._active.pop(source, None) is not None
            changed = (self._active.pop(source + ".invalidate", None) is not None) or changed
        if changed:
            self.publish()

    def publish(self):
        with self._lock:
            status = "FAULT" if self._active else "OK"
            text = (f"{status} count={self._count} active={','.join(sorted(self._active)) or 'NONE'} "
                    f"last={self._last_error}")
            # Serialize these small diagnostic writes across callback groups.
            try:
                self._count_pub.publish(UInt64(data=self._count))
                self._status_pub.publish(String(data=text))
            except Exception:
                pass
