#!/bin/bash

# PTP clock workers are owned and supervised by systemd. This hook remains
# because the vendor process-manager launcher calls it during every startup.

set -u

for service in agibot-mdu-ptp4l.service agibot-mdu-phc2sys.service; do
    if ! systemctl is-active --quiet "$service"; then
        printf 'WARNING: %s is not active; systemd will continue recovery attempts.\n' "$service" >&2
    fi
done

exit 0
