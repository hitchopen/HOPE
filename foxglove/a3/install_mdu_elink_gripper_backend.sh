#!/usr/bin/env bash
# Install or verify the A3 Ultra Elink ROS 2 gripper command backend on the MDU.
# The installer changes exactly one backend list and preserves a recoverable
# copy of the original vendor file. It does not restart any service.
set -euo pipefail
umask 077
export LC_ALL=C
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

readonly ROBOT_MODEL_FILE=/agibot/data/info/model
readonly SUPPORTED_ROBOT_MODEL=A3_P1D0
readonly ELINK_CONFIG=/opt/agibot/config/hal_elink/hal_elink_a3_ultra_cfg.yaml
readonly BACKUP_ROOT=/var/backups/hope-mdu-elink-gripper
readonly TOPIC_LINE='      - topic_name: "/body_drive/hand_joint_command"'
readonly SOURCE_BACKEND_LINE='        enable_backends: [iceoryx]'
readonly TARGET_BACKEND_LINE='        enable_backends: [iceoryx, ros2]'

die() {
    echo "ERROR: $*" >&2
    exit 1
}

require_supported_target() {
    [[ -f "${ROBOT_MODEL_FILE}" && ! -L "${ROBOT_MODEL_FILE}" ]] || \
        die "robot model file is missing or is a symlink: ${ROBOT_MODEL_FILE}"
    [[ "$(<"${ROBOT_MODEL_FILE}")" == "${SUPPORTED_ROBOT_MODEL}" ]] || \
        die "unsupported robot model: $(<"${ROBOT_MODEL_FILE}")"
    [[ -f "${ELINK_CONFIG}" && ! -L "${ELINK_CONFIG}" ]] || \
        die "Ultra Elink config is missing or is a symlink: ${ELINK_CONFIG}"
    [[ "$(grep -Fc -- 'hand_enable: true' "${ELINK_CONFIG}")" == 1 ]] || \
        die "Ultra Elink config does not contain exactly one enabled hand section."
}

backend_state() {
    awk -v topic="${TOPIC_LINE}" \
        -v source_backend="${SOURCE_BACKEND_LINE}" \
        -v target_backend="${TARGET_BACKEND_LINE}" '
        $0 == topic {
            topics += 1
            if ((getline backend) <= 0) exit 40
            if (backend == source_backend) source += 1
            else if (backend == target_backend) target += 1
            else unexpected += 1
        }
        END {
            if (topics != 1 || source + target != 1 || unexpected != 0) exit 41
            if (source == 1) print "SOURCE"
            else print "TARGET"
        }
    ' "${ELINK_CONFIG}"
}

verify_target() {
    require_supported_target
    [[ "$(backend_state)" == TARGET ]] || \
        die "Ultra Elink gripper topic is not enabled for both iceoryx and ros2."
    printf 'MDU_ELINK_GRIPPER_BACKEND_OK path=%s sha256=%s\n' \
        "${ELINK_CONFIG}" "$(sha256sum "${ELINK_CONFIG}" | awk '{print $1}')"
}

install_target() {
    [[ "${EUID}" == 0 ]] || die "--install must run as root."
    require_supported_target

    local state source_sha stamp backup temporary
    state="$(backend_state)" || die "unexpected Ultra Elink gripper topic structure."
    if [[ "${state}" == TARGET ]]; then
        verify_target
        echo "MDU_ELINK_GRIPPER_BACKEND_ALREADY_INSTALLED"
        return
    fi

    source_sha="$(sha256sum "${ELINK_CONFIG}" | awk '{print $1}')"
    stamp="$(date -u +%Y%m%dT%H%M%SZ)"
    backup="${BACKUP_ROOT}/hal_elink_a3_ultra_cfg.yaml.${stamp}.${source_sha}.orig"
    install -d -o root -g root -m 0700 "${BACKUP_ROOT}"
    [[ ! -e "${backup}" ]] || die "backup already exists: ${backup}"
    cp -a -- "${ELINK_CONFIG}" "${backup}"

    temporary="$(mktemp "${ELINK_CONFIG}.hope-incoming.XXXXXX")"
    trap '[[ -z "${temporary:-}" ]] || unlink -- "${temporary}" 2>/dev/null || true' EXIT
    awk -v topic="${TOPIC_LINE}" \
        -v source_backend="${SOURCE_BACKEND_LINE}" \
        -v target_backend="${TARGET_BACKEND_LINE}" '
        $0 == topic {in_topic = 1; topics += 1; print; next}
        in_topic && $0 == source_backend {
            print target_backend
            changed += 1
            in_topic = 0
            next
        }
        {print}
        END {if (topics != 1 || changed != 1) exit 42}
    ' "${ELINK_CONFIG}" > "${temporary}" || \
        die "failed to render the minimal Ultra Elink backend change."

    chown --reference="${ELINK_CONFIG}" "${temporary}"
    chmod --reference="${ELINK_CONFIG}" "${temporary}"
    mv -f -- "${temporary}" "${ELINK_CONFIG}"
    temporary=""

    state="$(backend_state 2>/dev/null || true)"
    if [[ "${state}" != TARGET ]] || \
        [[ "$(grep -Fc -- 'hand_enable: true' "${ELINK_CONFIG}" || true)" != 1 ]]; then
        cp --preserve=all -- "${backup}" "${ELINK_CONFIG}"
        die "installed config verification failed; original config restored from ${backup}"
    fi
    verify_target
    printf 'MDU_ELINK_GRIPPER_BACKEND_INSTALLED_OK backup=%s source_sha256=%s\n' \
        "${backup}" "${source_sha}"
}

case "${1:-}" in
    --install)
        [[ "$#" == 1 ]] || die "--install accepts no additional arguments."
        install_target
        ;;
    --verify-only)
        [[ "$#" == 1 ]] || die "--verify-only accepts no additional arguments."
        verify_target
        ;;
    *)
        die "usage: $0 --install | --verify-only"
        ;;
esac
