#!/usr/bin/env bash
# Install the Arista camera udev rules.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Package dir .../src/arista_camera_middleman
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PACKAGE_DIR}/../.." && pwd)"
RULE_SOURCE="${REPO_ROOT}/udev_rules/99-arista-camera.rules"
RULE_TARGET="/etc/udev/rules.d/99-arista-camera.rules"

if [[ ! -f "${RULE_SOURCE}" ]]; then
    echo "❌ Udev rule not found at ${RULE_SOURCE}" >&2
    exit 1
fi

SUDO=""
if [[ $EUID -ne 0 ]]; then
    if ! command -v sudo >/dev/null 2>&1; then
        echo "❌ Script must be run as root or sudo must be available." >&2
        exit 1
    fi
    SUDO="sudo"
fi

echo "Installing udev rules from ${RULE_SOURCE} -> ${RULE_TARGET}"
${SUDO} cp "${RULE_SOURCE}" "${RULE_TARGET}"
${SUDO} chown root:root "${RULE_TARGET}"
${SUDO} chmod 644 "${RULE_TARGET}"

echo "Reloading udev rules..."
${SUDO} udevadm control --reload-rules
${SUDO} udevadm trigger

cat <<EOF
✅ Udev rules installed.

Symlinks provided:
  /dev/thermal_cam
  /dev/rgb_zoom_harshcam

Additional USB webcams may expose /dev/webcam after replugging.
Unplug and replug the cameras (or reboot) if the symlinks do not appear immediately.
EOF
