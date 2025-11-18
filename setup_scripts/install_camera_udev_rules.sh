#!/usr/bin/env bash
# Install the Arista camera payload video device udev rules.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULE_SOURCE="${PACKAGE_DIR}/udev/99-arista-camera.rules"
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

echo "Installing Arista camera rules -> ${RULE_TARGET}"
${SUDO} cp "${RULE_SOURCE}" "${RULE_TARGET}"
${SUDO} chown root:root "${RULE_TARGET}"
${SUDO} chmod 644 "${RULE_TARGET}"

echo "Reloading udev rules..."
${SUDO} udevadm control --reload-rules
${SUDO} udevadm trigger

cat <<EOF
✅ Camera udev rules installed.

Symlinks provided (after replug):
  /dev/thermal_cam
  /dev/rgb_zoom_harshcam
  /dev/webcam (USB2.0 FHD UVC)

Unplug and replug the cameras (or reboot) if the symlinks do not appear immediately.
EOF
