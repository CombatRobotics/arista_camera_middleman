#!/usr/bin/env bash
# Install the udev rule for the Harshcam serial control interface.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULE_SOURCE="${PACKAGE_DIR}/udev/99-harshcam-control.rules"
RULE_TARGET="/etc/udev/rules.d/99-harshcam-control.rules"

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

echo "Installing Harshcam control udev rule -> ${RULE_TARGET}"
${SUDO} cp "${RULE_SOURCE}" "${RULE_TARGET}"
${SUDO} chown root:root "${RULE_TARGET}"
${SUDO} chmod 644 "${RULE_TARGET}"

echo "Reloading udev rules..."
${SUDO} udevadm control --reload-rules
${SUDO} udevadm trigger

cat <<EOF
✅ Harshcam control udev rule installed.

Symlink provided once the device reconnects:
  /dev/harshcam_control

Unplug and replug the Harshcam USB3 NeoHD interface (or reboot) if the symlink does not appear.
EOF
