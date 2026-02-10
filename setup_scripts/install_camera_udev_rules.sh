#!/bin/bash

# Setup udev rules for Arista camera payload (PORT-INDEPENDENT)
# 1. RGB Zoom HARSHCAM (USB3 NeoHD) - video -> /dev/rgb_zoom_harshcam, serial -> /dev/harshcam_zoom_control
# 2. Thermal Camera (TRMR DigitalCore) - video -> /dev/thermal_cam_digitalcore

RULES_FILE="/etc/udev/rules.d/99-arista-camera.rules"
HARSHCAM_RULES_FILE="/etc/udev/rules.d/99-harshcam-control.rules"
RULES=""
HARSHCAM_RULES=""

echo "=== Arista Camera Payload udev rules setup ==="
echo ""

read -p "Setup RGB zoom harshcam? (/dev/rgb_zoom_harshcam & /dev/harshcam_zoom_control) [y/N]: " ans
if [[ "$ans" =~ ^[Yy]$ ]]; then
    echo ""
    echo "Detected USB3 NeoHD serial numbers:"
    lsusb -v -d 04b4:00f9 2>/dev/null | grep -E "iSerial|iProduct" | sed 's/^/  /'
    echo ""
    read -p "Enter USB3 NeoHD serial number: " serial
    if [ -z "$serial" ]; then
        echo "  Serial number cannot be empty. Skipping."
    else
        RULES+='# RGB Zoom HARSHCAM (USB3 NeoHD)\n'
        RULES+='SUBSYSTEM=="video4linux", KERNEL=="video*", ATTR{index}=="0", \\\n'
        RULES+="  ATTRS{idVendor}==\"04b4\", ATTRS{idProduct}==\"00f9\", \\\\\n"
        RULES+="  ATTRS{product}==\"USB3 NeoHD\", ATTRS{serial}==\"${serial}\", \\\\\n"
        RULES+='  SYMLINK+="rgb_zoom_harshcam", MODE="0660", GROUP="video"\n\n'

        HARSHCAM_RULES+='SUBSYSTEM=="tty", KERNEL=="ttyACM*", \\\n'
        HARSHCAM_RULES+="  ATTRS{idVendor}==\"04b4\", ATTRS{idProduct}==\"00f9\", \\\\\n"
        HARSHCAM_RULES+="  ATTRS{serial}==\"${serial}\", \\\\\n"
        HARSHCAM_RULES+='  SYMLINK+="harshcam_zoom_control", MODE="0666", GROUP="dialout"\n'

        echo "  -> Added rgb_zoom_harshcam and harshcam_zoom_control rules"
    fi
fi

read -p "Setup thermal camera digitalcore? (/dev/thermal_cam_digitalcore) [y/N]: " ans
if [[ "$ans" =~ ^[Yy]$ ]]; then
    RULES+='\n# Thermal Camera (TRMR - DigitalCore)\n'
    RULES+='SUBSYSTEM=="video4linux", KERNEL=="video*", ATTR{index}=="0", \\\n'
    RULES+='  ATTRS{idVendor}=="04b4", ATTRS{idProduct}=="00f9", \\\n'
    RULES+='  ATTRS{product}=="TRMR#02SY00014A00001", \\\n'
    RULES+='  SYMLINK+="thermal_cam_digitalcore", MODE="0660", GROUP="video"\n'
    echo "  -> Added thermal_cam_digitalcore rule"
fi

if [ -z "$RULES" ] && [ -z "$HARSHCAM_RULES" ]; then
    echo "No rules selected. Exiting."
    exit 0
fi

echo ""

if [ -n "$RULES" ]; then
    echo "Writing camera rules to $RULES_FILE..."
    echo -e "$RULES" | sudo tee "$RULES_FILE" > /dev/null
fi

if [ -n "$HARSHCAM_RULES" ]; then
    echo "Writing harshcam control rules to $HARSHCAM_RULES_FILE..."
    echo -e "$HARSHCAM_RULES" | sudo tee "$HARSHCAM_RULES_FILE" > /dev/null
fi

sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "Done. Verify with:"
echo "  ls -l /dev/rgb_zoom_harshcam"
echo "  ls -l /dev/harshcam_zoom_control"
echo "  ls -l /dev/thermal_cam_digitalcore"
