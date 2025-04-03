#!/usr/bin/bash
python px_uploader.py --port "/dev/serial/by-id/*_PX4_*,/dev/serial/by-id/usb-3D_Robotics*,/dev/serial/by-id/usb-The_Autopilot*,/dev/serial/by-id/usb-Bitcraze*,/dev/serial/by-id/pci-3D_Robotics*,/dev/serial/by-id/pci-Bitcraze*,/dev/serial/by-id/usb-Gumstix*" crazyflie_default.px4

