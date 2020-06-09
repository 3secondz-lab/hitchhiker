#!/bin/bash
echo "Waiting for 3 seconds before running usbip..."
sleep 3s
echo "Try to execute usbip"
/mnt/c/usbip/usbip.exe -a localhost "1-1"
