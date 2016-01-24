#!/bin/bash

# This script is a work-around for the lack of a mechanism to tell hid-generic
# and usbhid to not bind to the mcp2210 device.

vid="04d8"
pid="00de"

die() {
	echo "${BAD}ERROR${NORMAL}$(test $# -eq 0 || echo ": $*")" >&2
	exit 1
}

# Find sysfs mount point (may not be /sys)
sysfs_mount=$(
	grep '^sysfs ' < /proc/mounts |
	perl -pe 's:sysfs (/\w+) .*:$1:g' |
	tail -1
)

test -n "${sysfs_mount}" || die "OMG! no sysfs mount point! O_o"
pushd "${sysfs_mount}/bus/usb/devices" || die

# Find all devices
mcp2210_devices=$(
	for f in *; do
		if grep -i ${vid} $f/idVendor  > /dev/null 2>&1 &&
		   grep -i ${pid} $f/idProduct > /dev/null 2>&1; then
			echo $f;
		fi;
	done
)

if [[ -z "${mcp2210_devices}" ]]; then
	die "No devices found.  If you have altered the vid/pid, please" \
	    "update this script with the new values."
fi

for dev in ${mcp2210_devices}; do
	echo "${dev}:1.0" > "${sysfs_mount}/bus/usb/drivers/usbhid/unbind"
done

# This only needs to be done if the mcp2210 driver has already been probed.
# Otherwise, it will properly bind to the devices on its own.
if lsmod | grep '^mcp2210 ' > /dev/null; then
	for dev in ${mcp2210_devices}; do
		echo "${dev}:1.0" > "${sysfs_mount}/bus/usb/drivers/mcp2210/bind"
	done
fi
