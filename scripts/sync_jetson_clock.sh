#!/usr/bin/env bash
# Sync a USB-C-connected Jetson to this laptop's clock.

set -euo pipefail

jetson_host="${1:-jetson-usbc}"
laptop_utc="$(date -u '+%Y-%m-%d %H:%M:%S')"

echo "Laptop UTC: ${laptop_utc}"
echo "Syncing ${jetson_host}; enter the Jetson sudo password when prompted."

ssh -tt "${jetson_host}" \
  "sudo bash -c 'set -e
timedatectl set-ntp false
date -u -s \"${laptop_utc}\"
for rtc in /dev/rtc0 /dev/rtc1; do
  if [ -e \"\${rtc}\" ]; then
    hwclock --systohc --utc --rtc=\"\${rtc}\"
  fi
done
touch /var/lib/systemd/timesync/clock
timedatectl set-ntp true
timedatectl status'"

echo "Clock sync complete."
