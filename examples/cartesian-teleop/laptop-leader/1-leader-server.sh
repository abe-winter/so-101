#!/usr/bin/env bash
# Terminal 1: serve the SO-101 leader from this Mac with a local, cloud-less viam-server.
# No app.viam.com machine is involved; the client dials it at localhost:8080 with -leader-insecure.
set -euo pipefail
cd "$(dirname "$0")/../.."
make bin/arm
echo "serial ports present:"; ls /dev/cu.usbmodem* || true
echo "leader-local.json ports:"; grep -o "/dev/cu.usbmodem[0-9A-Za-z]*" examples/cartesian-teleop/leader-local.json
exec viam-server -config examples/cartesian-teleop/leader-local.json
