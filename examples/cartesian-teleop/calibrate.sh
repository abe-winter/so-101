#!/usr/bin/env bash
# Calibrate one SO-101 leader on the cart from this laptop (or use the so101-setup app).
#   ./calibrate.sh right
set -euo pipefail
side="${1:?usage: $0 left|right}"
case "$side" in left|right) ;; *) echo "side must be left or right" >&2; exit 2;; esac
cd "$(dirname "$0")/../.."
: "${VIAM_API_KEY_ID:?set VIAM_API_KEY_ID}" "${VIAM_API_KEY:?set VIAM_API_KEY}"
MACHINE_ADDR="${MACHINE_ADDR:-arm-teleop-demo-main.kssbd6djf3.viam.cloud}"
exec go run ./cmd/so101-calibrate -addr "$MACHINE_ADDR" -sensor "$side-leader-calibration"
