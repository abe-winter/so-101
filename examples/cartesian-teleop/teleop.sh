#!/usr/bin/env bash
# Run the Cartesian mirror from this laptop against the arm-teleop-demo cart (SO-101 leaders
# and xArms both live there). One process, one connection, both sides by default.
#   ./teleop.sh                 # left + right
#   ./teleop.sh right           # one side
#   ./teleop.sh -align-only     # only meaningful with "align_on_start": true in the json
# Needs VIAM_API_KEY_ID / VIAM_API_KEY for the machine (org viam-dev).
set -euo pipefail
cd "$(dirname "$0")/../.."
: "${VIAM_API_KEY_ID:?set VIAM_API_KEY_ID}" "${VIAM_API_KEY:?set VIAM_API_KEY}"
MACHINE_ADDR="${MACHINE_ADDR:-arm-teleop-demo-main.kssbd6djf3.viam.cloud}"
configs=()
rest=()
for a in "$@"; do
  case "$a" in
    left|right) configs+=(-config "examples/cartesian-teleop/teleop-$a.json") ;;
    *) rest+=("$a") ;;
  esac
done
if [ ${#configs[@]} -eq 0 ]; then
  configs=(-config examples/cartesian-teleop/teleop-left.json -config examples/cartesian-teleop/teleop-right.json)
fi
# macOS ships bash 3, where an empty array is "unbound" under set -u.
exec go run ./cmd/teleop-client "${configs[@]}" -leader-addr "$MACHINE_ADDR" -status-every 2s ${rest[@]+"${rest[@]}"}
