#!/usr/bin/env bash
# One-time per SO-101: calibrate it through the local viam-server (terminal 1 must be running).
#   ./0-calibrate.sh right
# Walks start -> set_homing -> range recording -> save_calibration with prompts. Afterwards
# restart terminal 1 so the arm component reloads <side>_leader_calibration.json.
set -euo pipefail
side="${1:?usage: $0 left|right}"
case "$side" in left|right) ;; *) echo "side must be left or right" >&2; exit 2;; esac
cd "$(dirname "$0")/../.."
exec go run ./cmd/so101-calibrate -addr localhost:8080 -insecure -sensor "$side-calibration"
