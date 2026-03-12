#!/usr/bin/env bash
#
# Bandwidth calibration sweep for telemoq DROID replay demo.
# Tests multiple bandwidth limits and finds the sweet spot where
# control tracks > 95% delivery and video tracks < 50%.
#
# Usage:
#   ./scripts/calibrate.sh
#
# Output: prints recommended bandwidth and saves to scripts/.calibrated_bw

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RELAY_DIR="$PROJECT_DIR/../lumina-video/moq"
REPLAY_PATH="${REPLAY_PATH:-$PROJECT_DIR/sample_data}"
BROADCAST="calibrate"
RELAY_URL="https://localhost:4443"
DURATION=10  # seconds per test

BANDWIDTHS=(1000 750 500 300 150 100)

if [ ! -d "$REPLAY_PATH" ]; then
  echo "Error: Replay data not found at $REPLAY_PATH"
  echo "Run: python scripts/convert_droid.py --output sample_data --episodes 3"
  exit 1
fi

echo "Building telemoq..."
(cd "$PROJECT_DIR" && cargo build --quiet 2>/dev/null)

TELEMOQ="cargo run --quiet --manifest-path $PROJECT_DIR/Cargo.toml --"

# Start relay
echo "Starting relay..."
(cd "$RELAY_DIR" && cargo run --quiet --bin moq-relay -- --listen '[::]:4443' --tls-generate localhost --auth-public '') &
RELAY_PID=$!
sleep 3

cleanup() {
  kill "$RELAY_PID" 2>/dev/null || true
  "$SCRIPT_DIR/netem.sh" stop 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo ""
echo "Bandwidth Calibration Sweep (${DURATION}s per test)"
echo "================================================"
printf "%-10s %-15s %-15s %-10s\n" "BW(kbit)" "Control(recv/s)" "Video(recv/s)" "Verdict"
echo "------------------------------------------------"

BEST_BW=""

for bw in "${BANDWIDTHS[@]}"; do
  # Apply impairment
  "$SCRIPT_DIR/netem.sh" start "$bw" >/dev/null 2>&1

  # Start publisher
  $TELEMOQ --url "$RELAY_URL" --tls-disable-verify --broadcast "$BROADCAST" --replay "$REPLAY_PATH" --all-cameras --loop-replay publish &
  PUB_PID=$!
  sleep 1

  # Run subscriber with CSV for DURATION seconds
  CSV_OUT=$(timeout "$DURATION" $TELEMOQ --url "$RELAY_URL" --tls-disable-verify --broadcast "$BROADCAST" --replay --all-cameras --csv subscribe 2>/dev/null || true)

  kill "$PUB_PID" 2>/dev/null || true
  wait "$PUB_PID" 2>/dev/null || true

  # Remove impairment
  "$SCRIPT_DIR/netem.sh" stop >/dev/null 2>&1

  # Parse CSV: average control/streaming recv_per_s and video/camera0 recv_per_s
  CTRL_AVG=$(echo "$CSV_OUT" | grep "control/streaming" | awk -F, '{sum+=$4; n++} END {if(n>0) printf "%.1f", sum/n; else print "0"}')
  VID_AVG=$(echo "$CSV_OUT" | grep "video/camera0" | awk -F, '{sum+=$4; n++} END {if(n>0) printf "%.1f", sum/n; else print "0"}')

  CTRL_PCT=$(echo "$CTRL_AVG 15" | awk '{printf "%.0f", ($1/$2)*100}')
  VID_PCT=$(echo "$VID_AVG 15" | awk '{printf "%.0f", ($1/$2)*100}')

  if [ "$CTRL_PCT" -ge 95 ] && [ "$VID_PCT" -le 50 ]; then
    VERDICT="OPTIMAL"
    [ -z "$BEST_BW" ] && BEST_BW="$bw"
  elif [ "$CTRL_PCT" -ge 80 ]; then
    VERDICT="usable"
  else
    VERDICT="too low"
  fi

  printf "%-10s %-15s %-15s %-10s\n" "${bw}" "${CTRL_AVG}/s (${CTRL_PCT}%)" "${VID_AVG}/s (${VID_PCT}%)" "$VERDICT"

  sleep 1
done

echo ""
if [ -n "$BEST_BW" ]; then
  echo "Recommended bandwidth: ${BEST_BW} kbit/s"
  echo "$BEST_BW" > "$SCRIPT_DIR/.calibrated_bw"
  echo "Saved to $SCRIPT_DIR/.calibrated_bw"
else
  echo "No optimal bandwidth found. Try adjusting the range."
fi
