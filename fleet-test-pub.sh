#!/usr/bin/env bash
# Fleet stress test — Lemur side (N publishers)
# Usage: ./fleet-test-pub.sh [NUM_ROBOTS] [RELAY_URL] [--replay <path>] [--all-cameras]
#
# Starts N publisher processes, each broadcasting as robot-{N}.
# Publishers start in batches of 5 with 3s gaps to avoid connection storm.
# Kill all with: pkill -f telemoq
#
# Examples:
#   ./fleet-test-pub.sh 10                          # 10 robots, synthetic data
#   ./fleet-test-pub.sh 10 https://... --replay sample_data --all-cameras
#                                                    # 10 robots, real DROID data, 3 cameras

set -euo pipefail

NUM_ROBOTS="${1:-3}"
RELAY_URL="${2:-https://192.168.8.213:4443}"
shift 2 2>/dev/null || true
EXTRA_ARGS="$*"

BATCH_SIZE=5
BATCH_DELAY=3
PUB_BIN="$HOME/Desktop/develop/telemoq/target/release/telemoq"
LOG_DIR="fleet-pub-log"

mkdir -p "$LOG_DIR"

# Clear tc netem if any
sudo tc qdisc del dev wlp0s20f3 root 2>/dev/null || true

for i in $(seq 1 "$NUM_ROBOTS"); do
  BROADCAST="robot-$i"
  LOG_FILE="$LOG_DIR/$BROADCAST.log"
  echo ">>> Starting publisher: $BROADCAST → $LOG_FILE"
  $PUB_BIN \
    --url "$RELAY_URL" \
    --tls-disable-verify \
    --broadcast "$BROADCAST" \
    --log-bandwidth \
    $EXTRA_ARGS \
    publish > "$LOG_FILE" 2>&1 &
  echo "    publisher PID=$!"
  sleep 0.5

  # Pause between batches
  if (( i % BATCH_SIZE == 0 && i < NUM_ROBOTS )); then
    echo "--- Batch of $BATCH_SIZE started. Waiting ${BATCH_DELAY}s before next batch..."
    sleep "$BATCH_DELAY"
  fi
done

echo ""
echo "=== Fleet test: $NUM_ROBOTS publishers running ==="
echo "=== Logs in $LOG_DIR/ ==="
echo "=== Stop with: pkill -f telemoq ==="
