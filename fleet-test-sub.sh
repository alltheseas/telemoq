#!/usr/bin/env bash
# Fleet stress test — Mac Mini side (relay + N subscribers)
# Usage: ./fleet-test-sub.sh [NUM_ROBOTS] [--no-relay]
#
# Starts moq-relay (unless --no-relay) and N subscriber processes.
# Subscribers start in batches of 5 with 3s gaps to avoid slamming the relay.
# CSV output per robot: fleet-csv/robot-{N}.csv
# Kill all with: pkill -f telemoq; pkill -f moq-relay

set -euo pipefail

NUM_ROBOTS="${1:-3}"
BATCH_SIZE=5
BATCH_DELAY=3
RELAY_BIN="/Users/e/Desktop/develop/lumina-video/moq/target/release/moq-relay"
SUB_BIN="/Users/e/Desktop/develop/telemoq/target/release/telemoq"
CSV_DIR="fleet-csv"
SKIP_RELAY=false

for arg in "$@"; do
  [[ "$arg" == "--no-relay" ]] && SKIP_RELAY=true
done

mkdir -p "$CSV_DIR"

# Start relay
if [[ "$SKIP_RELAY" == false ]]; then
  echo ">>> Killing existing relay..."
  pkill -f moq-relay 2>/dev/null || true
  sleep 1
  echo ">>> Starting moq-relay..."
  "$RELAY_BIN" \
    --server-bind='[::]:4443' \
    --tls-generate=localhost \
    --auth-public='' \
    --log-level=warn &
  RELAY_PID=$!
  echo "    relay PID=$RELAY_PID"
  sleep 2
fi

# Start subscribers in batches
for i in $(seq 1 "$NUM_ROBOTS"); do
  BROADCAST="robot-$i"
  CSV_FILE="$CSV_DIR/$BROADCAST.csv"
  echo ">>> Starting subscriber: $BROADCAST → $CSV_FILE"
  "$SUB_BIN" \
    --url https://localhost:4443 \
    --tls-disable-verify \
    --broadcast "$BROADCAST" \
    --csv \
    subscribe > "$CSV_FILE" 2>&1 &
  echo "    subscriber PID=$!"

  # Pause between batches
  if (( i % BATCH_SIZE == 0 && i < NUM_ROBOTS )); then
    echo "--- Batch of $BATCH_SIZE started. Waiting ${BATCH_DELAY}s before next batch..."
    sleep "$BATCH_DELAY"
  fi
done

echo ""
echo "=== Fleet test: $NUM_ROBOTS subscribers running ==="
echo "=== CSV output in $CSV_DIR/ ==="
echo "=== Now start publishers on Lemur ==="
echo "=== Stop with: pkill -f telemoq; pkill -f moq-relay ==="
echo ""

# Tail all subscriber CSVs
tail -f "$CSV_DIR"/robot-*.csv
