#!/usr/bin/env bash
# Zenoh transport benchmark — Mac Mini side (zenohd router + subscriber)
# Usage: ./zenoh-bench-mac.sh <tcp|quic|quic-ms> [DURATION]
#
# Runs zenohd with the specified transport, then a subscriber that outputs CSV.
# Start this FIRST, then start the publisher on Lemur.
#
# Prerequisites:
#   cargo install zenohd --version 1.9.0
#   cargo build --release   (in this directory)

set -euo pipefail

MODE="${1:?Usage: $0 <tcp|quic|quic-ms> [DURATION]}"
DURATION="${2:-63}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BENCH_BIN="$SCRIPT_DIR/target/release/zenoh-teleop-bench"
CSV_DIR="$SCRIPT_DIR/benchmarks/zenoh-transport"
PORT=7447

mkdir -p "$CSV_DIR"

# Validate mode and select config + connect endpoint
case "$MODE" in
  tcp)
    ZENOHD_CFG="$SCRIPT_DIR/config/zenohd-tcp.json5"
    CLIENT_CFG=""
    CONNECT="tcp/127.0.0.1:${PORT}"
    ;;
  quic)
    ZENOHD_CFG="$SCRIPT_DIR/config/zenohd-quic.json5"
    CLIENT_CFG=""
    CONNECT="tcp/127.0.0.1:7448"
    ;;
  quic-ms)
    ZENOHD_CFG="$SCRIPT_DIR/config/zenohd-quic-ms.json5"
    CLIENT_CFG=""
    CONNECT="tcp/127.0.0.1:7448"
    ;;
  *)
    echo "ERROR: unknown mode '$MODE'. Use: tcp, quic, quic-ms"
    exit 1
    ;;
esac

# Check prerequisites
if ! command -v zenohd &>/dev/null; then
  echo "ERROR: zenohd not found. Install with: cargo install zenohd --version 1.9.0"
  exit 1
fi
if [[ ! -x "$BENCH_BIN" ]]; then
  echo "ERROR: $BENCH_BIN not found. Run: cargo build --release"
  exit 1
fi

# Kill any existing zenohd/bench processes
pkill -f zenohd 2>/dev/null || true
pkill -f zenoh-teleop-bench 2>/dev/null || true
sleep 1

# Start zenohd
echo ">>> Starting zenohd (mode=$MODE)"
echo "    config: $ZENOHD_CFG"
zenohd --config "$ZENOHD_CFG" > "$CSV_DIR/zenohd-${MODE}.log" 2>&1 &
ZENOHD_PID=$!
echo "    zenohd PID=$ZENOHD_PID"
sleep 2

# Verify zenohd is running
if ! kill -0 "$ZENOHD_PID" 2>/dev/null; then
  echo "ERROR: zenohd failed to start. Check $CSV_DIR/zenohd-${MODE}.log"
  cat "$CSV_DIR/zenohd-${MODE}.log"
  exit 1
fi

TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
CSV_FILE="$CSV_DIR/zenoh-${MODE}-${TIMESTAMP}.csv"

echo ""
echo "=== Ready. Start publisher on Lemur now: ==="
echo "    ./zenoh-bench-lemur.sh $MODE $DURATION"
echo ""
echo ">>> CSV output: $CSV_FILE"
echo ">>> Duration: ${DURATION}s"
echo ""

# For QUIC modes, use --config (includes TLS settings); for TCP, use --connect
if [[ -n "$CLIENT_CFG" ]]; then
  echo ">>> Subscriber using config: $CLIENT_CFG"
  "$BENCH_BIN" \
    --config "$CLIENT_CFG" \
    --duration "$DURATION" \
    subscribe > "$CSV_FILE" 2>"$CSV_DIR/sub-${MODE}-${TIMESTAMP}.log"
else
  echo ">>> Subscriber connecting: $CONNECT"
  "$BENCH_BIN" \
    --connect "$CONNECT" \
    --duration "$DURATION" \
    subscribe > "$CSV_FILE" 2>"$CSV_DIR/sub-${MODE}-${TIMESTAMP}.log"
fi

echo ""
echo "=== Done: $CSV_FILE ==="
echo ""

# Cleanup
kill "$ZENOHD_PID" 2>/dev/null || true
wait "$ZENOHD_PID" 2>/dev/null || true
