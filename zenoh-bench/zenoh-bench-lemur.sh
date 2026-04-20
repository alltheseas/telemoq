#!/usr/bin/env bash
# Zenoh transport benchmark — Lemur side (publisher)
# Usage: ./zenoh-bench-lemur.sh <tcp|quic|quic-ms> [DURATION] [MAC_IP]
#
# Publishes 8 tracks (IHMC KST schema) to zenohd running on Mac Mini.
# Start the Mac Mini script FIRST, then run this.
#
# Prerequisites:
#   cargo build --release   (in this directory)

set -euo pipefail

MODE="${1:?Usage: $0 <tcp|quic|quic-ms> [DURATION] [MAC_IP]}"
DURATION="${2:-63}"
MAC_IP="${3:-192.168.8.213}"
PORT=7447
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BENCH_BIN="$SCRIPT_DIR/target/release/zenoh-teleop-bench"

case "$MODE" in
  tcp)
    CONNECT="tcp/${MAC_IP}:${PORT}"
    CLIENT_CFG=""
    ;;
  quic)
    CONNECT=""
    CLIENT_CFG="$SCRIPT_DIR/config/client-quic.json5"
    ;;
  quic-ms)
    CONNECT=""
    CLIENT_CFG="$SCRIPT_DIR/config/client-quic-ms.json5"
    ;;
  *)
    echo "ERROR: unknown mode '$MODE'. Use: tcp, quic, quic-ms"
    exit 1
    ;;
esac

if [[ ! -x "$BENCH_BIN" ]]; then
  echo "ERROR: $BENCH_BIN not found. Run: cargo build --release"
  exit 1
fi

# Clear tc netem if any
sudo tc qdisc del dev wlp0s20f3 root 2>/dev/null || true

echo "=== Zenoh publisher benchmark ==="
echo "    mode:     $MODE"
echo "    duration: ${DURATION}s"
echo ""

if [[ -n "$CLIENT_CFG" ]]; then
  echo "    config: $CLIENT_CFG"
  "$BENCH_BIN" \
    --config "$CLIENT_CFG" \
    --duration "$DURATION" \
    publish 2>&1
else
  echo "    connect: $CONNECT"
  "$BENCH_BIN" \
    --connect "$CONNECT" \
    --duration "$DURATION" \
    publish 2>&1
fi
