#!/usr/bin/env bash
#
# One-command telemoq DROID replay demo.
# Starts relay + publisher + opens web viewer.
#
# Usage:
#   ./scripts/demo.sh                    # clean network
#   ./scripts/demo.sh --impair           # with bandwidth limiting
#   ./scripts/demo.sh --impair 300       # custom bandwidth (kbit/s)
#
# Prerequisites:
#   - moq-relay built at ../lumina-video/moq/
#   - sample_data/ present (or --replay path)
#   - cargo build completed

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RELAY_DIR="$PROJECT_DIR/../lumina-video/moq"
REPLAY_PATH="${REPLAY_PATH:-$PROJECT_DIR/sample_data}"
BROADCAST="droid-1"
RELAY_URL="https://localhost:4443"
IMPAIR=false
BW_KBIT=500

# Parse args
while [[ $# -gt 0 ]]; do
  case $1 in
    --impair)
      IMPAIR=true
      if [[ "${2:-}" =~ ^[0-9]+$ ]]; then
        BW_KBIT="$2"
        shift
      fi
      shift
      ;;
    --replay)
      REPLAY_PATH="$2"
      shift 2
      ;;
    *)
      echo "Usage: $0 [--impair [bw_kbit]] [--replay path]"
      exit 1
      ;;
  esac
done

# Verify prerequisites
if [ ! -d "$REPLAY_PATH" ]; then
  echo "Error: Replay data not found at $REPLAY_PATH"
  echo "Run: python scripts/convert_droid.py --output sample_data --episodes 3"
  exit 1
fi

if [ ! -f "$PROJECT_DIR/target/debug/telemoq" ] && [ ! -f "$PROJECT_DIR/target/release/telemoq" ]; then
  echo "Building telemoq..."
  (cd "$PROJECT_DIR" && cargo build)
fi

TELEMOQ="cargo run --manifest-path $PROJECT_DIR/Cargo.toml --"

cleanup() {
  echo ""
  echo "Shutting down..."
  [ -n "${RELAY_PID:-}" ] && kill "$RELAY_PID" 2>/dev/null || true
  [ -n "${PUB_PID:-}" ] && kill "$PUB_PID" 2>/dev/null || true
  if [ "$IMPAIR" = true ]; then
    "$SCRIPT_DIR/netem.sh" stop 2>/dev/null || true
  fi
  echo "Done."
}
trap cleanup EXIT INT TERM

# Start relay
echo "Starting moq-relay..."
(cd "$RELAY_DIR" && cargo run --bin moq-relay -- --listen '[::]:4443' --tls-generate localhost --auth-public '') &
RELAY_PID=$!
sleep 3

# Apply network impairment if requested
if [ "$IMPAIR" = true ]; then
  echo "Applying network impairment: ${BW_KBIT} kbit/s..."
  "$SCRIPT_DIR/netem.sh" start "$BW_KBIT"
fi

# Start publisher
echo "Starting DROID replay publisher..."
$TELEMOQ --url "$RELAY_URL" --tls-disable-verify --broadcast "$BROADCAST" --replay "$REPLAY_PATH" --all-cameras --loop-replay publish &
PUB_PID=$!
sleep 2

# Open viewer
VIEWER="$PROJECT_DIR/viewer.html"
if [ -f "$VIEWER" ]; then
  echo "Opening web viewer..."
  open "$VIEWER" 2>/dev/null || xdg-open "$VIEWER" 2>/dev/null || echo "Open $VIEWER in your browser"
else
  echo "Web viewer not found at $VIEWER — start terminal subscriber instead"
  $TELEMOQ --url "$RELAY_URL" --tls-disable-verify --broadcast "$BROADCAST" --replay subscribe
fi

echo ""
echo "Demo running. Press Ctrl+C to stop."
wait
