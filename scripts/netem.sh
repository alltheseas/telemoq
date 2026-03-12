#!/usr/bin/env bash
#
# Network impairment for telemoq demo.
# Applies bandwidth limiting (primary) and optional packet loss to loopback.
#
# Usage:
#   ./scripts/netem.sh start [bw_kbit] [loss_pct] [delay_ms]
#   ./scripts/netem.sh stop
#
# Defaults: 500 kbit/s bandwidth, 0% loss, 0ms delay
# Example:  ./scripts/netem.sh start 500 3 20

set -euo pipefail

ACTION="${1:-help}"
BW_KBIT="${2:-500}"
LOSS_PCT="${3:-0}"
DELAY_MS="${4:-0}"

PORT=4443

case "$(uname -s)" in
  Linux)
    case "$ACTION" in
      start)
        echo "Linux: applying bandwidth limit ${BW_KBIT}kbit/s, loss ${LOSS_PCT}%, delay ${DELAY_MS}ms on lo"
        sudo tc qdisc del dev lo root 2>/dev/null || true
        if [ "$LOSS_PCT" != "0" ] || [ "$DELAY_MS" != "0" ]; then
          sudo tc qdisc add dev lo root handle 1: netem loss "${LOSS_PCT}%" delay "${DELAY_MS}ms"
          sudo tc qdisc add dev lo parent 1: handle 2: tbf rate "${BW_KBIT}kbit" burst 10kb latency 50ms
        else
          sudo tc qdisc add dev lo root tbf rate "${BW_KBIT}kbit" burst 10kb latency 50ms
        fi
        echo "Impairment active. Run '$0 stop' to remove."
        ;;
      stop)
        echo "Linux: removing impairment from lo"
        sudo tc qdisc del dev lo root 2>/dev/null || true
        echo "Impairment removed."
        ;;
      *)
        echo "Usage: $0 start|stop [bw_kbit] [loss_pct] [delay_ms]"
        exit 1
        ;;
    esac
    ;;
  Darwin)
    case "$ACTION" in
      start)
        echo "macOS: applying bandwidth limit ${BW_KBIT}Kbit/s, loss ${LOSS_PCT}%, delay ${DELAY_MS}ms on port $PORT"
        # Clean up any existing rules
        sudo dnctl pipe 1 delete 2>/dev/null || true
        sudo pfctl -F all 2>/dev/null || true

        # Build dnctl config
        DNCTL_ARGS="bw ${BW_KBIT}Kbit/s"
        [ "$LOSS_PCT" != "0" ] && DNCTL_ARGS="$DNCTL_ARGS plr $(echo "scale=4; $LOSS_PCT / 100" | bc)"
        [ "$DELAY_MS" != "0" ] && DNCTL_ARGS="$DNCTL_ARGS delay ${DELAY_MS}"

        sudo dnctl pipe 1 config $DNCTL_ARGS

        # Apply to loopback traffic on the relay port
        echo "dummynet in on lo0 proto udp from any to any port $PORT pipe 1
dummynet out on lo0 proto udp from any port $PORT to any pipe 1" | sudo pfctl -f -
        sudo pfctl -e 2>/dev/null || true
        echo "Impairment active. Run '$0 stop' to remove."
        ;;
      stop)
        echo "macOS: removing impairment"
        sudo pfctl -F all 2>/dev/null || true
        sudo pfctl -d 2>/dev/null || true
        sudo dnctl pipe 1 delete 2>/dev/null || true
        echo "Impairment removed."
        ;;
      *)
        echo "Usage: $0 start|stop [bw_kbit] [loss_pct] [delay_ms]"
        exit 1
        ;;
    esac
    ;;
  *)
    echo "Unsupported OS: $(uname -s)"
    exit 1
    ;;
esac
