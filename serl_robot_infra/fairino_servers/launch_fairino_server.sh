#!/usr/bin/env bash
set -euo pipefail

# Example launcher for the Fairino servo Flask server.
# Usage:
#   bash serl_robot_infra/fairino_servers/launch_fairino_server.sh
# Or override:
#   ROBOT_IP=192.168.58.6 HOST=127.0.0.1 PORT=5000 SERVO_MODE=cart CMDT=0.008 DOF=6 bash serl_robot_infra/fairino_servers/launch_fairino_server.sh

ROBOT_IP="${ROBOT_IP:-192.168.58.6}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-5000}"
SERVO_MODE="${SERVO_MODE:-cart}"   # cart|joint
CMDT="${CMDT:-0.008}"
DOF="${DOF:-6}"

python -m fairino_servers.fairino_server \
  --robot_ip="${ROBOT_IP}" \
  --flask_url="${HOST}" \
  --port="${PORT}" \
  --servo_mode="${SERVO_MODE}" \
  --cmdT="${CMDT}" \
  --dof="${DOF}"


