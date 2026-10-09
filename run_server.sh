#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${REMOTE_CONTROLLER_CONFIG:-$ROOT_DIR/config/remote_controller.env}"

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Config file not found: $CONFIG_FILE" >&2
    exit 1
fi

set -a
source "$CONFIG_FILE"
set +a

: "${ROBOT_IP:?ROBOT_IP is required}"
: "${GRIPPER_IP:?GRIPPER_IP is required}"
: "${XMLRPC_PORT:?XMLRPC_PORT is required}"
: "${SERVER_BINARY:?SERVER_BINARY is required}"

SERVER_PATH="$SERVER_BINARY"
if [[ "$SERVER_PATH" != /* ]]; then
    SERVER_PATH="$ROOT_DIR/$SERVER_PATH"
fi

if [[ ! -x "$SERVER_PATH" ]]; then
    echo "Server binary not executable: $SERVER_PATH" >&2
    echo "Build it first:" >&2
    echo "  cd $ROOT_DIR/cpp_server" >&2
    echo "  cmake -S . -B build" >&2
    echo "  cmake --build build -j" >&2
    exit 1
fi

echo "Starting remote_controller_server"
echo "robot_ip:    $ROBOT_IP"
echo "gripper_ip:  $GRIPPER_IP"
echo "xmlrpc_port: $XMLRPC_PORT"
echo "binary:      $SERVER_PATH"

exec sudo "$SERVER_PATH" "$ROBOT_IP" "$GRIPPER_IP" "$XMLRPC_PORT"
