#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ACTION="${1:-up}"

case "$ACTION" in
  up)
    echo "=== Starting H3 Robot container ==="
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" up -d
    echo "=== Container started. Attach with: docker exec -it h3_robot bash ==="
    ;;
  down)
    echo "=== Stopping H3 Robot container ==="
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" down
    ;;
  logs)
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" logs -f
    ;;
  restart)
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" restart
    ;;
  shell)
    docker exec -it h3_robot bash
    ;;
  *)
    echo "Usage: $0 {up|down|logs|restart|shell}"
    exit 1
    ;;
esac
