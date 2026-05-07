#!/bin/bash
set -e

docker-compose -f "$(dirname "${BASH_SOURCE[0]}")/docker-compose.yml" up -d
