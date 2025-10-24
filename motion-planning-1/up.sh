#!/usr/bin/env bash
set -e
bash ./host_x11_prep.sh
docker compose up -d
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
