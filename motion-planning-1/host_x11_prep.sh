#!/usr/bin/env bash
set -e
xhost +local:root
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - || true
chmod 600 $XAUTH
echo "X11 ready: $XAUTH"
