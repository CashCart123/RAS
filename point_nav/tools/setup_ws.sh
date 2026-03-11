#!/usr/bin/env bash
set -euo pipefail

SOURCE_PKG_DEFAULT="$HOME/RAS-main/point_nav"
WS_DEFAULT="$HOME/point_nav_ws"

SOURCE_PKG="${1:-$SOURCE_PKG_DEFAULT}"
WS_ROOT="${2:-$WS_DEFAULT}"
WS_SRC="$WS_ROOT/src"
WS_LINK="$WS_SRC/point_nav"

if [[ ! -d "$SOURCE_PKG" ]]; then
  echo "ERROR: source package not found: $SOURCE_PKG" >&2
  exit 1
fi

mkdir -p "$WS_SRC"

if [[ -e "$WS_LINK" && ! -L "$WS_LINK" ]]; then
  echo "ERROR: $WS_LINK exists and is not a symlink. Remove or rename it first." >&2
  exit 1
fi

ln -sfn "$SOURCE_PKG" "$WS_LINK"

echo "Linked:"
echo "  $WS_LINK -> $SOURCE_PKG"
echo
echo "Next:"
echo "  cd $WS_ROOT"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select point_nav --symlink-install"
echo "  . install/setup.bash"
