#!/usr/bin/env bash
# Low-friction condition. Changes nothing in the localisation stack - it only
# drops the wheels' longitudinal friction so the wheels actually slip, which is
# the failure mode VO is meant to cover for on regolith. Restores the xacro and
# rebuilds on exit, whatever happens.
set +u
# Resolve paths relative to this script so the harness works from any checkout.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
XACRO="$REPO/qpl_ws/src/qpl_rover/description/rover_core.xacro"
BACKUP="$(mktemp -d)/rover_core.xacro.orig"
cp "$XACRO" "$BACKUP"

rebuild() {
  cd "$REPO/qpl_ws" || return 1
  source /opt/ros/humble/setup.bash
  colcon build --packages-select qpl_rover --symlink-install >/dev/null 2>&1
}
restore() {
  cp "$BACKUP" "$XACRO"
  rebuild
  echo ">>> friction restored to $(grep -o '<mu1>[^<]*' "$XACRO" | head -1)"
}
trap restore EXIT

sed -i 's|<mu1>1.4</mu1>|<mu1>0.35</mu1>|' "$XACRO"
rebuild
echo ">>> rebuilt with $(grep -o '<mu1>[^<]*' "$XACRO" | head -1)"

"$HERE/batch.sh" "$REPO/tools/dr_results/runs_slip" "novo_slip vo_slip" 2
echo ">>> SLIP BATCH COMPLETE"
