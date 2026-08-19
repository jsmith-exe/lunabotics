#!/usr/bin/env bash
# Alternates conditions so machine state (thermal, page cache) can't bias one arm.
# Retries a run up to 3 times: bringing up gazebo_ros2_control is racy and an
# aborted bring-up must not silently become a missing sample.
# Resolve paths relative to this script so the harness works from any checkout.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT=${1:-$REPO/tools/dr_results/runs}
CONDS=${2:-"novo vo"}
REPS=${3:-3}
mkdir -p $OUT
DOMAIN=40
for i in $(seq 1 $REPS); do
  for cond in $CONDS; do
    case "$cond" in *vo*) V=true;; *) V=false;; esac
    case "$cond" in novo*) V=false;; esac
    for attempt in 1 2 3; do
      DOMAIN=$(( (DOMAIN % 90) + 10 ))
      echo ">>> $(date +%H:%M:%S) ${cond}_$i attempt $attempt (domain $DOMAIN)"
      timeout 500 $HERE/run_dr_test.sh "${cond}_$i" "$V" "$OUT" 70 1 "$DOMAIN" >/dev/null 2>&1
      rc=$?
      if [ -s "$OUT/${cond}_$i.json" ]; then
        echo ">>> $(date +%H:%M:%S) ${cond}_$i OK"
        break
      fi
      echo ">>> $(date +%H:%M:%S) ${cond}_$i FAILED (rc=$rc), retrying"
      sleep 10
    done
    sleep 10
  done
done
echo ">>> BATCH COMPLETE"
