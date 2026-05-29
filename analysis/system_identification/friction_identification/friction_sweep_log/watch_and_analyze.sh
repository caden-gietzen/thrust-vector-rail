#!/usr/bin/env bash
# watch_and_analyze.sh
#
# Polls candidate/ for 18 CSV files (pulling from remote each cycle),
# then runs the Python friction analysis and commits the results.

set -euo pipefail

REPO_ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
CANDIDATE_DIR="$REPO_ROOT/data/raw/system_identification/friction_identification/friction_sweep_log/candidate"
SCRIPT_DIR="$REPO_ROOT/analysis/system_identification/friction_identification/friction_sweep_log"
BRANCH="$(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD)"
TARGET=18
POLL_SECS=15

echo "[watch] Monitoring: $CANDIDATE_DIR"
echo "[watch] Target: $TARGET CSV files"
echo "[watch] Branch: $BRANCH"
echo "[watch] Poll interval: ${POLL_SECS}s"
echo ""

while true; do
    # Pull latest from remote so new data files appear locally
    git -C "$REPO_ROOT" pull --ff-only origin "$BRANCH" 2>&1 | grep -v "^Already up to date" || true

    N=$(find "$CANDIDATE_DIR" -maxdepth 1 -name "*.csv" | wc -l)
    echo "[watch] $(date '+%H:%M:%S')  candidate files: $N / $TARGET"

    if [ "$N" -ge "$TARGET" ]; then
        echo ""
        echo "[watch] Threshold reached ($N files). Running analysis..."
        echo ""
        break
    fi

    sleep "$POLL_SECS"
done

# Run analysis
cd "$SCRIPT_DIR"
python3 analyze_friction_sweep.py
STATUS=$?

if [ "$STATUS" -ne 0 ]; then
    echo "[watch] Analysis exited with status $STATUS — not committing."
    exit "$STATUS"
fi

# Commit results
PLOT_DIR="$REPO_ROOT/plots/system_identification/friction_identification/friction_sweep_log"
REPORT="$PLOT_DIR/analyze_friction_sweep.report.md"

git -C "$REPO_ROOT" add \
    "$SCRIPT_DIR/analyze_friction_sweep.py" \
    "$SCRIPT_DIR/watch_and_analyze.sh" \
    "$PLOT_DIR"

git -C "$REPO_ROOT" commit -m "$(cat <<'EOF'
Add friction analysis results (18-file dataset)

Run analyze_friction_sweep.py on candidate data once 18 CSV files
were present; saves figures and markdown report to plots/.

https://claude.ai/code/session_01FnEpXSWc3SBgqzoPQBZFsq
EOF
)"

git -C "$REPO_ROOT" push -u origin "$BRANCH"
echo ""
echo "[watch] Done. Results committed and pushed to $BRANCH."
