#!/usr/bin/env bash
# Unified ROSA start wrapper. Delegates execution to scripts/run_rosa_with_ros.sh
# so both entry points run the same dynamically-loaded tool stack from src/.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="$REPO/scripts/run_rosa_with_ros.sh"
PY_ENTRY="${ROSA_PY_ENTRY:-src/rosa_agent.py}"

echo "============================================================================"
echo "ROSA Robot Controller - Unified Start"
echo "============================================================================"

if [[ ! -f "$RUNNER" ]]; then
  echo "Missing runner script: $RUNNER" >&2
  exit 1
fi

# Preserve existing behavior of loading optional API keys for LLM providers.
if [[ -f "$HOME/.rosa_env" ]]; then
  # shellcheck source=/dev/null
  source "$HOME/.rosa_env"
fi

# Keep a run marker for easier log correlation.
export ROSA_DEBUG_RUN_ID="run-$(date +%s)"

exec "$RUNNER" "$PY_ENTRY" "$@"

