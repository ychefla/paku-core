#!/bin/bash
# Configure git to use the project's .githooks directory.
# Run once after cloning: bash scripts/setup-hooks.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

git -C "$REPO_ROOT" config core.hooksPath .githooks
chmod +x "$REPO_ROOT"/.githooks/*

echo "✅  Git hooks configured for paku-core"
echo "    Commit messages are now validated against Conventional Commits format."
echo ""
echo "    fix: ...     → patch bump (1.4.2 → 1.4.3)"
echo "    feat: ...    → minor bump (1.4.2 → 1.5.0)"
echo "    feat!: ...   → major bump (1.4.2 → 2.0.0)"
