#!/usr/bin/env bash
# Build and run the native unit tests for the Arduino-free core modules.
# Rule 10: compile at the most pedantic setting and treat warnings as errors.
set -euo pipefail
cd "$(dirname "$0")"

CORE="classification.cpp device_table.cpp known_cache.cpp persistence.cpp"
TEST="test/test_native/test_helpers.cpp"
OUT="$(mktemp -d)/test_pax"

g++ -std=c++17 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror \
    -I . ${CORE} ${TEST} -o "${OUT}"

"${OUT}"
