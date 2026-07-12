#!/usr/bin/env sh
set -eu

python3 "$(dirname "$0")/generate_wrapper_abi.py" "$@"
