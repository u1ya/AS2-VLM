#!/usr/bin/env bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sdk="${1:-$root/.runtime/unitree_sdk2}"
prefix="$root/.runtime/unitree-sdk-install"

if [[ ! -f "$sdk/include/unitree/robot/as2/sport/sport_client.hpp" ]]; then
    printf 'AS2 SDK headers not found at: %s\n' "$sdk" >&2
    exit 1
fi

printf 'SDK revision: '
git -C "$sdk" rev-parse HEAD
cmake -S "$sdk" -B "$root/build/unitree-sdk" \
    -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX="$prefix"
cmake --install "$root/build/unitree-sdk"
cmake -S "$root/robot/bridge_cpp" -B "$root/build/as2-probe" \
    -DCMAKE_PREFIX_PATH="$prefix"
cmake --build "$root/build/as2-probe" --parallel 2
"$root/build/as2-probe/as2_state_probe" --help
printf 'Build complete. No robot connection or motion was requested.\n'
