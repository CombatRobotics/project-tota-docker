#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
device_type="controller"
ws_dir="${repo_root}/ws/${device_type}_ws"

echo "Checking for local changes in $ws_dir..."

cd "$ws_dir" || { echo -e "\e[31m Directory $ws_dir not found! \e[0m"; exit 0; }

# Check for uncommitted changes
changed_repos=()
if [[ -d "$ws_dir/src" ]]; then
    for repo in "$ws_dir/src"/*; do
        if [[ -d "$repo/.git" ]]; then
            if [[ -n $(git -C "$repo" status --porcelain) ]]; then
                changed_repos+=("$(basename "$repo")")
            fi
        fi
    done
fi

# Output JSON
if [ ${#changed_repos[@]} -eq 0 ]; then
    echo 'No local changes detected'
    exit 2
else
    printf 'Local changes detected in: ['
    for i in "${!changed_repos[@]}"; do
        printf '"%s"' "${changed_repos[$i]}"
        if [ $i -lt $((${#changed_repos[@]} - 1)) ]; then
            printf ', '
        fi
    done
    printf ']}\n'
    exit 1
fi