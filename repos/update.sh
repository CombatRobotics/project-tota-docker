#!/bin/bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Exit on error
set -e

# Check if repos directory exists
if [ ! -d "${repo_root}/repos" ]; then
    echo "Error: repos directory not found"
    exit 1
fi

# Function to ensure branches match ally.repos
ensure_branches() {
    local ws_dir=$1
    local repos_file="${repo_root}/repos/ally.repos"
    
    if [ ! -f "$repos_file" ]; then
        echo "Warning: ally.repos file not found, skipping branch check"
        return
    fi
    
    echo "Checking branches..."
    
    # Parse the repos file and switch branches if needed
    local repo_name=""
    local branch=""
    local in_repo=false
    local repos_changed=0
    
    while IFS= read -r line; do
        # Skip empty lines and comments
        if [[ -z "$line" ]] || [[ "$line" =~ ^[[:space:]]*$ ]] || [[ "$line" =~ ^[[:space:]]*# ]]; then
            continue
        fi
        
        # Check if it's a repo name line (two spaces, name, colon)
        if [[ "$line" =~ ^"  "([a-zA-Z0-9_-]+):$ ]]; then
            repo_name="${BASH_REMATCH[1]}"
            in_repo=true
            branch=""
        # Check if it's a version line (four spaces, "version:", value)
        elif [[ "$line" =~ ^"    version: "(.+)$ ]] && [ "$in_repo" = true ]; then
            branch="${BASH_REMATCH[1]}"
            
            local repo_dir="${ws_dir}/src/${repo_name}"
            
            if [ -d "$repo_dir/.git" ]; then
                # Get current branch
                local current_branch=$(git -C "$repo_dir" branch --show-current)
                
                # Only switch if we're not already on the target branch
                if [ "$current_branch" != "$branch" ]; then
                    echo "  Switching ${repo_name}: ${current_branch} → ${branch}"
                    repos_changed=$((repos_changed + 1))
                    
                    # Try to checkout the branch (fetch will happen with vcs pull later)
                    git -C "$repo_dir" checkout "${branch}" 2>/dev/null || {
                        # If local branch doesn't exist, try to create it from origin
                        git -C "$repo_dir" fetch origin "${branch}:${branch}" 2>/dev/null && \
                        git -C "$repo_dir" checkout "${branch}" 2>/dev/null || \
                        echo "    Warning: Could not switch to ${branch}"
                    }
                fi
            elif [ -n "$repo_name" ]; then
                # Repo doesn't exist, but that's ok - vcs import will handle it
                :
            fi
            
            # Reset for next repo
            in_repo=false
        fi
    done < "$repos_file"
    
    if [ $repos_changed -gt 0 ]; then
        echo "Switched ${repos_changed} repo(s) to correct branch"
    fi
}

# Function to update device
update_device() {
    device_type="controller"
    ws_dir="${repo_root}/ws/${device_type}_ws"

    cd "$ws_dir" || { echo -e "\e[31m Directory $ws_dir not found! \e[0m"; exit 1; }
    
    # Check for uncommitted changes in workspace
    # Temporarily disable exit on error since check_changes.sh returns 1 when changes exist
    set +e
    "${repo_root}/check_changes.sh"
    has_local_changes=$?
    set -e

    if [ $has_local_changes -eq 1 ]; then
        echo -e "\n\e[33mWARNING: Local changes detected in $ws_dir. This will remove all local changes! \e[0m"
        read -p "Are you sure you want to continue? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Update cancelled ..."
            exit 1
        else   
            echo "Updating ${device_type}_ws..."
            # First reset hard to remove local changes
            vcs custom --args reset --hard
            # Ensure branches match ally.repos
            ensure_branches "$ws_dir"
            # Pull all repos at once
            echo "Pulling latest changes..."
            vcs pull
        fi
    elif [ $has_local_changes -eq 2 ]; then
        echo -e "\e[32mNo local changes detected. Proceeding with update...\e[0m"
        # Ensure branches match ally.repos
        ensure_branches "$ws_dir"
        # Pull all repos at once
        echo "Pulling latest changes..."
        vcs pull
    else
        echo -e "\e[32mThe SRC directory is empty. Exiting...\e[0m"
        exit 1
    fi

    colcon build --symlink-install
    source install/setup.bash
    echo "$device_type updated successfully"

    source ~/.bashrc
    exit 0
}

update_device