#!/bin/bash
# Script to ensure Claude configuration persists across rebuilds

# Detect user home directory
USER_HOME="${HOME:-/home/devuser}"

# Function to save Claude configuration
save_claude() {
    echo "Saving Claude configuration..."
    if [ -d ~/.claude ]; then
        rm -rf ${USER_HOME}/.config/claude/.claude 2>/dev/null || true
        cp -r ~/.claude ${USER_HOME}/.config/claude/.claude
        echo "✓ Claude configuration saved to volume"
    else
        echo "⚠ No Claude configuration found to save"
    fi
}

# Function to restore Claude configuration
restore_claude() {
    echo "Restoring Claude configuration..."
    if [ -d "${USER_HOME}/.config/claude/.claude" ]; then
        rm -rf ~/.claude 2>/dev/null || true
        cp -r ${USER_HOME}/.config/claude/.claude ~/.claude 2>/dev/null || true
        echo "✓ Claude configuration restored from volume"
    else
        echo "⚠ No Claude configuration found in volume"
    fi
}

# Main logic
case "${1:-check}" in
    save)
        save_claude
        ;;
    restore)
        restore_claude
        ;;
    *)
        echo "Checking Claude persistence setup..."

        # Create necessary directories if they don't exist
        mkdir -p ${USER_HOME}/.config/claude
        mkdir -p ${USER_HOME}/.local/share/claude
        mkdir -p ${USER_HOME}/.cache/claude

        # Check if volumes are properly mounted
        if mount | grep -q "tota-claude-config"; then
            echo "✓ Claude config volume mounted"
        else
            echo "⚠ Claude config volume not found"
        fi

        # Set proper permissions (only if running as root)
        if [ "$(id -u)" -eq 0 ]; then
            chown -R devuser:devuser ${USER_HOME}/.config/claude 2>/dev/null || true
            chown -R devuser:devuser ${USER_HOME}/.local/share/claude 2>/dev/null || true
            chown -R devuser:devuser ${USER_HOME}/.cache/claude 2>/dev/null || true
        fi

        # Try to restore if available
        restore_claude

        echo "Claude persistence check complete"
        ;;
esac