#!/bin/bash
# Script to ensure Claude configuration persists across rebuilds

# Function to save Claude configuration
save_claude() {
    echo "Saving Claude configuration..."
    if [ -d ~/.claude ]; then
        rm -rf /root/.config/claude/.claude 2>/dev/null || true
        cp -r ~/.claude /root/.config/claude/.claude
        echo "✓ Claude configuration saved to volume"
    else
        echo "⚠ No Claude configuration found to save"
    fi
}

# Function to restore Claude configuration
restore_claude() {
    echo "Restoring Claude configuration..."
    if [ -d "/root/.config/claude/.claude" ]; then
        rm -rf ~/.claude 2>/dev/null || true
        cp -r /root/.config/claude/.claude ~/.claude 2>/dev/null || true
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
        mkdir -p /root/.config/claude
        mkdir -p /root/.local/share/claude
        mkdir -p /root/.cache/claude

        # Check if volumes are properly mounted
        if mount | grep -q "tota-claude-config"; then
            echo "✓ Claude config volume mounted"
        else
            echo "⚠ Claude config volume not found"
        fi

        # Set proper permissions
        chown -R root:root /root/.config/claude 2>/dev/null || true
        chown -R root:root /root/.local/share/claude 2>/dev/null || true
        chown -R root:root /root/.cache/claude 2>/dev/null || true

        # Try to restore if available
        restore_claude

        echo "Claude persistence check complete"
        ;;
esac