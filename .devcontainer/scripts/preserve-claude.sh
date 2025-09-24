#!/bin/bash
# Script to ensure Claude configuration persists across rebuilds

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

if mount | grep -q "tota-claude-data"; then
    echo "✓ Claude data volume mounted"
else
    echo "⚠ Claude data volume not found"
fi

if mount | grep -q "tota-claude-cache"; then
    echo "✓ Claude cache volume mounted"
else
    echo "⚠ Claude cache volume not found"
fi

# Set proper permissions
chown -R root:root /root/.config/claude 2>/dev/null || true
chown -R root:root /root/.local/share/claude 2>/dev/null || true
chown -R root:root /root/.cache/claude 2>/dev/null || true

echo "Claude persistence check complete"