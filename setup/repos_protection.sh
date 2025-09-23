#!/bin/bash

# Ally.repos Merge Protection Setup Script
# This script configures Git to prevent repos/ally.repos from being merged between branches

set -e

echo "Setting up ally.repos merge protection..."

# Check if we're in a git repository
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    echo "Error: Not in a git repository"
    exit 1
fi

# Get repository root
REPO_ROOT=$(git rev-parse --show-toplevel)
cd "$REPO_ROOT"

echo "Repository root: $REPO_ROOT"

# 1. Add to .gitattributes
echo "Adding merge protection to .gitattributes..."
if ! grep -q "repos/ally.repos merge=keepmine" .gitattributes 2>/dev/null; then
    echo "# Prevent ally.repos from being merged - always keep the current branch's version" >> .gitattributes
    echo "repos/ally.repos merge=keepmine" >> .gitattributes
    echo "✓ Added to .gitattributes"
else
    echo "✓ Already configured in .gitattributes"
fi

# 2. Create merge driver script
echo "Creating custom merge driver script..."
cat > .git/keep-mine-driver.sh << 'EOF'
#!/bin/bash
# Custom merge driver that always keeps the current branch's version
# Arguments: %O %A %B %L %P
# %O = ancestor's version
# %A = current branch's version  
# %B = other branch's version
# %L = conflict marker size
# %P = file path

# Simply copy current branch's version (%A) as the result
cp "$2" "$2"
exit 0
EOF

chmod +x .git/keep-mine-driver.sh
echo "✓ Created merge driver script"

# 3. Configure git merge driver
echo "Configuring Git merge driver..."
git config merge.keepmine.driver "$REPO_ROOT/.git/keep-mine-driver.sh %O %A %B %L %P"
echo "✓ Configured Git merge driver"

# 4. Create post-merge hook
echo "Creating post-merge hook..."
cat > .git/hooks/post-merge << 'EOF'
#!/bin/bash
# Post-merge hook to reset ally.repos to the pre-merge state

# Check if ally.repos was modified in this merge
if git diff HEAD@{1} HEAD --name-only | grep -q "repos/ally.repos"; then
    echo "Resetting repos/ally.repos to maintain current branch configuration..."
    git checkout HEAD@{1} -- repos/ally.repos
    git add repos/ally.repos
    git commit -m "Maintain ally.repos configuration after merge

🤖 Generated with [Claude Code](https://claude.ai/code)

Co-Authored-By: Claude <noreply@anthropic.com>"
fi
EOF

chmod +x .git/hooks/post-merge
echo "✓ Created post-merge hook"

echo ""
echo "✅ Ally.repos merge protection setup complete!"
echo ""
echo "How it works:"
echo "  - Custom merge driver prevents Git from merging changes to repos/ally.repos"
echo "  - Post-merge hook automatically resets the file if it was changed during merge"
echo ""
echo "Now when you merge branches, repos/ally.repos will maintain the current branch's configuration."