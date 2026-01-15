#!/bin/bash

# --- 1. CONFIGURATION & MESSAGE CAPTURE ---
if [ "$1" == "-dev" ]; then
    BRANCH="dev"
    shift
    MESSAGE="$*"
else
    BRANCH="main"
    MESSAGE="$*"
fi

# --- 2. VALIDATION ---
if [ -z "$MESSAGE" ]; then
    echo "❌ Error: No commit message provided."
    echo "Usage: ./git.sh [-dev] your message here"
    exit 1
fi

echo "--------------------------------------------"
echo "🛠️  Target Branch: $BRANCH"
echo "💬 Message: $MESSAGE"
echo "--------------------------------------------"

# --- 3. BRANCH MANAGEMENT ---
echo "🌿 Checking out $BRANCH..."
git checkout $BRANCH 2>/dev/null || git checkout -b $BRANCH

# --- 4. GIT WORKFLOW ---
echo "📦 Adding changes..."
git add .

echo "📝 Committing..."
# Capture output to check if there were actually changes
COMMIT_OUTPUT=$(git commit -m "$MESSAGE" 2>&1)
if echo "$COMMIT_OUTPUT" | grep -q "nothing to commit"; then
    echo "⚠️  Nothing to commit (working tree clean)."
else
    echo "✅ Commit successful."
fi

echo "🔄 Pulling latest from GitHub (Rebase)..."
if git pull origin $BRANCH --rebase; then
    echo "✅ Sync complete."
else
    echo "❌ Sync failed! You might have conflicts to resolve manually."
    exit 1
fi

echo "🚀 Pushing to origin..."
if git push origin $BRANCH; then
    echo "--------------------------------------------"
    echo "✨ SUCCESS: Your changes are now on GitHub!"
    echo "🔗 URL: https://github.com/Agroecology-Lab/Open_agbot_devkit_ros/tree/$BRANCH"
    echo "--------------------------------------------"
else
    echo "❌ Push failed. Check your token or network."
    exit 1
fi
