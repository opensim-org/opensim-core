# List all commits on current branch that are not on main, newest first
commits=$(git rev-list main..HEAD)

# Create a directory to hold the patches
mkdir -p patches

# Loop over each commit and save diff to a separate file
for commit in $commits; do
  git diff $commit^ $commit > patches/${commit}.diff
done