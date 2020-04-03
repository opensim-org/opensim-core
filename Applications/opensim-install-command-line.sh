#!/bin/bash
# This script makes the OpenSim command-line tools accessible from any
# directory (assuming /usr/local/bin is on the PATH, by being listed in the file
# /etc/paths).
# This script should be run from the directory containing opensim-cmd and
# opensense.

if [ ! -f "$(pwd)/opensim-cmd" ]; then
    echo "Could not find opensim-cmd in the current working directory. Aborting."
    exit 1;
fi

# Create symbolic links.
# -i: Prompt the user if the target file already exists.
# -s: Create a symbolic link.
sudo ln -i -s "$(pwd)/opensim-cmd" /usr/local/bin/opensim-cmd
sudo ln -i -s "$(pwd)/opensense" /usr/local/bin/opensense
# Un-cache the password, so that the next time sudo is used, a password is
# required.
sudo -k

echo "Created symlinks to opensim-cmd and opensense in /usr/local/bin."
echo "You can now use these commands from any directory."
