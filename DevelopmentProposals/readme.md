## Overview

This directory contains all the development proposals for OpenSim.

## How to create a proposal

1. Create a new directory under directory ***DevelopmentProposals***.
2. Copy ***template.md*** into the new directory, and rename it appropriately.
3. Fill in the proposal details, and add any relevant code snippet files and media files to the directory.
 * Please keep the files under 1MB and as few media files (like jpg, gif, png, etc) as possible.
4. Commit your proposal to a new branch whose name begins with `devprop` (e.g., `devprop_myFeature`);
   this will tell Travis-CI and AppVeyor (automated testing servers) to not run tests on the branch.
5. Submit a pull-request to review the proposal. Make sure to *label* the pull-request with the label ***DevProp***.
6. Once approved, start work on the implementation of the proposed changes.
7. Remember to get any significant changes to proposal approved before starting to work on implementation.

## Archiving

This directory should only contain live proposals: implemented, or
stale, proposals should be moved to `archive/`.
