## Project: TITLE
***OpenSim Design Document***

### Overview

Just a few sentences that describes the purpose of this project. For example:
Create a graphical tool to plot simulation data.

### Requirements

List the set of requirements that this project must fulfill. Include all constraints this proposal is working within.
If the list gets too long, consider splitting the project into multiple small projects.

For example:

1. GUI should plot values over time, where values can be joint angles, poses of objects, forces on objects, diagnostic signals, and values from topics.
1. Up to four values per plot is allowed.
1. Multiple plots should be supported.

### Architecture
Include a system architecture diagram.
This should be a conceptual diagram that describes what components of OpnSim will be utilized or changed, the flow of information, new classes, etc.

### Interfaces
Describe any new interfaces or modifications to interfaces, function API changes, SDF changes, and GUI changes. These changes can be notional. Describe if/how/when OpenSim users/developers will be affected.

For example:
Plot proto message: A message that carries plot data will be created to transmit data from the server to the client.

Include any UX design drawings.

#### Bindings
Describe implications to Python and Java bindings.

### Backwards Compatibility
Describe implications on backwards compatibility. Particularly if/how/when code written to previous versions of OpenSim will break and how are they addressed going forward.

### Lifecycle and Ownership
Describe the intended lifecycle of new objects.
Based on the lifecycle, suggest an ownership model.
These characteristics should motivate the types of pointers, smart pointers,
and/or references that are used in the interfaces.

### Performance Considerations
Will this project cause changes to performance?
If so, describe how.
One or more performance tests may be required.

### Tests
List and describe the tests that will be created. For example:

1. Test: Plot View
    1. case: Plot window should appear when signaled by QT.
    1. case: Plot simulation time should produce correct results when save to CSV
    1. case: Signalling a close should close the plotting window.
1. Test: Multiple plots
    1. case: Create two plots with identical data. Saved CSV data from each should be identical

### Pull Requests
List and describe the pull requests that will be created to merge this project.
Consider separating large refactoring operations from additions of new code.

Keep in mind that smaller, atomic pull requests are easier to review.
