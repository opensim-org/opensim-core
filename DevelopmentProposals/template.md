## Project: TITLE
***OpenSim Development Proposal***

### Overview

Just a few sentences that describes the purpose of this project. Provide
motivation for the proposal by:
* referring to the existing
[Issues](https://github.com/opensim-org/opensim-core/issues) it addresses.
* describing the problem (not captured in existing issues) it solves.

For example:
*Add ability to read/write table of quaternions to/from STO files. This is a
new feature and does not directly address any existing issue.* 

### Requirements

List the set of requirements that this project must fulfill. Include all
constraints this proposal is working within.
If the list gets too long, consider splitting the project into multiple small
projects.

For example:
* Add support for reading and writing of `TimeSeriesTables` of `Quaternion`s
to/from STO files through the existing class `STOFileAdapter`.

### Architecture
Describe changes to the architecture and include a diagram if
applicable/helpful.
This should be a conceptual diagram that describes what components of OpenSim
will be utilized or changed, the flow of information, new classes, etc.

For example:
Existing classes `STOFileAdapter` and `DelimFileAdapter` will be edited by
adding template specializations. The existing dispatch mechanism will be
extended to include `Quaternion`. ....

### Interfaces
Describe any new interfaces or modifications to interfaces, function API
changes, SDF changes. These changes can be notional. Describe if/how/when
OpenSim users/developers will be affected.

For example:
There will no change to the existing interface. This proposal is to add ability
to read and write quaternion tables to files using existing interface.

#### Bindings
Describe implications for Python and Java bindings.

### Backwards Compatibility
Describe implications on backwards compatibility. Particularly if/how/when code
written to previous versions of OpenSim will break and how are they addressed
going forward.

For example:
This proposal intends to add a new feature that previously did not exist.

### Lifecycle and Ownership / Memory Management
Describe the intended lifecycle of new objects.
Based on the lifecycle, suggest an ownership model.
These characteristics should motivate the types of pointers, smart pointers,
and/or references that are used in the interfaces.

### Performance Considerations
Will this project cause changes to performance?
If so, describe how.
One or more performance tests may be required.

### Tests
List and describe the tests that will be created.

For example:
Programmatically create a few `TimeSeriesTable`s of `Quaternion`, write them to
files, read those files back in and verify their contents.

### Potential hurdles or roadblocks
List any known problems with this proposal. Any potential hurdles or roadblocks in moving forward this proposal.

### Pull Requests
List and describe the pull requests that will be created to merge this project.
Consider separating large refactoring operations from additions of new code.

Keep in mind that smaller, atomic pull requests are easier to review.
