## Project: Address problem/bug with nested loops using `ComponentListIterator`

### Overview
The issue is detailed in issue [#1159](https://github.com/opensim-org/opensim-core/issues/1159).

### Requirements
* No change the existing interface of `ComponentList`, `ComponentListIterator`, `Component`.
* Current state of tests in `testIterators` should not change. Meaning existing tests should run/pass and produce exactly the same output before & after the change.
* Introduce a test into `testIterator` to confirm the fix/solution.

### Problem
The problem seems to be with the handling of `_nextComponent` data member of `Component`. For every component, it is meant to keep track of the next component to be traversed in the pre-order traversal of the component tree. When for loops are nested as shown in the issue [#1159](https://github.com/opensim-org/opensim-core/issues/1159), `_nextComponent` incorrectly becomes `nullptr` at the inner for-loop, resulting in `ComponentListIterator` finding a `nullptr` in place of next component during outer for-loop. `ComponentListIterator` terminates the loop prematurely.

### Solution
The solution is to edit the function `Component::initComponentTreeTraversal` to make sure `_nextComponent` is not set to `nullptr` for the root of the tree. Refer to the changes made to sources in this PR for details.

### Architecture
No change. 

### Interfaces
No change.

#### Bindings
No implications to bindings.

### Backwards Compatibility
No breaking change.

### Performance Considerations
No change to performance expected.

### Tests
The test introduced will be a nested for loop and assertion of number of iterations expected in the two loops.

### Pull Requests
This proposal is part of the pull request that provides the fix as well.
