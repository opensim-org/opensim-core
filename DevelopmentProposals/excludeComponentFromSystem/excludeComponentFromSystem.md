## Proposal: Exclude Components from the underlying Computational System
***OpenSim Development Proposal***

### Overview
This proposal describes how OpenSim Component's can be excluded from computations and thereby replace 
the adhoc mechanisms in place that attempt to "disable" certain components, like Forces and Constraints. 
The adhoc nature of the current mechanism puts the burden of "disabling" a Component on the Component
writer and results in inconsistent behaviours, including partially disabling by not applying the force of a Muscle, for example, to the the MultibodySystem, but still computing the force or other costly values such as its length.
The overall desired behaviour is that a Component that is disabled (excluded) is excluded from any 
downstream computations and is effectively removed from the Model and resulting System.

For background, see:
- [issue #845](https://github.com/opensim-org/opensim-core/issues/845)
- [issue #1111](https://github.com/opensim-org/opensim-core/issues/1111)
- [issue #1358](https://github.com/opensim-org/opensim-core/issues/1358)

**Current Behaviour**

Several Components, namely Forces, Constraints and Controllers, can be disabled with the expectation that disabled means they no longer effect the computations and dynamics of the System. This is not the case. While an OpenSim `Force::computeForce(s)` is not invoked if a Force is disabled, any and all of its `realize` methods remain and are apart of the realization of the System. That means that position and velocity calculations are available for invocation and the onus is on the Force writer to check that the Force is disabled to skip those evaluations. In many case, such as Muscle path length calculations and drawing of the path these are likely to have unexpected consequences on a Model (System) where the disable Component is expected to have no effect.

### Requirements
- Exclusion means the Component is not present in computational System in any shape or form. It has absolutely no effect, direct or otherwise, on the System.
- Any Component can be excluded from the Model's System.
- Exclusion of a Component means the exclusion of all its subcomponents.
- Exclusion status is a property NOT a discrete state variable.
- The only way to change the exclusion status is to update the associated property (flag) and triggering  `finalizeFromPropeties()`  
- Exclusion status overrides existing disabling behaviour. In other words, if a Component is excluded, the status of an existing disabled flag is irrelevant, since the Component for all intents and purposes does not exist. If the Component is not excluded
 
### Architecture
The design proposes adding the ability to exclude Components independent of whether or not they can be disabled.
These are the proposed changes:

1. add an `excluded` flag as a property to `Component` with a default value of `false`
2. the inclusion of a Component property as a subcomponent becomes contingent on the flag being `false` and if true it is not included as a subcomponent and it can no longer receive delegated Component interface calls to `connect()`, `addToSystem()`, `realize*()`, etc...  
3. Component::getComponentList<C> is devoid of excluded components and does not merely "skip-over" excluded Components.

### Interfaces
The added `excluded` property results in property related methods being added to the `Component` interface, such as: `get_excluded()`, `upd_excluded`, `getProperty_excluded()`, ...

### Bindings and GUI
There are no special implications for Python and Java bindings. A Component that is excluded is still accessible via Property accessors so its exclusion status can be changed. It is imperative then that the GUI provide a list of ALL components (including excluded Components) and cannot rely on the ComponentList iterator to achieve this. 

### Backwards Compatibility
The ConsoleReporter and TableReporter classes are new to 4.0, so there are no backwards compatibility issues.

### Lifecycle and Ownership / Memory Management
No new objects are being proposed.

### Performance Considerations
No significant changes in performance are expected.

### Required Tests
Additional test cases will be added to verify that:
1. an excluded Component is inaccessible from its parent (owning) Component and Model. 
2. excluding Components never increases the number of computations to realize the Model from `Time` to `Report`.
3. re-including a Component(setting excluded to false)  and finalizing from properties
4. verify that Tool performances with excluded Constraints, Forces and Controllers yields the same results as when they were disabled.

### Potential hurdles or roadblocks
Exclusion and disabling will coexist (at least initially) and we should decide in which use cases does the user/test case intend to exclude the Component altogether in the analysis or whether the Component was intentionally and temporarily being disabled as part of a more complex analysis (e.g. InducedAccelerationAnalysis disables all Forces and them enables them one by one to compute its contribute to system acceleration) in which case to exclude would be incorrect usage).


### Pull Requests
A single PR should be necessary to implement the proposal, however a subsequent PR may be required 
to improve error messages (e.g. excluding the pelvis from a gait model will fail when attempting to establish valid connections for hip joints, etc...)
