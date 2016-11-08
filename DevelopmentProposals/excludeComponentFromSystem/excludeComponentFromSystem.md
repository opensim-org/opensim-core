## Proposal: Add a Feature to Exclude Components from the underlying Computational System

***OpenSim Development Proposal***

### The Need:
Models are complicated and the Component paradigm will permit the composition of ever more complex models and behaviours. Modeller have to troubleshoot and users must test these models, which often requires isolating a problematic component. The easiest way to isolate a problematic Component's effect is to effectively remove it from the Model and see if the problem is alleviated. The current approach to this task is to *disable* the Component, which is applicable to a handful of Components (Constraints and Forces). Unfortunately, disabling does not equate to effectively removing the Component and the effect of disabling is inconsistent across Components.   

The purpose of this proposal is to enable the fundamental ability to exclude a Component from all computations. It is necessary to accurately assess a Component's impact on computation (timing, CPU usage) and results. Currently, users have little confidence that disabling a Component (if it has that option) effectively removes its calculations. 

### Overview
This proposal describes a simple mechanism to exclude OpenSim Components from all computations. It can, in some cases, replace the adhoc mechanisms to "disable" components (offered by Forces and Constraints). The nature of the current adhoc scheme places the burden of "disabling" a Component on the Component writer and results in inconsistent behaviours. For example, a disabled Muscle will not apply forces to the MultibodySystem, but there is now preventing it from still computing the muscle tension or other costly values such as its length. The objective of excluding a Component is to accurately assess what effect that Component is having on the System. Without being able to guarantee that a disabled Force will not perform calculations users have to resort to removing Forces via XML editing.

For background, see:
- [issue #845](https://github.com/opensim-org/opensim-core/issues/845)
- [issue #1111](https://github.com/opensim-org/opensim-core/issues/1111)
- [issue #1358](https://github.com/opensim-org/opensim-core/issues/1358)

**Current Behaviour**
There is currently no method of excluding a Component from the underlying System short of editing the Model's XML file. Several Components, namely Forces, Constraints and Controllers, can be disabled with the expectation that disabled will  result in no computations of that Component in the System. This is not currently enforceable. While an OpenSim `Force::computeForce(s)` is not invoked if a Force is *disabled*, any and all of its `realize` methods remain a part of the realization of the System. That means that Position, Velocity, ... (includes reporting) realizations are available for invocation.  The onus is on the Force writer to check that the Force is disabled to skip those evaluations. In many cases, such as Muscle path length calculations and drawing of the GeometryPath, these are likely to have unexpected consequences/computations on a Model (System) where one may expect the disabled Component to have no effect.

### Requirements
- Users require a robust method of excluding components without editing XML to manually remove the Component.
- Exclusion must entail that the Component is not present in the computational System in any shape or form. It must have absolutely no effect, direct or otherwise, on the System.
- Any Component can be excluded from the Model's System.
- Exclusion of a Component means the exclusion of all its subcomponents.
- Exclusion status is a property NOT a discrete state variable.
- The only way to change the exclusion status is to update the associated property (flag) and invoke  `finalizeFromProperties()`  
- Exclusion status overrides any existing disabling behaviour. In other words, if a Component is excluded, the status of an existing disabled flag is irrelevant, since the Component for all intents and purposes does not exist. If the Component is not excluded, a disabled Constraint or Force will ONLY have its dynamic contribution ignored and will have no guarantees on other computations.
 
### Architecture
The design proposes adding the ability to exclude Components independent of whether or not it can be disabled.
These are the main changes:

1. add an `excluded` flag (bool) as a property to `Component` with a default value of `false`
2. the inclusion of a Component property as a subcomponent becomes contingent on the flag being `false` and if true it is not included as a subcomponent and it can no longer receive delegated Component interface calls to `connect()`, `addToSystem()`, `realize*()`, etc...  
3. modify `Component::getComponentList<C>()` to ensure it is devoid of excluded components and their subcomponents. This is necessary so that users and Component aggregators (e.g. Controller, Model) do not accidentally access excluded Components. 
4. add `Component::get/updExcludedComponentList<C>()` get access to excluded components. This is necessary to access and update its flag to re-include excluded Components.

### Interfaces
No significant implications to the Component interface other than additional accessors to the `excluded` property and to access the list of excluded subcomponents. 

### Bindings and GUI
Need to wrap `Component::get/updExcludedComponentList<C>` to enable scripting access to excluded Components.  

A Component that is excluded is still accessible via Property accessors so its exclusion status can be changed. It will be imperative that the GUI can provide a list of ALL components (including excluded Components) so users can toggle the Components exclusion status. 

### Backwards Compatibility
The ConsoleReporter and TableReporter classes are new to 4.0, so there are no backwards compatibility issues.

### Lifecycle and Ownership / Memory Management
No new classes are being proposed.

### Performance Considerations
No significant changes in performance are expected.

### Required Tests
Additional test cases will be added to verify that:
1. an excluded Component is not accessible via its parent (owning) Component (or Model) getComponentList<C>(). 
2. excluding Components should provide identical performance (timing and results) as manually removing the Components from XML.
3. re-including a Component(setting `excluded` to false) and finalizing from properties restores the Component as part of the System and reproduces pre-exclusion results
4. Tool performances with excluded Constraints, Forces and Controllers yield the same results as when these Components were disabled.

### Potential hurdles or roadblocks
Exclusion and disabling will coexist (at least initially) and we should decide in which use cases does the user/test case intend to exclude the Component altogether or whether the Component was intentionally and temporarily being disabled as part of a more complex analysis. For examples, InducedAccelerationAnalysis disables all Forces and then enables them one by one to compute its contribute to system acceleration, in which case to exclude would be incorrect usage of the feature.

### Pull Requests
A single PR should be sufficient to implement this proposal, however a subsequent PR may be required 
to improve error messages to inform users of expected failures. For example, if one excluded the pelvis from a gait model it will fail to load due to invalid connections for hip joints, etc... and those messages should be properly relayed/presented to the user.

