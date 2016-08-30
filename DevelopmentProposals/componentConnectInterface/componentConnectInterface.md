## Project: Improved interface for connecting Components
***OpenSim Development Proposal***

### Overview
Connecting Components currently looks something like "update the Input of my ConsoleReporter and connect it to this Output from my Model".
Some users may find it more intuitive to (i) begin with the verb *connect*, and (ii) swap the order of the Input and Output.
This project proposes to change the user interface for connecting Components while retaining all current functionality.
See [issue #1118](https://github.com/opensim-org/opensim-core/issues/1118).

### Requirements
- No change to what can be connected or how the underlying connections are formed.
- No redundancy in the interface (i.e., the new interface replaces the existing interface).

### Architecture
Changes to `Component`:
- Create a new method for forming connections between Components. There will be several variants:
  - `static void connect(const Object& object, Component& comp)` and `static void connect(Component& comp, const Object& object)`,
    which will call [this method](https://github.com/opensim-org/opensim-core/blob/master/OpenSim/Common/ComponentConnector.h#L278).
  - `static void connect(const AbstractOutput& output, AbstractInput& input, const std::string& annotation = "")` and
    `static void connect(AbstractInput& input, const AbstractOutput& output, const std::string& annotation = "")`,
    which will call [this method](https://github.com/opensim-org/opensim-core/blob/master/OpenSim/Common/ComponentConnector.h#L494).
  - `static void connect(const AbstractChannel& channel, AbstractInput& input, const std::string& annotation = "")` and
    `static void connect(AbstractInput& input, const AbstractChannel& channel, const std::string& annotation = "")`,
    which will call [this method](https://github.com/opensim-org/opensim-core/blob/master/OpenSim/Common/ComponentConnector.h#L497).
- Change the name of the existing `void connect(Component& root)` method to `finalizeConnections()`.

Changes to `AbstractConnector`, `Connector`, `AbstractInput`, and `Input`:
- Move the existing `connect()` methods from public to protected.
  (Note that `Component` is already a friend of `AbstractConnector`.)

### Interfaces
Example from hopper:
```cpp
// before
reporter->updInput("inputs").connect(device.getOutput(outputName));
// after: either
Component::connect(reporter->updInput("inputs"), device.getOutput(outputName));
// or
Component::connect(device.getOutput(outputName), reporter->updInput("inputs"));
```

### Bindings
No anticipated issues.

### Backwards Compatibility
No backwards compatibility concerns because Components are new to 4.0.

### Lifecycle and Ownership / Memory Management
No new objects; only adding static methods to an existing class.

### Performance Considerations
No expected performance issues because the same methods will be called behind-the-scenes.

### Tests
Updates to testComponentInterface.cpp will retain current test coverage for the existing `connect()` method.
Will add tests to ensure `connect(a, b)` has the same effect as `connect(b, a)`.

### Potential hurdles or roadblocks
Agreeing on implementing either (a) `connect(in, out)`, (b) `connect(out, in)`, or (c) both.

### Pull Requests
A single PR should suffice.
