## Project: Improved interface for connecting Components
***OpenSim Development Proposal***

### Overview
The interface for connecting Components currently looks something like the following:
- "update this Input to my ConsoleReporter and connect it to this Output from my Model"
  (e.g., [`reporter->updInput("inputs").connect(hopper.getComponent(hopperHeightCoord).getOutput("value"), "height");`]
  (https://github.com/opensim-org/opensim-core/blob/master/OpenSim/Sandbox/ExampleHopperDevice/exampleHopperDevice_answers.cpp#L159))
- "update this Connector to my Component and connect it to this object"
  (e.g., [`controller->updConnector("actuator").connect(*pathActuator);`]
  (https://github.com/opensim-org/opensim-core/blob/master/OpenSim/Sandbox/ExampleHopperDevice/buildDeviceModel_answers.cpp#L127))

Some users may find this design difficult to learn for two reasons:
- The verb "connect" appears in the middle of a potentially long line, not the beginning where it would be more visible.
- The verb "connect" does not imply directionality, yet `connect()` can be called only in one direction: `input.connect(output)`.

This project proposes to change the interface for connecting Components while retaining all current functionality.
See [issue #1118](https://github.com/opensim-org/opensim-core/issues/1118).

### Requirements
- No change to what can be connected or how the underlying connections are implemented.
- No redundancy in the interface (i.e., the new interface replaces the existing interface).

### Architecture
**Change to `Component`**
Create new methods for forming connections between Components.
```cpp
// 1. Plugging an AbstractOutput into an AbstractInput.
//    -> calls Input::connect(const AbstractOutput& output, const std::string& annotation = "")
static void connectToInput(AbstractInput& input, const AbstractOutput& output, const std::string& annotation = "");

// 2. Plugging an AbstractChannel into an AbstractInput.
//    -> calls Input::connect(const AbstractChannel& channel, const std::string& annotation = "")
static void connectToInput(AbstractInput& input, const AbstractChannel& channel, const std::string& annotation = "");

// 3. Plugging an Object into a Connector.
//    -> calls Connector::connect(const Object& object)
static void connectToConnector(Connector& connector, const Object& object);
```

**Change to `Component`**
Rename the existing `void connect(Component& root)` method to `finalizeConnections()`.

**Changes to `AbstractConnector`, `Connector`, `AbstractInput`, and `Input`**
Move the existing `connect()` methods from public to protected.
(Note that `Component` is already a friend of `AbstractConnector`.)

### Interfaces
Example from hopper:
```cpp
// Before
reporter->updInput("inputs").connect(device.getOutput(outputName));
// After
Component::connectToInput(reporter->updInput("inputs"), device.getOutput(outputName));
```

### Bindings
No anticipated issues.

### Backwards Compatibility
No backwards compatibility concerns because Components are new to 4.0.

### Lifecycle and Ownership / Memory Management
No new objects; only adding static methods to an existing class.

### Performance Considerations
No expected performance issues because the same methods will be called under the hood.

### Tests
The necessary updates to testComponentInterface.cpp will retain the current test coverage for the existing `connect()` method.

### Potential hurdles or roadblocks
No anticipated issues.

### Pull Requests
A single PR should suffice.
