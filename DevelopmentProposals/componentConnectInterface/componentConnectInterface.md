## Project: Improved interface for connecting Outputs to Inputs
***OpenSim Development Proposal***

### Overview
The interface for connecting Outputs to Inputs currently looks like the following:
```cpp
// "update this Input to my ConsoleReporter and connect it to this Output from my Model"

// Use case 1.
reporter->updInput("inputs").connect(hopper.getComponent(hopperHeightCoord).getOutput("value"), "height");

// Use case 2.
auto& input  = reporter->updInput("inputs");
auto& output = hopper.getComponent(hopperHeightCoord).getOutput("value");
input.connect(output, "height");
```

Some users may find this design difficult to learn for two reasons:
- In Use case 1, the verb "connect" appears in the middle of a potentially long line.
- The verb "connect" does not imply directionality, yet `connect()` can be called only in one direction: `input.connect(output)`.

This project proposes to change the interface for connecting Outputs to Inputs while retaining all current functionality.
See [issue #1118](https://github.com/opensim-org/opensim-core/issues/1118).

### Requirements
- No change to what can be connected or how the underlying connections are implemented.
- No redundancy in the interface (i.e., the new interface replaces the existing interface).

### Architecture and Interfaces
There are several possible designs.

**Design 1**

Create convenience methods to simplify the code that precedes the word `connect`.
Where applicable, make the existing `connect()` method private.
```cpp
// Current design (for reference).
const auto& output = hopper.getComponent(hopperHeightCoord).getOutput("value");
reporter->updInput("inputs").connect(output, "height");

// Alternative designs (mutually exclusive).
reporter->connect("inputs", output, "height");  //of the form [register] = [expression]
reporter->connect(output, "inputs", "height");  //mirrors left-to-right signal flow diagram
// Specific to Reporters, which have only one Input.
reporter->connectToInput(output, "height");     //hides the Reporter's Input
reporter->satisfyInput(output, "height");       //avoids the overloaded verb "connect"
reporter->addToReport(output, "height");        //might match user psychology
```

**Design 2**

Create a new static method on Component for connecting Outputs to Inputs.
```cpp
// Current design (for reference).
const auto& output = hopper.getComponent(hopperHeightCoord).getOutput("value");
reporter->updInput("inputs").connect(output, "height");

// Alternative designs (mutually exclusive).
// Would rename the existing connect() method to, e.g., finalizeConnections().
Component::connect(reporter->output, updInput("inputs"), "height");  //not both
Component::connect(reporter->updInput("inputs"), output, "height");  //of these
Component::connect(reporter->updInput(), output, "height");
Component::connect(reporter, output, "height");
Component::satisfyInput(reporter->updInput(), output, "height");
```

**Design 3 (unable to wrap)**

Overload `operator<<` or `operator>>`.
```cpp
// Current design (for reference).
const auto& output = hopper.getComponent(hopperHeightCoord).getOutput("value");
reporter->updInput("inputs").connect(output, "height");

// Alternative designs.
output >> reporter->updInput();  //would need a separate setAnnotation() method
reporter->updInput() << output;
reporter << output << annotation;
```

**Design 4**

Create a `Connection` Component that is analogous to the wires between the Output and Input.
```cpp
const auto& output = hopper.getComponent(hopperHeightCoord).getOutput("value");
auto& rainbow = Connection(reporter->updInput(), output, "height");

// Opens up several possibilities.
rainbow.setProducer(hopper.getComponent(vastus).getOutput("activation"));
rainbow.setConsumer(reporter2);
rainbow.setAnnotation("Hi ho!");
```

### Bindings
No anticipated issues for Designs 1 or 2.

### Backwards Compatibility
No backwards compatibility concerns because Components are new to 4.0.

### Lifecycle and Ownership / Memory Management
No new objects.

### Performance Considerations
No expected performance issues because the same methods will be called under the hood.

### Tests
The necessary updates to testComponentInterface.cpp will retain the current test coverage for the existing `connect()` method.

### Potential hurdles or roadblocks
No anticipated issues.

### Pull Requests
A single PR should be sufficient.
