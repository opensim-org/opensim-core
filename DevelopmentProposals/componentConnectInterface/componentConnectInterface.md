## Project: Improved interface for connecting Components
***OpenSim Development Proposal***

### Overview
The interface for connecting Outputs to Inputs currently looks like the following:
```cpp
// "update this Input to my ConsoleReporter and connect it to this Output from my Model"

// Use Case 1.
reporter->updInput("inputs").connect(hopper.getComponent(hopperHeightCoord).getOutput("value"), "height");

// Use Case 2.
auto& input  = reporter->updInput("inputs");
auto& output = hopper.getComponent(hopperHeightCoord).getOutput("value");
input.connect(output, "height");
```

Connectors are connected similarly.
Some users may find this design difficult to learn for two reasons:
- In Use Case 1, the verb "connect" appears in the middle of a potentially long line.
- The verb "connect" does not imply directionality, yet `connect()` can be called only in one direction: `input.connect(output)`.
- The distinction between a Connector and an Input is potentially confusing.
  (*Connectors* express topological dependencies among Components; *Inputs and Outputs* express the flow of time-varying signals.)

This project proposes to change the interface for connecting Outputs to Inputs while retaining all current functionality.
Connectors are a secondary, but related, issue.
See [issue #1118](https://github.com/opensim-org/opensim-core/issues/1118).

### Requirements
- No change to what can be connected or how the underlying connections are implemented.
- No redundancy in the interface (i.e., the new interface replaces the existing interface).

### Architecture and Interfaces

**The current proposal involves four main changes:**

1. Rename Connector to Socket; avoids the awkward-sounding action of connecting connectors.
2. Rename existing `Component::connect()` method to `finalizeConnections()`.
   Aligns with `finalizeFromProperties()` and avoids confusion with other `connect()` methods.
3. Modify macros to automatically generate methods of the form `connectInput_[nameOfInput]()`.
4. Add convenience methods for common use cases, like `Reporter::addToReport()`.

Details and alternative designs are provided below.
Popular ideas from dev team meeting are indicated with `<-- LIKED` (see Designs 1 and 6).

**Design 1**

Create convenience methods to simplify the code that precedes the word `connect`.
Where applicable, make the existing `connect()` method private to avoid redundancy in the API.
```cpp
// Current design (for reference).
const auto& output = hopper.getComponent(hopperHeightCoord).getOutput("value");
reporter->updInput("inputs").connect(output, "height");

// Alternative designs (mutually exclusive).
reporter->connect("inputs", output, "height");  //of the form [register] = [expression]
reporter->connect(output, "inputs", "height");  //mirrors left-to-right signal flow diagram

// These methods are specific to Reporters, which have only one Input. For
// multi-input Components, the first argument would be the name of the Input.
reporter->connectToInput(output, "height");     //hides the Reporter's Input
reporter->connectInput(output, "height");       //unclear whether connecting to or from Input
reporter->satisfyInput(output, "height");       //avoids the overloaded verb "connect"
reporter->specifyInput(output, "height");
reporter->setInput(output, "height");           //conflicts with existing use of "set" prefix
                                                //  (where argument would be of type Input)
reporter->addToReport(output, "height");        //might match user psychology               <-- LIKED

// Other ideas from dev team meeting.
muscle.getOutput("activation").wireTo(component.getInput("activationIn"));
muscle.getInput("activationIn").wireFrom(component.getOutput("activation"));
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

**Design 3**

Create a free `connect()` function; similar to Design 2 but without the `Component::`.
```cpp
using namespace OpenSim;
connect(reporter->updInput(), output, "height");
```

**Design 4**

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

**Design 5**

Create a `Connection` Component that is analogous to the wires between the Output and Input.
```cpp
const auto& output = hopper.getComponent(hopperHeightCoord).getOutput("value");
auto& rainbow = Connection(reporter->updInput(), output, "height");

// Opens up several possibilities.
rainbow.setProducer(hopper.getComponent(vastus).getOutput("activation"));
rainbow.setConsumer(reporter2);
rainbow.setAnnotation("Hi ho!");
myComponent.getConnections();
```

**Design 6**

Rename `Connector` to `Socket` and use `connect()` for both Sockets and Inputs.
Sockets and Inputs can be accessed using methods generated through the property system (e.g., `upd_sockets()`)
as well as macro-generated methods that bake the name of the input into the method name (e.g., `connectInput_activation()`).
```cpp
myComponent.upd_sockets("actuator").connect(*pathActuator);                      // <-- LIKED
// Equivalently:
myComponent.upd_sockets(0).connect(*pathActuator);
myComponent.upd_sockets().get(0).connect(*pathActuator);
myComponent.connectSocket_actuator(*pathActuator);                               // <-- LIKED

myComponent.upd_inputs("activation").connect( vastus.getOutput("activation") );  // <-- LIKED
// Equivalently:
myComponent.connectInput_activation( vastus.getOutput("activation") );           // <-- LIKED
myComponent.wireInput_activation( vastus.getOutput("activation") );              // <-- LIKED
myComponent.satisfyInput_activation( vastus.getOutput("activation") );
myComponent.setInput_activation( vastus.getOutput("activation") );
myComponent.appendInput_activation( vastus.getOutput("activation") );
myComponent.getInputs().get(0).connect( vastus.getOutput("activation") );        // <-- LIKED
```

### Bindings
Design 3 would require custom wrapping code in Bindings/Java/swig.
Design 4 cannot be wrapped.

### Backwards Compatibility
No backwards compatibility concerns because Components are new to 4.0.

### Lifecycle and Ownership / Memory Management
Design 5 introduces a `Connection` class, which would be owned by Model.

### Performance Considerations
No expected performance issues because the same methods will be called under the hood.

### Tests
The necessary updates to testComponentInterface.cpp will retain the current test coverage for the existing `connect()` method.

### Potential hurdles or roadblocks
No anticipated issues.

### Pull Requests
To be determined.
