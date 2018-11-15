MATLAB Example: *Hopper Device*
===============================

NOTE: This example only works with MATLAB versions R2014b and later.

This is the MATLAB version of the Hopper Device example (there is also a C++
version). The example demonstrates some of the new features of the OpenSim 4.0
API. The Component architecture allows us to join sub-assemblies using Sockets
to form larger Models. Additionally, Components can pass information between
each other using Inputs and Outputs. For more information, please refer to the
Doxygen documentation.

This example consists of three steps, each having its own MATLAB script:

1. **RunHopper.m**: Build and simulate a single-legged hopping mechanism.
2. **RunHopperWithDevice.m**: Connect the device to the hopper to increase jump
   height.
3. **InteractiveHopper.m**: A MATLAB GUI for optimizing jump height with a
   device. Run "InteractiveHopper" in the MATLAB Command Window to launch the
   GUI.

Here is a brief description of the other files in this example:

BuildHopper.m: Builds an OpenSim Model of a single-legged hopper, actuated
    by a single muscle.
BuildDevice.m: Builds an OpenSim Model of an assistive device, which can be
    attached to the hopper.
EvaluateHopper.m: Computes the jump height from a provided simulation.

RunHopper_answers.m: The same as RunHopper.m but with all "TODO" blocks
    filled in.
RunHopperWithDevice_answers.m: The same as RunHopperWithDevice.m but with all
    "TODO" blocks filled in.

The following files are used by the InteractiveHopper GUI:
  - BuildInteractiveHopperSolution.m
  - InteractiveHopperParameters.m
  - InteractiveHopperSettings.m

