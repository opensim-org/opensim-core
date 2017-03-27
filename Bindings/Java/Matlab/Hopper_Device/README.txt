MATLAB Example: *Hopper Device*
===============================

This is the MATLAB version of the Hopper Device example. The example
demonstrates some of the new features of the OpenSim 4.0 API. The Component
architecture allows us to join sub-assemblies to form larger Models, with
information flowing between Components via Inputs, Outputs, and Sockets.
For more information, please refer to the OpenSim doxygen documentation for the
Component class.

This example consists of three steps, each having its own MATLAB script:

1. **RunHopper.m**: Build and simulate a single-legged hopping mechanism.
2. **RunHopperWithDevice.m**: Connect the device to the hopper to increase jump
   height.
3. TODO optimization.

Here is a brief description of the other files in this example:

BuildHopper.m: Builds an OpenSim Model of a single-legged hopper, actuated
               by a single muscle.
BuildDevice.m: Builds an OpenSim Model of an assistive device, which can be
               attached to the hopper.
Simulate.m: Runs a forward simulation of an OpenSim model from a given State.




