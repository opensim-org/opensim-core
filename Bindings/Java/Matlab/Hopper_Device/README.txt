MATLAB Example: *Hopper Device*
===============================

This is the MATLAB version of the Hopper Device C++ example. The example
demonstrates some of the new features of the OpenSim 4.0 API. The Component
architecture allows us to join sub-assemblies to form larger Models, with
information flowing between Components via Inputs, Outputs, and Sockets.
For more information, please refer to the OpenSim doxygen documentation for the
Component class.

This example consists of three steps, each having its own MATLAB script:

1. **RunHopperOnly.m**: Build and simulate a single-legged hopping mechanism.
2. **RunDeviceTestbed.m**: Build an assistive device and test it on a testbed.
3. **RunHopperWithDevice.m**: Connect the device to the hopper to increase jump
   height.

Here is a brief description of the other files in this example:

BuildHopperModel.m: Builds an OpenSim Model of a single-legged hopper, actuated
                    by a single muscle.
BuildTestbedModel.m: Builds an OpenSim Model of a simple testbed, to which the
                     device can be attached for troubleshooting and design.
BuildDevice.m: Builds an OpenSim Model of an assistive device, which can be
               attached to the hopper or testbed models.
ConnectDeviceToModel.m: Attach the device to either the testbed or hopper.
AddSignalGeneratorToDevice.m: Provides the input signal that the device's
                              controller requires; for use with the testbed.
Simulate.m: Runs a forward simulation of an OpenSim model from a given State.




