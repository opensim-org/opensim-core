/* -------------------------------------------------------------------------- *
 *                     OpenSim:  exampleHopperDevice.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth,             *
 *            Thomas Uchida                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


#include <OpenSim/OpenSim.h>
#include "helperMethods.h"
#include "StretchController.h"
#include "Spindle.h"

static const double SIGNAL_GEN_CONSTANT{ 0.33 };
static const double REPORTING_INTERVAL{ 0.2 };
static const double LENGTH_GAIN = 1.0;
static const std::string testbedAttachment1{"ground"};
static const std::string testbedAttachment2{"load"};
using namespace std;
namespace OpenSim {

// Forward declarations for methods used below.
Model buildTestbed();   //defined in defineStretchController.h

//------------------------------------------------------------------------------
// Build the StretchController.
//------------------------------------------------------------------------------

StretchController* buildStretchController(Model& model, PathActuator& pa) {
	try {
		cout << "building controller with pathactuator " << pa.getName() << endl;
		
		auto spindle = new Spindle();
		spindle->setName(pa.getName() + "_spindle");
		spindle->set_delay_time(2);
		spindle->updInput("fiberLength").connect(pa.getGeometryPath().getOutput("length"));
		model.addComponent(spindle);

		auto controller = new StretchController();
		controller->setName("stretch_controller");
		controller->set_length_gain(LENGTH_GAIN);

		//controller->updInput("length").connect(pa.getGeometryPath().getOutput("length"));
		//controller->set_delay_time(1.0);

		controller->updInput("length").connect(spindle->getOutput("length"));
		controller->updConnector("actuator").connect(pa); 
		model.addComponent(controller);

		return controller;
	}
	catch (Exception e) {
		cout << e.what() << endl;
	}
}

Controller* buildStretchController(const Muscle& m) {
	try {
		cout << "building controller for muscle " << m.getName() << endl;
		auto controller = new StretchController();
		controller->setName("stretchCon");
		controller->set_length_gain(LENGTH_GAIN);

		controller->updInput("length").connect(m.getOutput("fiber_length"));
		controller->updConnector("actuator").connect(m);
		return controller;
	}
	catch (Exception e) {
		cout << e.what() << endl;
	}
}

//------------------------------------------------------------------------------
// Add a SignalGenerator to a StretchController.
//------------------------------------------------------------------------------
void addSignalGeneratorToController(Controller& controller, double length_setpoint)
{
    auto lengthSignalGen = new SignalGenerator();
	lengthSignalGen->setName("lengthSetPointGen");

    // Try changing the constant value and/or the function (e.g., try a
    // LinearFunction).
	lengthSignalGen->set_function(Constant(length_setpoint));
	controller.addComponent(lengthSignalGen);
	
	// Connect the signal generator's output signal to the controller's
    // setpoint 
	controller.updInput("length_setpoint")
        .connect(lengthSignalGen->getOutput("signal"));
} 


//------------------------------------------------------------------------------
// Add a ConsoleReporter to a model for displaying outputs from a device.
//------------------------------------------------------------------------------
void addDeviceConsoleReporterToModel(Model& model, Controller& controller,
    const std::vector<std::string>& deviceOutputs)
{
    // Create a new ConsoleReporter. Set its name and reporting interval.
    auto reporter = new ConsoleReporter();
    reporter->setName(model.getName() + "_" + controller.getName() + "_results");
    reporter->set_report_time_interval(REPORTING_INTERVAL);

    // Loop through the desired device outputs and add them to the reporter.
    for (auto thisOutputName : deviceOutputs)
        reporter->updInput("inputs").connect(controller.getOutput(thisOutputName));

    // Add the reporter to the model.
    model.addComponent(reporter);
}

} // namespace OpenSim


//------------------------------------------------------------------------------
// START HERE! Toggle "if (false)" to "if (true)" to enable/disable each step in
// the exercise. The project should execute without making any changes (you
// should see the unassisted hopper hop slightly).
//------------------------------------------------------------------------------
int main()
{
	try {
    using namespace OpenSim;
	using namespace std;
    //==========================================================================
    // Step 1. Build an stretch controller and test it on a simple testbed.
    //==========================================================================
    if (true){
		// Build the testbed and controller.
		cout << "starting" << endl;
		auto& testbed = buildTestbed();
		cout << "model created" << endl;
		
		auto& pathActuator = testbed.updComponent<PathActuator>("muscle");
		auto controller = buildStretchController(testbed, pathActuator);
		
		cout << "model and controller created" << endl;

		// Use a SignalGenerator to create a set point signal for testing the
		// controller. ```
		addSignalGeneratorToController(*controller, 0.3);// pathActuator.getLength());
		cout << "generator added" << endl;

		// Show all Components in the testbed.
		showSubcomponentInfo(testbed);
		showSubcomponentInfo(*controller);

		// List the device outputs we wish to display during the simulation.
		std::vector<std::string> controllerOutputs{ "stretch_control"};

		// Add a ConsoleReporter to report deviceOutputs.
		//addDeviceConsoleReporterToModel(testbed, *controller, controllerOutputs);

		// Create a new ConsoleReporter. Set its name and reporting interval.
		auto reporter = new ConsoleReporter();
		reporter->setName(testbed.getName() + "_" + controller->getName() + "_results");
		reporter->set_report_time_interval(REPORTING_INTERVAL);
		reporter->updInput("inputs").connect(controller->getOutput("stretch_control"));
		reporter->updInput("inputs").connect(pathActuator.getOutput("geometrypath_/length"));

		// Add the reporter to the model.
		testbed.addComponent(reporter);
		
		// Create the system, initialize the state, and simulate.
		SimTK::State& sDev = testbed.initSystem();
		
		OpenSim::simulate(testbed, sDev);
    }
	

}
catch (const std::exception& e) {
	std::cout << e.what() << std::endl;
}
system("pause");
return 0;

};
