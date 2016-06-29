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

static const double SIGNAL_GEN_CONSTANT{ 0.33 };
static const double REPORTING_INTERVAL{ 0.2 };
static const double LENGTH_GAIN = 1.0;
static const std::string testbedAttachment1{"ground"};
static const std::string testbedAttachment2{"load"};
using namespace std;

namespace OpenSim {

// Forward declarations for methods used below.
Model buildTugofwarModel();   //defined in buildTugofwarModel.cpp

//------------------------------------------------------------------------------
// Build the StretchController.
//------------------------------------------------------------------------------

Controller* buildStretchController(PathActuator& pa) {
	try {
		cout << "building controller" << endl;
		auto controller = new StretchController();
		controller->setName("stretchCon");
		controller->set_length_gain(LENGTH_GAIN);

		Muscle* mus = dynamic_cast <Muscle*> (&pa);
		controller->updInput("fiberLength").connect(mus->getOutput("fiber_length"));   // TODO connect this to what?
		controller->updConnector("actuator").connect(pa); // TODO need pointer to muscle!
		return controller;
	}
	catch (Exception e) {
		cout << e.what() << endl;
	}
}


//------------------------------------------------------------------------------
// Attach the stretch controller to a model
//------------------------------------------------------------------------------
void connectControllerToModel(Controller& controller, Model& model)
{
	model.addController(&controller);
}

//------------------------------------------------------------------------------
// Add a SignalGenerator to a StretchController.
//------------------------------------------------------------------------------
void addSignalGeneratorToController(Controller& controller)
{
    auto lengthSignalGen = new SignalGenerator();
	lengthSignalGen->setName("lengthSetPointGen");

    // Try changing the constant value and/or the function (e.g., try a
    // LinearFunction).
	lengthSignalGen->set_function(Constant(SIGNAL_GEN_CONSTANT));
	controller.addComponent(lengthSignalGen);
	
	// Connect the signal generator's output signal to the controller's
    // setpoint 
	controller.updInput("fiberLength_setpoint")
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


Model buildTugofwarModel()
{
	using namespace SimTK;
	using namespace std;

	// Create a new OpenSim model on earth.

	clock_t startTime = clock();

	try {
		//////////////////////
		// MODEL PARAMETERS //
		//////////////////////

		// Specify body mass of a 20 kg, 0.1m sides of cubed block body
		double blockMass = 20.0, blockSideLength = 0.1;

		// Constant distance of constraint to limit the block's motion
		double constantDistance = 0.2;

		// Contact parameters
		double stiffness = 1.0e7, dissipation = 0.1, friction = 0.2, viscosity = 0.01;

		///////////////////////////////////////////
		// DEFINE BODIES AND JOINTS OF THE MODEL //
		///////////////////////////////////////////

		// Create an OpenSim model and set its name
		auto osimModel = Model();
		osimModel.setName("tugOfWar");

		// GROUND FRAME
		// Get a reference to the model's ground frame
		Ground& ground = osimModel.updGround();
		// Create Frames to attach Geometry to
		// Left brick
		OpenSim::PhysicalFrame* leftAnchorFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(0, 0.05, 0.35)));
		leftAnchorFrame->setName("LeftAnchor");
		osimModel.addFrame(leftAnchorFrame);
		// Right brick
		OpenSim::PhysicalFrame* rightAnchorFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(0, 0.05, -0.35)));
		rightAnchorFrame->setName("RightAnchor");
		osimModel.addFrame(rightAnchorFrame);
		// Cylinder
		OpenSim::PhysicalFrame* cylFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(-.2, 0.0, 0.)));
		cylFrame->setName("CylAnchor");
		osimModel.addFrame(cylFrame);
		// Ellipsoid
		OpenSim::PhysicalFrame* ellipsoidFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(-.6, 0.6, 0.)));
		ellipsoidFrame->setName("EllipsoidAnchor");
		osimModel.addFrame(ellipsoidFrame);


		// Add display geometry to the ground to visualize in the Visualizer and GUI
		// add a checkered floor
		ground.attachGeometry(new Mesh("checkered_floor.vtp"));
		// add anchors for the muscles to be fixed to
		Brick* leftAnchorGeometry = new Brick(SimTK::Vec3(0.05, 0.05, 0.05));
		leftAnchorGeometry->upd_Appearance().set_color(SimTK::Vec3(0.0, 1.0, 0.0));
		Brick* rightAnchorGeometry = new Brick(SimTK::Vec3(0.05, 0.05, 0.05));
		rightAnchorGeometry->upd_Appearance().set_color(SimTK::Vec3(1.0, 1.0, 0.0));
		rightAnchorGeometry->upd_Appearance().set_opacity(0.5);

		// block is 0.1 by 0.1 by 0.1m cube and centered at origin. 
		// transform anchors to be placed at the two extremes of the sliding block (to come)

		// scale the anchors
		leftAnchorGeometry->set_scale_factors(Vec3(5, 1, 1));
		rightAnchorGeometry->set_scale_factors(Vec3(5, 1, 1));
		// position the anchors
		leftAnchorFrame->attachGeometry(leftAnchorGeometry);
		rightAnchorFrame->attachGeometry(rightAnchorGeometry);

		Geometry* cylGeometry = new Cylinder(0.2, .3);
		cylGeometry->upd_Appearance().set_representation(VisualRepresentation::DrawWireframe);
		cylFrame->attachGeometry(cylGeometry);

		Geometry* ellipsoidGeometry = new Ellipsoid(0.2, .7, .5);
		ellipsoidGeometry->setColor(SimTK::Vec3(1.0, .5, 0.1));
		ellipsoidFrame->attachGeometry(ellipsoidGeometry);

		// BLOCK BODY
		Vec3 blockMassCenter(0);
		Inertia blockInertia =
			blockMass*Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

		// Create a new block body with the specified properties
		OpenSim::Body *block = new OpenSim::Body("block", blockMass, blockMassCenter, blockInertia);

		// Add display geometry to the block to visualize in the GUI
		block->attachGeometry(new Mesh("block.vtp"));

		// Use attachGeometry to set frame name & addGeometry
		block->attachGeometry(new Sphere(0.1));

		// FREE JOINT

		// Create a new free joint with 6 degrees-of-freedom (coordinates) between the block and ground frames
		Vec3 locationInParent(0, blockSideLength / 2, 0), orientationInParent(0), locationInBody(0), orientationInBody(0);
		FreeJoint *blockToGround = new FreeJoint("blockToGround", ground, locationInParent, orientationInParent, *block, locationInBody, orientationInBody);

		// Get a reference to the coordinate set (6 degrees-of-freedom) between the block and ground frames
		CoordinateSet& jointCoordinateSet = blockToGround->upd_CoordinateSet();

		// Set the angle and position ranges for the coordinate set
		double angleRange[2] = { -SimTK::Pi / 2, SimTK::Pi / 2 };
		double positionRange[2] = { -1, 1 };
		jointCoordinateSet[0].setRange(angleRange);
		jointCoordinateSet[1].setRange(angleRange);
		jointCoordinateSet[2].setRange(angleRange);
		jointCoordinateSet[3].setRange(positionRange);
		jointCoordinateSet[4].setRange(positionRange);
		jointCoordinateSet[5].setRange(positionRange);

		// GRAVITY

		osimModel.setGravity(Vec3(0, -9.80665, 0));

		// Obtain the default acceleration due to gravity
		//Vec3 gravity = osimModel.getGravity();

		// Define non-zero default states for the free joint
		//jointCoordinateSet[3].setDefaultValue(constantDistance); // set x-translation value
		//double h_start = blockMass*gravity[1] / (stiffness*blockSideLength*blockSideLength);
		//jointCoordinateSet[4].setDefaultValue(h_start); // set y-translation which is height

		// Add the block and joint to the model
		osimModel.addBody(block);
		osimModel.addJoint(blockToGround);

		/////////////////////////////////////////////
		// DEFINE CONSTRAINTS IMPOSED ON THE MODEL //
		/////////////////////////////////////////////
		//Vec3 pointOnGround(0, blockSideLength / 2, 0);
		//Vec3 pointOnBlock(0, 0, 0);

		//// Create a new constant distance constraint
		//ConstantDistanceConstraint *constDist =
		//	new ConstantDistanceConstraint(ground,
		//		pointOnGround, *block, pointOnBlock, constantDistance);

		//// Add the new point on a line constraint to the model
		//osimModel.addConstraint(constDist);

		///////////////////////////////////////
		// DEFINE FORCES ACTING ON THE MODEL //
		///////////////////////////////////////

		// MUSCLE FORCES
		// Create two new muscles with identical properties
		double maxIsometricForce = 1000.0, optimalFiberLength = 0.25, tendonSlackLength = 0.1, pennationAngle = 0.0;
		Thelen2003Muscle *muscle1 = new Thelen2003Muscle("muscle1", maxIsometricForce, optimalFiberLength, tendonSlackLength, pennationAngle);
		Thelen2003Muscle *muscle2 = new Thelen2003Muscle("muscle2", maxIsometricForce, optimalFiberLength, tendonSlackLength, pennationAngle);

		// Specify the paths for the two muscles
		// Path for muscle 1
		muscle1->addNewPathPoint("muscle1-point1", ground, Vec3(0.0, 0.05, -0.35));
		muscle1->addNewPathPoint("muscle1-point2", *block, Vec3(0.0, 0.0, -0.05));
		// Path for muscle 2
		muscle2->addNewPathPoint("muscle2-point1", ground, Vec3(0.0, 0.05, 0.35));
		muscle2->addNewPathPoint("muscle2-point2", *block, Vec3(0.0, 0.0, 0.05));

		// Add the two muscles (as forces) to the model
		osimModel.addForce(muscle1);
		osimModel.addForce(muscle2);

		// CONTACT FORCE
		// Define contact geometry
		// Create new floor contact halfspace
		//ContactHalfSpace *floor = new ContactHalfSpace(SimTK::Vec3(0), SimTK::Vec3(0, 0, -0.5*SimTK_PI), ground, "floor");
		//// Create new cube contact mesh
		//OpenSim::ContactMesh *cube = new OpenSim::ContactMesh("blockMesh.obj", SimTK::Vec3(0), SimTK::Vec3(0), *block, "cube");

		//// Add contact geometry to the model
		//osimModel.addContactGeometry(floor);
		//osimModel.addContactGeometry(cube);

		//// Define contact parameters for elastic foundation force
		//OpenSim::ElasticFoundationForce::ContactParameters *contactParams =
		//	new OpenSim::ElasticFoundationForce::ContactParameters(stiffness, dissipation, friction, friction, viscosity);
		//contactParams->addGeometry("cube");
		//contactParams->addGeometry("floor");

		//// Create a new elastic foundation (contact) force between the floor and cube.
		//OpenSim::ElasticFoundationForce *contactForce = new OpenSim::ElasticFoundationForce(contactParams);
		//contactForce->setName("contactForce");

		//// Add the new elastic foundation force to the model
		//osimModel.addForce(contactForce);

		//// PRESCRIBED FORCE
		//// Create a new prescribed force to be applied to the block
		//PrescribedForce *prescribedForce = new PrescribedForce("prescribedForce", *block);

		//// Specify properties of the force function to be applied to the block
		//double time[2] = { 0, finalTime };                    // time nodes for linear function
		//double fXofT[2] = { 0,  -blockMass*gravity[1] * 3.0 };  // force values at t1 and t2

		//														// Create linear function for the force components
		//PiecewiseLinearFunction *forceX = new PiecewiseLinearFunction(2, time, fXofT);
		//// Set the force and point functions for the new prescribed force
		//prescribedForce->setForceFunctions(forceX, new Constant(0.0), new Constant(0.0));
		//prescribedForce->setPointFunctions(new Constant(0.0), new Constant(0.0), new Constant(0.0));

		//// Add the new prescribed force to the model
		//osimModel.addForce(prescribedForce);

		///////////////////////////////////
		// DEFINE CONTROLS FOR THE MODEL //
		///////////////////////////////////
		// Create a prescribed controller that simply applies controls as function of time
		// For muscles, controls are normalized motor-neuron excitations
		//PrescribedController *muscleController = new PrescribedController();
		//muscleController->setActuators(osimModel.updActuators());
		//// Define linear functions for the control values for the two muscles
		//Array<double> slopeAndIntercept1(0.0, 2);  // array of 2 doubles
		//Array<double> slopeAndIntercept2(0.0, 2);
		//// muscle1 control has slope of -1 starting 1 at t = 0
		//slopeAndIntercept1[0] = -1.0 / (finalTime - initialTime);  slopeAndIntercept1[1] = 1.0;
		//// muscle2 control has slope of 0.95 starting 0.05 at t = 0
		//slopeAndIntercept2[0] = 0.95 / (finalTime - initialTime);  slopeAndIntercept2[1] = 0.05;

		//// Set the individual muscle control functions for the prescribed muscle controller
		//muscleController->prescribeControlForActuator("muscle1", new LinearFunction(slopeAndIntercept1));
		//muscleController->prescribeControlForActuator("muscle2", new LinearFunction(slopeAndIntercept2));

		//// Add the muscle controller to the model
		//osimModel.addController(muscleController);

		///////////////////////////////////
		// SPECIFY MODEL DEFAULT STATES  //
		///////////////////////////////////
		// Define the default states for the two muscles
		// Activation
		muscle1->setDefaultActivation(0.01);
		muscle2->setDefaultActivation(0.01);
		// Fiber length
		muscle2->setDefaultFiberLength(optimalFiberLength);
		muscle1->setDefaultFiberLength(optimalFiberLength);

		// Save the model to a file
		//osimModel.print("tugOfWar_model.osim");

		return osimModel;

	}
	catch (const std::exception& ex)
	{
		cerr << ex.what() << endl;
	}
	catch (...)
	{
		cerr << "UNRECOGNIZED EXCEPTION" << endl;
	}

	cout << "buildTugofwarModel() routine time = " << 1.e3*(clock() - startTime) / CLOCKS_PER_SEC << "ms\n";

}

} // namespace OpenSim

int main()
{

	try {
		using namespace SimTK;
		using namespace OpenSim;
		using namespace std;

		auto model = Model();
		Ground& ground = model.updGround();
		ground.attachGeometry(new Mesh("checkered_floor.vtp"));

		// Ellipsoid
		OpenSim::PhysicalFrame* ellipsoidFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(-.6, 0.6, 0.)));
		ellipsoidFrame->setName("EllipsoidAnchor");
		model.addFrame(ellipsoidFrame);
/*
		auto con = new PrescribedController();
		con->setName("con");
		model.addComponent(con);
*/
		auto pa = new PathActuator();
		pa->setName("pathact");
		model.addComponent(pa);

		auto controller = buildStretchController(*pa);
		model.addComponent(controller);





	}
	catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
	}
}


//------------------------------------------------------------------------------
// START HERE! Toggle "if (false)" to "if (true)" to enable/disable each step in
// the exercise. The project should execute without making any changes (you
// should see the unassisted hopper hop slightly).
//------------------------------------------------------------------------------
int main2()
{
	try {
    using namespace OpenSim;
	using namespace std;
    //==========================================================================
    // Step 1. Build an stretch controller and test it on a simple testbed.
    //==========================================================================

	// Build the testbed and controller.
	cout << "starting" << endl;
	auto tugofwar = buildTugofwarModel();
	cout << "model created" << endl;

	//// Update the hopper model's internal data members, which includes
	//// identifying its subcomponents from its properties.
	//tugofwar.finalizeFromProperties();

	//// Show all Components in the model.
	//showSubcomponentInfo(tugofwar);
	//// Show only the Joints in the model.
	//showSubcomponentInfo<Joint>(tugofwar);

		
	auto& pathActuator = tugofwar.updComponent<PathActuator>("muscle1");
	auto controller = buildStretchController(pathActuator);
		

	cout << "model and controller created" << endl;
        
	// Connect the controller to the testbed. 
	connectControllerToModel(*controller, tugofwar);

	cout << "connected" << endl;
	// Use a SignalGenerator to create a set point signal for testing the
	// controller. 
	addSignalGeneratorToController(*controller);
	cout << "generator added" << endl;

	// Show all Components in the testbed.
	showSubcomponentInfo(tugofwar);
	showSubcomponentInfo(*controller);


	cout << "sub info are shown" << endl;

	// List the device outputs we wish to display during the simulation.
	std::vector<std::string> controllerOutputs{ "stretch_control"};

	// Add a ConsoleReporter to report deviceOutputs.
	addDeviceConsoleReporterToModel(tugofwar, *controller, controllerOutputs);
	cout << "before init" << endl;
	// Create the system, initialize the state, and simulate.
		

	SimTK::State& sDev = tugofwar.initSystem();
	cout << "before sim" << endl;
	OpenSim::simulate(tugofwar, sDev);

}
catch (const std::exception& e) {
	std::cout << e.what() << std::endl;
}
system("pause");
return 0;

};
