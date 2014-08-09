/* -------------------------------------------------------------------------- *
 *                             OpenSim:  test.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

/*
 *  Below is an example of an OpenSim application that provides its own
 *  main() routine.
 */

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

//______________________________________________________________________________
/**
 * Create a model that does nothing.
 */
int main()
{
    try {

        ///////////////////////////////////////////
        // DEFINE BODIES AND JOINTS OF THE MODEL //
        ///////////////////////////////////////////

        // Create an OpenSim model and set its name
        Model osimModel;
        osimModel.setName("tugOfWar");

        // GROUND BODY

        // Get a reference to the model's ground body
        OpenSim::Body& ground = osimModel.getGroundBody();

        // Add display geometry to the ground to visualize in the GUI
        ground.addDisplayGeometry("ground.vtp");
        ground.addDisplayGeometry("anchor1.vtp");
        ground.addDisplayGeometry("anchor2.vtp");

        // BLOCK BODY

        // Specify properties of a 20 kg, 0.1 m^3 block body
        double blockMass = 20.0, blockSideLength = 0.1;
        Vec3 blockMassCenter(0);
        Inertia blockInertia = blockMass*Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

        // Create a new block body with the specified properties
        OpenSim::Body *block = new OpenSim::Body("block", blockMass, blockMassCenter, blockInertia);

        // Add display geometry to the block to visualize in the GUI
        block->addDisplayGeometry("block.vtp");

        // FREE JOINT

        // Create a new free joint with 6 degrees-of-freedom (coordinates) between the block and ground bodies
        Vec3 locationInParent(0, blockSideLength/2, 0), orientationInParent(0), locationInBody(0), orientationInBody(0);
        FreeJoint *blockToGround = new FreeJoint("blockToGround", ground, locationInParent, orientationInParent, *block, locationInBody, orientationInBody);

        // Get a reference to the coordinate set (6 degrees-of-freedom) between the block and ground bodies
        CoordinateSet& jointCoordinateSet = blockToGround->upd_CoordinateSet();

        // Set the angle and position ranges for the coordinate set
        double angleRange[2] = {-SimTK::Pi/2, SimTK::Pi/2};
        double positionRange[2] = {-1, 1};
        jointCoordinateSet[0].setRange(angleRange);
        jointCoordinateSet[1].setRange(angleRange);
        jointCoordinateSet[2].setRange(angleRange);
        jointCoordinateSet[3].setRange(positionRange);
        jointCoordinateSet[4].setRange(positionRange);
        jointCoordinateSet[5].setRange(positionRange);

        // Add the block body to the model
        osimModel.addBody(block);
        osimModel.addJoint(blockToGround);

        ///////////////////////////////////////////////
        // DEFINE THE SIMULATION START AND END TIMES //
        ///////////////////////////////////////////////

        // Define the initial and final simulation times
        double initialTime = 0.0;
        double finalTime = 3.00;

        /////////////////////////////////////////////
        // DEFINE CONSTRAINTS IMPOSED ON THE MODEL //
        /////////////////////////////////////////////

        // Specify properties of a constant distance constraint to limit the block's motion
        double distance = 0.2;
        Vec3 pointOnGround(0, blockSideLength/2 ,0);
        Vec3 pointOnBlock(0, 0, 0);

        // Create a new constant distance constraint
        ConstantDistanceConstraint *constDist = new ConstantDistanceConstraint(ground,
                pointOnGround, *block, pointOnBlock, distance);

        // Add the new point on a line constraint to the model
        osimModel.addConstraint(constDist);

        ///////////////////////////////////////
        // DEFINE FORCES ACTING ON THE MODEL //
        ///////////////////////////////////////

        // GRAVITY
        // Obtaine the default acceleration due to gravity
        Vec3 gravity = osimModel.getGravity();


        // MUSCLE FORCES
        // Create two new muscles with identical properties
        double maxIsometricForce = 1000.0, optimalFiberLength = 0.25, tendonSlackLength = 0.1, pennationAngle = 0.0;
        Thelen2003Muscle *muscle1 = new Thelen2003Muscle("muscle1",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
        Thelen2003Muscle *muscle2 = new Thelen2003Muscle("muscle2",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);

        // Specify the paths for the two muscles
        // Path for muscle 1
        muscle1->addNewPathPoint("muscle1-point1", ground, Vec3(0.0,0.05,-0.35));
        muscle1->addNewPathPoint("muscle1-point2", *block, Vec3(0.0,0.0,-0.05));
        // Path for muscle 2
        muscle2->addNewPathPoint("muscle2-point1", ground, Vec3(0.0,0.05,0.35));
        muscle2->addNewPathPoint("muscle2-point2", *block, Vec3(0.0,0.0,0.05));

        // Add the two muscles (as forces) to the model
        osimModel.addForce(muscle1);
        osimModel.addForce(muscle2);


        // PRESCRIBED FORCE
        // Create a new prescribed force to be applied to the block
        PrescribedForce *prescribedForce = new PrescribedForce(block);
        prescribedForce->setName("prescribedForce");

        // Specify properties of the force function to be applied to the block
        double time[2] = {0, finalTime};					// time nodes for linear function
        double fXofT[2] = {0,  -blockMass*gravity[1]*3.0};	// force values at t1 and t2

        // Create linear function for the force components
        PiecewiseLinearFunction *forceX = new PiecewiseLinearFunction(2, time, fXofT);
        // Set the force and point functions for the new prescribed force
        prescribedForce->setForceFunctions(forceX, new Constant(0.0), new Constant(0.0));
        prescribedForce->setPointFunctions(new Constant(0.0), new Constant(0.0), new Constant(0.0));

        // Add the new prescribed force to the model
        osimModel.addForce(prescribedForce);

        ///////////////////////////////////
        // DEFINE CONTROLS FOR THE MODEL //
        ///////////////////////////////////
        // Create a prescribed controller that simply applies controls as function of time
        // For muscles, controls are normalized motor-neuron excitations
        PrescribedController *muscleController = new PrescribedController();
        muscleController->setActuators(osimModel.updActuators());
        // Define linear functions for the control values for the two muscles
        Array<double> slopeAndIntercept1(0.0, 2);  // array of 2 doubles
        Array<double> slopeAndIntercept2(0.0, 2);
        // muscle1 control has slope of -1 starting 1 at t = 0
        slopeAndIntercept1[0] = -1.0/(finalTime-initialTime);
        slopeAndIntercept1[1] = 1.0;
        // muscle2 control has slope of 0.95 starting 0.05 at t = 0
        slopeAndIntercept2[0] = 0.95/(finalTime-initialTime);
        slopeAndIntercept2[1] = 0.05;

        // Set the indiviudal muscle control functions for the prescribed muscle controller
        muscleController->prescribeControlForActuator("muscle1", new LinearFunction(slopeAndIntercept1));
        muscleController->prescribeControlForActuator("muscle2", new LinearFunction(slopeAndIntercept2));

        // Add the muscle controller to the model
        osimModel.addController(muscleController);

        ///////////////////////////////////
        // SPECIFY MODEL DEFAULT STATES  //
        ///////////////////////////////////
        // Define the default states for the two muscles
        // Activation
        muscle1->setDefaultActivation(slopeAndIntercept1[1]);
        muscle2->setDefaultActivation(slopeAndIntercept2[1]);
        // Fiber length
        muscle2->setDefaultFiberLength(optimalFiberLength);
        muscle1->setDefaultFiberLength(optimalFiberLength);

        // Save the model to a file
        osimModel.print("tugOfWar_model.osim");

        //////////////////////////
        // PERFORM A SIMULATION //
        /////////////////////////

        //osimModel.setUseVisualizer(true);

        // Initialize the system and get the default state
        SimTK::State& si = osimModel.initSystem();

        // Define non-zero (defaults are 0) states for the free joint
        CoordinateSet& modelCoordinateSet = osimModel.updCoordinateSet();
        modelCoordinateSet[3].setValue(si, distance); // set x-translation value
        modelCoordinateSet[5].setValue(si, 0.0); // set z-translation value
        modelCoordinateSet[3].setSpeedValue(si, 0.0); // set x-speed value
        double h_start = 0.5;
        modelCoordinateSet[4].setValue(si, h_start); // set y-translation which is height

        std::cout << "Start height = "<< h_start << std::endl;

        osimModel.getMultibodySystem().realize(si, Stage::Velocity);

        // Compute initial conditions for muscles
        osimModel.equilibrateMuscles(si);

        double mfv1 = muscle1->getFiberVelocity(si);
        double mfv2 = muscle2->getFiberVelocity(si);

        // Create the force reporter for obtaining the forces applied to the model
        // during a forward simulation
        ForceReporter* reporter = new ForceReporter(&osimModel);
        osimModel.addAnalysis(reporter);

        // Create the integrator for integrating system dynamics
        SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
        integrator.setAccuracy(1.0e-6);

        // Create the manager managing the forward integration and its outputs
        Manager manager(osimModel,  integrator);

        // Integrate from initial time to final time
        manager.setInitialTime(initialTime);
        manager.setFinalTime(finalTime);
        std::cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
        manager.integrate(si);

    }
    catch (const std::exception& ex)
    {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    std::cout << "OpenSim environment test completed successfully. You should see a block attached to two muscles visualized in a separate window." << std::endl;

    return 0;
}