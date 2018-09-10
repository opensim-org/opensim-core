/* -------------------------------------------------------------------------- *
 *                      OpenSim:  TugOfWar_Complete.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Jeffrey A. Reinbolt, Ayman Habib, Ajay Seth, Samuel R. Hamner   *
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
 *  main() routine.  This application is a forward simulation of tug-of-war between two
 *  muscles pulling on a block.
 */

// Author:  Jeff Reinbolt, Ayman Habib, Ajay Seth, Samuel Hamner

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"

#include <ctime>    // for clock()

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//______________________________________________________________________________
/**
 * Run a simulation of block sliding with contact on by two muscles sliding with contact 
 */
int main()
{
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
        double stiffness = 1.0e7, dissipation = 0.1, friction = 0.2, viscosity=0.01;

        ///////////////////////////////////////////
        // DEFINE BODIES AND JOINTS OF THE MODEL //
        ///////////////////////////////////////////

        // Create an OpenSim model and set its name
        Model osimModel;
        osimModel.setName("tugOfWar");

        // GROUND FRAME
        // Get a reference to the model's ground frame
        Ground& ground = osimModel.updGround();
        // Create Frames to attach Geometry to
        // Left brick
        OpenSim::PhysicalFrame* leftAnchorFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(0, 0.05, 0.35)));
        leftAnchorFrame->setName("LeftAnchor");
        osimModel.addComponent(leftAnchorFrame);
        // Right brick
        OpenSim::PhysicalFrame* rightAnchorFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(0, 0.05, -0.35)));
        rightAnchorFrame->setName("RightAnchor");
        osimModel.addComponent(rightAnchorFrame);
        // Cylinder
        OpenSim::PhysicalFrame* cylFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(-.2, 0.0, 0.)));
        cylFrame->setName("CylAnchor");
        osimModel.addComponent(cylFrame);
        // Ellipsoid
        OpenSim::PhysicalFrame* ellipsoidFrame = new PhysicalOffsetFrame(ground, Transform(Vec3(-.6, 0.6, 0.)));
        ellipsoidFrame->setName("EllipsoidAnchor");
        osimModel.addComponent(ellipsoidFrame);


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
        Vec3 locationInParent(0, blockSideLength/2, 0), orientationInParent(0), locationInBody(0), orientationInBody(0);
        FreeJoint *blockToGround = new FreeJoint("blockToGround", ground, locationInParent, orientationInParent, *block, locationInBody, orientationInBody);
        
        // Set the angle and position ranges for the free (6-degree-of-freedom)
        // joint between the block and ground frames.
        double angleRange[2] = {-SimTK::Pi/2, SimTK::Pi/2};
        double positionRange[2] = {-1, 1};
        blockToGround->updCoordinate(FreeJoint::Coord::Rotation1X).setRange(angleRange);
        blockToGround->updCoordinate(FreeJoint::Coord::Rotation2Y).setRange(angleRange);
        blockToGround->updCoordinate(FreeJoint::Coord::Rotation3Z).setRange(angleRange);
        blockToGround->updCoordinate(FreeJoint::Coord::TranslationX).setRange(positionRange);
        blockToGround->updCoordinate(FreeJoint::Coord::TranslationY).setRange(positionRange);
        blockToGround->updCoordinate(FreeJoint::Coord::TranslationZ).setRange(positionRange);

        // GRAVITY
        // Obtain the default acceleration due to gravity
        Vec3 gravity = osimModel.getGravity();

        // Define non-zero default states for the free joint
        blockToGround->updCoordinate(FreeJoint::Coord::TranslationX)
                       .setDefaultValue(constantDistance);
        double h_start = blockMass*gravity[1] /
                         (stiffness*blockSideLength*blockSideLength);
        blockToGround->updCoordinate(FreeJoint::Coord::TranslationY)
                       .setDefaultValue(h_start); //y-translation is height

        // Add the block and joint to the model
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
        Vec3 pointOnGround(0, blockSideLength/2 ,0);
        Vec3 pointOnBlock(0, 0, 0);

        // Create a new constant distance constraint
        ConstantDistanceConstraint *constDist = 
            new ConstantDistanceConstraint(ground, 
                pointOnGround, *block, pointOnBlock, constantDistance);

        // Add the new point on a line constraint to the model
        osimModel.addConstraint(constDist);

        ///////////////////////////////////////
        // DEFINE FORCES ACTING ON THE MODEL //
        ///////////////////////////////////////
    
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

        // CONTACT FORCE
        // Define contact geometry
        // Create new floor contact halfspace
        ContactHalfSpace *floor = new ContactHalfSpace(SimTK::Vec3(0), SimTK::Vec3(0, 0, -0.5*SimTK_PI), ground, "floor");
        // Create new cube contact mesh
        OpenSim::ContactMesh *cube = new OpenSim::ContactMesh("blockMesh.obj", SimTK::Vec3(0), SimTK::Vec3(0), *block, "cube");

        // Add contact geometry to the model
        osimModel.addContactGeometry(floor);
        osimModel.addContactGeometry(cube);

        // Define contact parameters for elastic foundation force
        OpenSim::ElasticFoundationForce::ContactParameters *contactParams = 
            new OpenSim::ElasticFoundationForce::ContactParameters(stiffness, dissipation, friction, friction, viscosity);
        contactParams->addGeometry("cube");
        contactParams->addGeometry("floor");
        
        // Create a new elastic foundation (contact) force between the floor and cube.
        OpenSim::ElasticFoundationForce *contactForce = new OpenSim::ElasticFoundationForce(contactParams);
        contactForce->setName("contactForce");

        // Add the new elastic foundation force to the model
        osimModel.addForce(contactForce);

        // PRESCRIBED FORCE
        // Create a new prescribed force to be applied to the block
        PrescribedForce *prescribedForce = new PrescribedForce("prescribedForce", *block);

        // Specify properties of the force function to be applied to the block
        double time[2] = {0, finalTime};                    // time nodes for linear function
        double fXofT[2] = {0,  -blockMass*gravity[1]*3.0};  // force values at t1 and t2

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
        slopeAndIntercept1[0] = -1.0/(finalTime-initialTime);  slopeAndIntercept1[1] = 1.0;
        // muscle2 control has slope of 0.95 starting 0.05 at t = 0
        slopeAndIntercept2[0] = 0.95/(finalTime-initialTime);  slopeAndIntercept2[1] = 0.05;
        
        // Set the individual muscle control functions for the prescribed muscle controller
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
        //////////////////////////

        // set use visualizer to true to visualize the simulation live
        osimModel.setUseVisualizer(false);

        // Initialize the system and get the default state
        SimTK::State& si = osimModel.initSystem();
        // Enable constraint consistent with current configuration of the model
        constDist->setIsEnforced(si, true);

        cout << "Start height = "<< h_start << endl;
        osimModel.getMultibodySystem().realize(si, Stage::Velocity);

        // Compute initial conditions for muscles
        osimModel.equilibrateMuscles(si);

        /*double mfv1 = */muscle1->getFiberVelocity(si);
        /*double mfv2 = */muscle2->getFiberVelocity(si);

        // Create the force reporter for obtaining the forces applied to the model
        // during a forward simulation
        ForceReporter* reporter = new ForceReporter(&osimModel);
        osimModel.addAnalysis(reporter);

        // Create the manager managing the forward integration and its outputs
        Manager manager(osimModel);
        manager.setIntegratorAccuracy(1.0e-6);

        // Print out details of the model
        osimModel.printDetailedInfo(si, cout);

        // Integrate from initial time to final time
        si.setTime(initialTime);
        manager.initialize(si);
        cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<endl;
        manager.integrate(finalTime);

        //////////////////////////////
        // SAVE THE RESULTS TO FILE //
        //////////////////////////////
        // Save the model states from forward integration
        auto statesTable = manager.getStatesTable();
        STOFileAdapter_<double>::write(statesTable, "tugOfWar_states.sto");

        auto forcesTable = reporter->getForcesTable();
        STOFileAdapter_<double>::write(forcesTable, "tugOfWar_forces.sto");
    }
    catch (const std::exception& ex)
    {
        cerr << ex.what() << endl;
        return 1;
    }
    catch (...)
    {
        cerr << "UNRECOGNIZED EXCEPTION" << endl;
        return 1;
    }

    cout << "main() routine time = " << 1.e3*(clock()-startTime)/CLOCKS_PER_SEC << "ms\n";

    cout << "OpenSim example completed successfully." << endl;

    return 0;
}
