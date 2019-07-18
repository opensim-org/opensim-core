/* -------------------------------------------------------------------------- *
 *                        OpenSim:  toyLeg_example.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers                                                  *
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
 *  main() routine.  This application acts as an example for utilizing the 
 *  ControllabeSpring actuator.
 *
 *  See the following link for a walkthrough of this example:
 *  https://simtk-confluence.stanford.edu/display/OpenSim/Creating+a+Customized+Actuator
 */

// Author:  Matt DeMers

//==============================================================================
//==============================================================================
#include "PistonActuator.h"
#include "ControllableSpring.h"
#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"

using namespace OpenSim;
using namespace SimTK;

int main()
{
    try {
        // Create a new OpenSim model.
        Model osimModel;
        osimModel.setName("osimModel");
        osimModel.setAuthors("Matt DeMers");

        double Pi = SimTK::Pi;
            
        // Get the ground body.
        Ground& ground = osimModel.updGround();
        ground.attachGeometry(new Mesh("checkered_floor.vtp"));

        // Create linkage body.
        double linkageMass = 0.001, linkageLength = 0.5, linkageDiameter = 0.06;
        
        Vec3 linkageMassCenter(0,linkageLength/2,0);
        Inertia linkageInertia =
            Inertia::cylinderAlongY(linkageDiameter/2.0, linkageLength/2.0);

        OpenSim::Body* linkage1 = new OpenSim::Body("linkage1", linkageMass,
                linkageMassCenter, linkageMass*linkageInertia);

        // Graphical representation.
        Sphere sphere(0.05);
        linkage1->attachGeometry(sphere.clone());
        Cylinder cyl(linkageDiameter/2, linkageLength/2);
        Frame* cyl1Frame = new PhysicalOffsetFrame(*linkage1, 
            Transform(Vec3(0.0, linkageLength / 2.0, 0.0)));
        cyl1Frame->setName("Cyl1_frame");
        cyl1Frame->attachGeometry(cyl.clone());
        osimModel.addComponent(cyl1Frame);

        // Create a second linkage body.
        OpenSim::Body* linkage2 = new OpenSim::Body("linkage2", linkageMass,
                linkageMassCenter, linkageMass*linkageInertia);
        linkage2->attachGeometry(sphere.clone());
        Frame* cyl2Frame = new PhysicalOffsetFrame(*linkage2,
            Transform(Vec3(0.0, linkageLength / 2.0, 0.0)));
        cyl2Frame->setName("Cyl2_frame");
        cyl2Frame->attachGeometry(cyl.clone());
        osimModel.addComponent(cyl2Frame);

        // Create a block to be the pelvis.
        double blockMass = 20.0, blockSideLength = 0.2;
        Vec3 blockMassCenter(0);
        Inertia blockInertia = blockMass*Inertia::brick(blockSideLength,
                blockSideLength, blockSideLength);
        OpenSim::Body* block = new OpenSim::Body("block", blockMass,
                blockMassCenter, blockInertia);
        block->attachGeometry(new Brick(SimTK::Vec3(0.05, 0.05, 0.05)));

        // Create 1 degree-of-freedom pin joints between the bodies to create a
        // kinematic chain from ground through the block.
        Vec3 orientationInGround(0);
        Vec3 locationInGround(0);
        Vec3 locationInParent(0.0, linkageLength, 0.0);
        Vec3 orientationInChild(0);
        Vec3 locationInChild(0);

        PinJoint* ankle = new PinJoint("ankle",
                ground, locationInGround, orientationInGround,
                *linkage1, locationInChild, orientationInChild);

        PinJoint* knee = new PinJoint("knee",
                *linkage1, locationInParent, orientationInChild,
                *linkage2, locationInChild, orientationInChild);

        PinJoint* hip = new PinJoint("hip",
                *linkage2, locationInParent, orientationInChild,
                *block, locationInChild, orientationInChild);
        
        double range[2] = {-SimTK::Pi*2, SimTK::Pi*2};
        ankle->updCoordinate().setName("q1");
        ankle->updCoordinate().setRange(range);

        knee->updCoordinate().setName("q2");
        knee->updCoordinate().setRange(range);

        hip->updCoordinate().setName("q3");
        hip->updCoordinate().setRange(range);

        // Add the bodies to the model
        osimModel.addBody(linkage1);
        osimModel.addBody(linkage2);
        osimModel.addBody(block);

        // Add the joints to the model
        osimModel.addJoint(ankle);
        osimModel.addJoint(knee);
        osimModel.addJoint(hip);
        // Define constraints on the model
        // Add a point on line constraint to limit the block to vertical motion
        Vec3 lineDirection(0,1,0), pointOnLine(0,0,0), pointOnBlock(0);
        PointOnLineConstraint *lineConstraint =
            new PointOnLineConstraint(ground, lineDirection, pointOnLine,
                    *block, pointOnBlock);
        osimModel.addConstraint(lineConstraint);

        // Add PistonActuator between the first linkage and the block
        Vec3 pointOnBodies(0);
        PistonActuator* piston = new PistonActuator();
        piston->setName("piston");
        piston->setFrameA(*linkage1);
        piston->setFrameB(*block);
        piston->setPointA(pointOnBodies);
        piston->setPointB(pointOnBodies);
        piston->setOptimalForce(200.0);
        piston->setPointsAreGlobal(false);

        osimModel.addForce(piston);
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Added ControllableSpring between the first linkage and the second block
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        ControllableSpring* spring = new ControllableSpring();
        spring->setName("spring");
        spring->setFrameA(*block);
        spring->setFrameB(*linkage1);
        spring->setPointA(pointOnBodies);
        spring->setPointB(pointOnBodies);
        spring->setOptimalForce(2000.0);
        spring->setPointsAreGlobal(false);
        spring->setRestLength(0.8);

        osimModel.addForce(spring);

        // define the simulation times
        double t0(0.0), tf(15);

        // create a controller to control the piston and spring actuators
        // the prescribed controller sets the controls as functions of time
        PrescribedController* legController = new PrescribedController();
        // give the legController control over all (two) model actuators
        legController->setActuators(osimModel.updActuators());

        // specify some control nodes for spring stiffness control
        double t[] = {0.0, 4.0, 7.0,  10.0, 15.0};
        double x[] = {1.0, 1.0, 0.25,  0.25, 5.0};

        // specify the control function for each actuator
        legController->prescribeControlForActuator("piston",
                new Constant(0.1));
        legController->prescribeControlForActuator("spring", 
                new PiecewiseLinearFunction(5, t, x));

        // add the controller to the model
        osimModel.addController(legController);     
        
        // define the acceleration due to gravity
        osimModel.setGravity(Vec3(0, -9.80665, 0));

        // Set to true to visualize the simulation, which can be useful for
        // debugging.
        osimModel.setUseVisualizer(false);

        // Initialize system
        SimTK::State& si = osimModel.initSystem();
        
        // Pin joint initial states
        double q1_i = -Pi/4;
        double q2_i = - 2*q1_i;
        CoordinateSet &coordinates = osimModel.updCoordinateSet();
        coordinates[0].setValue(si, q1_i, true);
        coordinates[1].setValue(si,q2_i, true);

        // Setup ForceReporter and Manager
        ForceReporter* forces = new ForceReporter(&osimModel);  
        osimModel.updAnalysisSet().adoptAndAppend(forces);
        Manager manager(osimModel);
        manager.setIntegratorAccuracy(1.0e-3);
    
        //Examine the model
        osimModel.printDetailedInfo(si, std::cout);

        // No need to call the finalizeConnections() as it is 
        // already called by the initSystem()
        // Save the model
        osimModel.print("toyLeg.osim");

        // Print out the initial position and velocity states
        si.getQ().dump("Initial q's");
        si.getU().dump("Initial u's");
        std::cout << "Initial time: " << si.getTime() << std::endl;

        // Integrate
        si.setTime(t0);
        manager.initialize(si);
        std::cout<<"\n\nIntegrating from " << t0 << " to " << tf << std::endl;
        manager.integrate(tf);

        // Save results
        auto controlsTable = osimModel.getControlsTable();
        STOFileAdapter::write(controlsTable, "SpringActuatedLeg_controls.sto");

        auto statesTable = manager.getStatesTable();
        osimModel.updSimbodyEngine().convertRadiansToDegrees(statesTable);
        STOFileAdapter::write(statesTable,
                "SpringActuatedLeg_states_degrees.sto");

        auto forcesTable = forces->getForcesTable();
        STOFileAdapter::write(forcesTable, "actuator_forces.sto");
    }
    catch (const std::exception& ex)
    {
        std::cout << "Exception in toyLeg_example: " << ex.what() << std::endl;
        return 1;
    }

    std::cout << "Done." << std::endl;
    return 0;
}
