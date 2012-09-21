/* -------------------------------------------------------------------------- *
 *                        OpenSim:  toyLeg_example.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
 */

// Author:  Matt DeMers

//==============================================================================
//==============================================================================
#include "PistonActuator.h"
#include "ControllableSpring.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

//______________________________________________________________________________
/**
 * Run a simulation of block sliding with contact on by two muscles sliding with contact 
 */
int main()
{

	try {
		// Create a new OpenSim model
		Model osimModel;
		osimModel.setName("osimModel");

		double Pi = SimTK::Pi;
		
		
		// Get the ground body
		OpenSim::Body& ground = osimModel.getGroundBody();
		ground.addDisplayGeometry("ground.vtp");

		// create linkage body
		double linkageMass = 0.001, linkageLength = 0.5;
		Vec3 linkageMassCenter(0,-linkageLength/2,0);
		Inertia linkageInertia = Inertia::cylinderAlongY(0.0, 0.5);

		OpenSim::Body *linkage1 = new OpenSim::Body("linkage1", linkageMass, linkageMassCenter, linkageMass*linkageInertia);
		// Graphical representation
		linkage1->addDisplayGeometry("linkage1.vtp");
		
		// Creat a second linkage body
		OpenSim::Body *linkage2 = new OpenSim::Body("linkage2", linkageMass, linkageMassCenter, linkageMass*linkageInertia);
		linkage2->addDisplayGeometry("linkage1.vtp");

		// Creat a block to be the pelvis
		double blockMass = 20.0, blockSideLength = 0.2;
		Vec3 blockMassCenter(0);
		Inertia blockInertia = blockMass*Inertia::brick(blockSideLength, blockSideLength, blockSideLength);
		OpenSim::Body *block = new OpenSim::Body("block", blockMass, blockMassCenter, blockInertia);
		block->addDisplayGeometry("big_block_centered.vtp");

		// Create 1 degree-of-freedom pin joints between the bodies to creat a kinematic chain from ground through the block
		
		Vec3 orientationInGround(0), locationInGround(0), locationInParent(0.0, 0.5, 0.0), orientationInChild(0), locationInChild(0);

		PinJoint *ankle = new PinJoint("ankle", ground, locationInGround, orientationInGround, *linkage1, 
			locationInChild, orientationInChild);

		PinJoint *knee = new PinJoint("knee", *linkage1, locationInParent, orientationInChild, *linkage2,
			locationInChild, orientationInChild);

		PinJoint *hip = new PinJoint("hip", *linkage2, locationInParent, orientationInChild, *block,
			locationInChild, orientationInChild);
		
		double range[2] = {-SimTK::Pi*2, SimTK::Pi*2};
		CoordinateSet& ankleCoordinateSet = ankle->getCoordinateSet();
		ankleCoordinateSet[0].setName("q1");
		ankleCoordinateSet[0].setRange(range);

		CoordinateSet& kneeCoordinateSet = knee->getCoordinateSet();
		kneeCoordinateSet[0].setName("q2");
		kneeCoordinateSet[0].setRange(range);

		CoordinateSet& hipCoordinateSet = hip->getCoordinateSet();
		hipCoordinateSet[0].setName("q3");
		hipCoordinateSet[0].setRange(range);

		// Add the bodies to the model
		osimModel.addBody(linkage1);
		osimModel.addBody(linkage2);
		osimModel.addBody(block);

		// Define contraints on the model
		//  Add a point on line constraint to limit the block to vertical motion

		Vec3 lineDirection(0,1,0), pointOnLine(0,0,0), pointOnBlock(0);
		PointOnLineConstraint *lineConstraint = new PointOnLineConstraint(ground, lineDirection, pointOnLine, *block, pointOnBlock);
		osimModel.addConstraint(lineConstraint);

		// Add PistonActuator between the first linkage and the block
		Vec3 pointOnBodies(0);
		PistonActuator *piston = new PistonActuator();
		piston->setName("piston");
		piston->setBodyA(linkage1);
		piston->setBodyB(block);
		piston->setPointA(pointOnBodies);
		piston->setPointB(pointOnBodies);
		piston->setOptimalForce(200.0);
		piston->setPointsAreGlobal(false);

		osimModel.addForce(piston);
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// Added ControllableSpring between the first linkage and the second block
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		ControllableSpring *spring = new ControllableSpring;
		spring->setName("spring");
		spring->setBodyA(block);
		spring->setBodyB(linkage1);
		spring->setPointA(pointOnBodies);
		spring->setPointB(pointOnBodies);
		spring->setOptimalForce(2000.0);
		spring->setPointsAreGlobal(false);
		spring->setRestLength(0.8);

		osimModel.addForce(spring);

		// define the simulation times
		double t0(0.0), tf(15);

		// define the acceration due to gravity
		osimModel.setGravity(Vec3(0, -9.80665, 0));

		// define the control values for the piston
		//double controlT0[1] = {0.982}, controlTf[1] = {0.978};

		// define the control values for the spring
		double controlT0[1] = {1.0}, controlT1[1] = {1.0}, controlT2[1] = {0.25},
			controlT3[1] = {.25}, controlT4[1] = {5};

		ControlSet *controlSet = new ControlSet();
		ControlLinear *control1 = new ControlLinear();
		control1->setName("spring"); // change this between 'piston' and 'spring'
		//control1->setUseSteps(true);
		controlSet->append(control1);

		// set control values for the piston
		/*controlSet->setControlValues(t0, controlT0);
		controlSet->setControlValues(tf, controlTf);*/

		// set control values for the spring
		controlSet->setControlValues(t0, controlT0);
		controlSet->setControlValues(4.0, controlT1);
		controlSet->setControlValues(7.0, controlT2);
		controlSet->setControlValues(10.0, controlT3);
		controlSet->setControlValues(tf, controlT4);

		ControlSetController *legController = new ControlSetController();
		legController->setControlSet(controlSet);
		osimModel.addController(legController);		

		// Initialize system
		SimTK::State& si = osimModel.initSystem();

		// Add analyses to the model
		
		ForceReporter *forces = new ForceReporter(&osimModel);
		
		osimModel.updAnalysisSet().append(forces);
		
		// Pin joint initial states

		double q1_i = -Pi/4;
		double q2_i = - 2*q1_i;
		CoordinateSet &coordinates = osimModel.updCoordinateSet();
		coordinates[0].setValue(si, q1_i, true);
		coordinates[1].setValue(si,q2_i, true);

		// Setup integrator and manager
		
		SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getSystem());
		integrator.setAccuracy(1.0e-3);
		integrator.setAbsoluteTolerance(1.0e-4);
		Manager manager(osimModel, integrator);
		
		
		//Examine the model
		osimModel.printDetailedInfo(si, std::cout);
		// Save the model
		osimModel.print("toyLeg.osim");
		// Print out the initial position and velocity states
		si.getQ().dump("Initial q's");
		si.getU().dump("Initial u's");
		std::cout << "Initial time: " << si.getTime() << std::endl;

		// Integrate
		manager.setInitialTime(t0);
		manager.setFinalTime(tf);
		std::cout<<"\n\nIntegrating from " << t0 << " to " << tf << std::endl;
		manager.integrate(si);

		// Save results
		
		Storage statesDegrees(manager.getStateStorage());
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		//statesDegrees.print("PistonActuatedLeg_states_degrees.mot");
		statesDegrees.print("SpringActuatedLeg_states_degrees.mot");

		forces->getForceStorage().print("actuator_forces.mot");
		
	}
    catch (std::exception ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }

	std::cout << "Exiting" << std::endl;
	return 0;
}
