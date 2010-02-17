// fatigue.cpp

/* Copyright (c)  2009 Stanford University
 * Use of the OpenSim software in source form is permitted provided that the following
 * conditions are met:
 *   1. The software is used only for non-commercial research and education. It may not
 *     be used in relation to any commercial activity.
 *   2. The software is not distributed or redistributed.  Software distribution is allowed 
 *     only through https://simtk.org/home/opensim.
 *   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
 *      presentations, or documents describing work in which OpenSim or derivatives are used.
 *   4. Credits to developers may not be removed from executables
 *     created from modifications of the source.
 *   5. Modifications of source code must retain the above copyright notice, this list of
 *     conditions and the following disclaimer. 
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine, and that uses a user-defined muscle model, LiuThelen2003Muscle.
 *  This application is a forward simulation of tug-of-war between two muscles pulling
 *  on a block, one of which experiences fatigue.
 */

// Author:  Jeff Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton, Peter Loan

//==============================================================================
//==============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PointOnLineConstraint.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Actuators/Schutte1993Muscle.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactGeometrySet.h>
#include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
#include <OpenSim/Simulation/Model/ContactMesh.h>
#include <OpenSim/Simulation/Model/PrescribedForce.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include "LiuThelen2003Muscle.h"

using namespace OpenSim;
using namespace SimTK;

//______________________________________________________________________________
/**
 * Run a simulation of block sliding with contact on by two muscles sliding with contact 
 */
int main()
{
	// Variables for Windows performance clocking
	LARGE_INTEGER start;
	LARGE_INTEGER stop;
	LARGE_INTEGER frequency;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start);

	// Create a new OpenSim model
	Model osimModel;
	osimModel.setName("osimModel");

	// Get the ground body
	OpenSim::Body& ground = osimModel.getGroundBody();
	ground.addDisplayGeometry("ground.vtp");
	ground.addDisplayGeometry("anchor1.vtp");
	ground.addDisplayGeometry("anchor2.vtp");

	// Create a 20 kg, 0.1 m^3 block body
	double blockMass = 20.0, blockSideLength = 0.1, blockInertia = blockMass/12.0*2.0*blockSideLength*blockSideLength;
	SimTK::Vec3 blockMassCenter(0,0,0);
	OpenSim::Body block("block", blockMass, blockMassCenter, Inertia(Vec3(blockInertia)).toMat33());
	// Graphical representation of the block for the GUI
	block.addDisplayGeometry("block.vtp");

	//Create a free joint with 6 degrees-of-freedom
	FreeJoint blockToGround(ground, Vec3(0,blockSideLength/2,0), Vec3(0), block, Vec3(0), Vec3(0));
	// Create 6 coordinates (degrees-of-freedom) between the ground and block
	CoordinateSet& jointCoordinateSet = blockToGround.getCoordinateSet();
	double angleRange[2] = {-SimTK::Pi/2, SimTK::Pi/2};
	double posRange[2] = {-1, 1};
	jointCoordinateSet[0].setName("xRotation");
	jointCoordinateSet[0].setRange(angleRange);
	jointCoordinateSet[1].setName("yRotation");
	jointCoordinateSet[1].setRange(angleRange);
	jointCoordinateSet[2].setName("zRotation");
	jointCoordinateSet[2].setRange(angleRange);
	jointCoordinateSet[3].setName("xTranslation");
	jointCoordinateSet[3].setMotionType(Coordinate::Translational);
	jointCoordinateSet[3].setRange(posRange);
	jointCoordinateSet[4].setName("yTranslation");
	jointCoordinateSet[4].setMotionType(Coordinate::Translational);
	jointCoordinateSet[4].setRange(posRange);
	jointCoordinateSet[5].setName("zTranslation");
	jointCoordinateSet[5].setMotionType(Coordinate::Translational);
	jointCoordinateSet[5].setRange(posRange);
	// Use this joint to connext the block to ground
	block.setJoint(blockToGround);
	// Add the block body to the model
	osimModel.addBody(&block);
	osimModel.updBodySet().setMemoryOwner(false);

	// Define any constraints on the model
	// Create a line constraint to limit the motion of the block
	Vec3 lineDirection(1,0,-1);
	Vec3 pointOnLine(1,0,-1);
	Vec3 pointOnFollowerBody(0,-0.05,0);
	PointOnLineConstraint lineConstraint(ground, lineDirection, pointOnLine, block, pointOnFollowerBody);
	osimModel.updConstraintSet().append(&lineConstraint);
	osimModel.updConstraintSet().setMemoryOwner(false);

	// Define all forces acting on the model
	// Create two muscles
	double maxIsometricForce1 = 2000.0, maxIsometricForce2 = 1000.0, optimalFiberLength = 0.2, tendonSlackLength = 0.2;
	double pennationAngle = 0.0, activation = 0.0001, deactivation = 1.0, fatigueFactor = 0.0, recoveryFactor = 0.00;

	// muscle 1 (model with fatigue)
	LiuThelen2003Muscle muscle1("muscle1",maxIsometricForce1,optimalFiberLength,tendonSlackLength,pennationAngle, fatigueFactor, recoveryFactor);
	muscle1.setActivationTimeConstant(activation);
	muscle1.setDeactivationTimeConstant(deactivation);
	// muscle 2 (model without fatigue)
	Thelen2003Muscle muscle2("muscle2",maxIsometricForce2,optimalFiberLength,tendonSlackLength,pennationAngle);
	muscle2.setActivationTimeConstant(activation);
	muscle2.setDeactivationTimeConstant(deactivation);

	// Update the model's force set
	osimModel.updForceSet().append(&muscle1);
	osimModel.updForceSet().append(&muscle2);
	osimModel.updForceSet().setMemoryOwner(false);

	// Reinitialize the system (with new muscles)
	//SimTK::State& s = osimModel.initSystem();

	// Define the paths of the muscles
	muscle1.addNewAttachmentPoint("muscle1-point1",ground,SimTK::Vec3(0.0,0.05,-0.35));
	muscle1.addNewAttachmentPoint("muscle1-point2",block,SimTK::Vec3(0.0,0.0,-0.05));

	muscle2.addNewAttachmentPoint("muscle2-point1",ground,SimTK::Vec3(0.0,0.05,0.35));
	muscle2.addNewAttachmentPoint("muscle2-point2",block,SimTK::Vec3(0.0,0.0,0.05));

	// Create ContactGeometry.
	ContactHalfSpace halfSpace(SimTK::Vec3(0), SimTK::Vec3(0, 0, -0.5*SimTK_PI), ground);
	halfSpace.setName("ground");
	osimModel.updContactGeometrySet().append(&halfSpace);
	OpenSim::ContactMesh geometry("blockRemesh192.obj", SimTK::Vec3(0), SimTK::Vec3(0), block);
	geometry.setName("block");
	osimModel.updContactGeometrySet().append(&geometry);
	osimModel.updContactGeometrySet().setMemoryOwner(false);

	// Create an ElasticFoundationForce.
	OpenSim::ElasticFoundationForce force;
	osimModel.updForceSet().append(&force);
	OpenSim::ElasticFoundationForce::ContactParameters contactParams;
	contactParams.updGeometry().append("block");
	contactParams.updGeometry().append("ground");
	contactParams.setStiffness(1.0e8);
	contactParams.setDissipation(0.01);
	contactParams.setDynamicFriction(0.25);
	force.updContactParametersSet().append(contactParams);

	// Define the initial and final simulation times
	double initialTime = 0.0;
	double finalTime = 6.0;

	// Define the acceleration due to gravity
	osimModel.setGravity(Vec3(0,-9.80665,0));

	// Define the initial and final controls
	double initialControl[2] = {1.0, 1.0};
	double finalControl[2] = {1.0, 1.0};
	ControlSet muscleControls;
	ControlLinear control1, control2;
	control1.setName("muscle1"); control2.setName("muscle2");
	muscleControls.append(&control1);
	muscleControls.append(&control2);
	muscleControls.setControlValues(initialTime, initialControl);
	muscleControls.setControlValues(finalTime, finalControl);
	muscleControls.setMemoryOwner(false);
	ControlSetController muscleController;
	muscleController.setControlSet(&muscleControls);

	osimModel.updControllerSet().append(&muscleController);
	osimModel.updControllerSet().setMemoryOwner(false);

	// Initialize the system and get the state representing the state system
	SimTK::State& si = osimModel.initSystem();

	// Free joint states
	CoordinateSet &coordinates = osimModel.updCoordinateSet();
	for (int i=0; i<6; i++) {
		coordinates[i].setValue(si, 0.0);
		coordinates[i].setSpeedValue(si, 0.0);
	}

	// Define the initial muscle states
	muscle1.setDefaultActivation(initialControl[0]); // muscle1 activation
	muscle1.setDefaultFiberLength(0.1); // muscle1 fiber length
	muscle1.initState(si); // initialize muscle1 state
	muscle2.setDefaultActivation(initialControl[1]); // muscle2 activation
	muscle2.setDefaultFiberLength(0.1); // muscle2 fiber length
	muscle2.initState(si); // initialize muscle2 state

	// Create the integrator and manager for the simulation.
	osimModel.getSystem().realize(si, SimTK::Stage::Velocity);
	SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getSystem());
	integrator.setMaximumStepSize(1.0e-3);
	integrator.setMinimumStepSize(1.0e-6);
	integrator.setAccuracy(1.0e-3);
	integrator.setAbsoluteTolerance(1.0e-4);
	Manager manager(osimModel, osimModel.getSystem(), integrator);

	// Examine the model
	osimModel.printDetailedInfo(si, std::cout);
	// Print out the initial position and velocity states
	si.getQ().dump("Initial q's"); // block positions
	si.getU().dump("Initial u's"); // block velocities

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
	manager.integrate(si);

	// Save the simulation results
	Storage statesDegrees(manager.getStateStorage());
	statesDegrees.print("tugOfWar_fatigue_states.sto");
	osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print("tugOfWar_fatigue_states_degrees.mot");
/*
	Storage forcesStorage(manager.getForceStorage());
	forcesStorage.setWriteSIMMHeader(true);
	forcesStorage.print("tugOfWar_forces.mot");
*/
	// Save the OpenSim model to a file
	osimModel.print("tugOfWar_fatigue_model.osim");

	QueryPerformanceCounter(&stop);
	double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	std::cout << "main() routine time = " << (duration*1.0e3) << " milliseconds" << std::endl;

	return 0;
}
