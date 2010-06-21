// testMomentArms.cpp
// Author:  Ajay Seth
/*
* Copyright (c) 2005-2010, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
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

//==========================================================================================================
//	testMomentArms builds various OpenSim models using the OpenSim API and compares moment arm
//  results from these models to the definition r*f = Tau , where r is the moment-arm about a coordinate,
//  f is the scaler maginitude of the Force and Tau is the resulting generalized force. 
//
//	Tests Include:
//      1. point to point muscle, no via points no wrapping spanning a pin joint
//		2. 
//		
//     Add more test cases to address specific problems with moment-arms
//
//==========================================================================================================
#include <OpenSim/OpenSim.h>
#include <iostream>
#include <OpenSim/Common/IO.h>
#include "SimTKmath.h"


using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}
#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs(double (expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

//==========================================================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-5;
const static double duration = 1.2;
const static Vec3 gravity_vec = Vec3(0, -9.8065, 0);

//Thigh
const static double femurMass = 8.806;
const static Vec3 femurCOM(0, 0.5, 0);
const static Inertia femurInertiaAboutCOM(Vec3(0.1268, 0.0332, 0.1337));
//Shank
const static MassProperties tibiaMass(3.510, Vec3(0), Inertia(Vec3(0.0477, 0.0048, 0.0484)));
//Foot
const static MassProperties footMass(1.20, Vec3(0), Inertia(Vec3(0.001361, 0.003709, 0.003916)));
//Toes
const static MassProperties toesMass(0.205126, Vec3(0), Inertia(Vec3(0.000117, 0.000179, 0.000119)));

// Joint locations
const static Vec3 hipInGround(0);
const static Vec3 hipInFemur(0.0020, 0.1715, 0);
const static Vec3 kneeInFemur(0.0033, -0.2294-0.1715, 0);
const static Vec3 kneeInTibia(0.0, 0.0, 0.0); //0.1862
const Vec3 ankleInTibia(0.0, -0.243800, 0);
const Vec3 ankleInFoot(-0.035902, 0.051347, 0);
const Vec3 mtpInFoot(0.098032, -0.038000, 0);
const Vec3 mtpInToes(-0.035902, 0.051347, 0);
//==========================================================================================================
static int counter=0;

//==========================================================================================================
// Utilities
//==========================================================================================================
int simulateToCheckMomentArms(Model &osimModel, SimTK::State &s)
{
	//==========================================================================================================
	// Compute the force and torque at the specified angles.

	double nsteps = 10;
	double dt = duration/nsteps;

	double q = osimModel.getCoordinateSet()[0].getValue(s);
	double dq = Pi/(2*nsteps);

	// Also disable gravity
	osimModel.getGravityForce().disable(s);

	for(int i = 0; i <=nsteps; i++){
		osimModel.getCoordinateSet()[0].setValue(s, q);

		osimModel.getMultibodySystem().realize(s, Stage::Acceleration);
	
		// Get all applied body forces like those from conact
		const Vector_<SpatialVec>& appliedBodyForces = osimModel.getMultibodySystem().getRigidBodyForces(s, Stage::Dynamics);

		// Get current system accelerations
		const Vector &knownUDots = s.getUDot();
		//knownUDots.dump("Actual accelerations:");

		//Results from an inverse dynamics for the generalized forces to satisfy accelerations
		Vector equivalentTorque, equivalentTorqueMUdot;

		// Convert body forces to equivalent mobility forces (joint torques)
		osimModel.getMultibodySystem().getMatterSubsystem().calcTreeEquivalentMobilityForces(s, appliedBodyForces, equivalentTorque);
		// Compute joint torque resulting from inertial forces (M*Udot) 
		osimModel.getMultibodySystem().getMatterSubsystem().calcMV(s, knownUDots, equivalentTorqueMUdot);

		// These should be equal
		//ASSERT_EQUAL(equivalentTorque[0], equivalentTorqueMUdot[0], integ_accuracy);

		// Model with one force, which is the muscle
		Thelen2003Muscle &muscle = *dynamic_cast<Thelen2003Muscle*>(&(osimModel.getForceSet()[0]));
		double force = muscle.getTendonForce(s);
		cout << "muscle  force: " << muscle.getForce(s) << endl;
		double ma = muscle.computeMomentArm(s, osimModel.getCoordinateSet()[0]);

		cout << "Momement arm = " << ma << " at q = " << s.getQ()[0]*180/Pi <<
			"  Torque = " << equivalentTorque[0] <<"::" << equivalentTorqueMUdot[0] << "  ma*force = " << ma*force << endl;
		
		// Verify that the definition of the moment-arm is satisfied
		ASSERT_EQUAL(equivalentTorqueMUdot[0], ma*force, integ_accuracy);

		// Increment the joint angle
		q += dq;
	}

	return 0;
}


//==========================================================================================================
// Test Cases
//==========================================================================================================
int testPointToPointMuscleAcrossPinJoint()
{
	//==========================================================================================================
	// Setup OpenSim model
	Model osimModel;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel.getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);
	osim_thigh.addDisplayGeometry("femur_r.vtp");

	// create hip as an Ball joint
	WeldJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));

	// Add the thigh body which now also contains the hip joint to the model
	osimModel.addBody(&osim_thigh);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());
	osim_shank.addDisplayGeometry("tibia_r.vtp");

	// create custom knee joint
	PinJoint knee("", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0));

	// Rename knee coordinates for a pin joint
	knee.getCoordinateSet()[0].setName("knee_q");
	double range[2] = {-Pi, Pi/20};
	knee.getCoordinateSet()[0].setRange(range);

	// Add the shank body which now also contains the knee joint to the model
	osimModel.addBody(&osim_shank);

	// Create a muscle that spans the knee
	double maxIsometricForce = 1000.0, optimalFiberLength = 0.1, tendonSlackLength = 0.2, pennationAngle = 0.0, activation = 0.0001, deactivation = 1.0;
	Thelen2003Muscle muscle("muscle1",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);

	// Path for muscle 1
	muscle.addNewPathPoint("muscle-on-thigh1", osim_thigh, Vec3(0.00500000000000000010, -0.21110000000000001000, 0.02340000000000000100));
	muscle.addNewPathPoint("muscle-on-shank1", osim_shank, Vec3(-0.03009999999999999800, -0.03599999999999999700, 0.02943000000000000100));

	// Add the muscle to the model
	osimModel.addForce(&muscle);

	osimModel.print("P2PMomentArmTest.osim");

	SimTK::State &s = osimModel.initSystem();
	knee.getCoordinateSet()[0].setValue(s, -Pi/2);
	osimModel.computeEquilibriumForAuxiliaryStates(s);

	int result = simulateToCheckMomentArms(osimModel, s);

	osimModel.disownAllComponents();

	return result;
}

//==========================================================================================================
int testMomentArmDefinitionForModel(string filename)
{
	// Load OpenSim model
	Model osimModel(filename);

	// Knee start flexed
	double q0 = -Pi/2;

	SimTK::State &s = osimModel.initSystem();
	
	osimModel.getCoordinateSet()[0].setValue(s, q0);

	if(osimModel.getCoordinateSet().getSize() > 2)
		osimModel.getCoordinateSet()[1].setValue(s, Pi/6);

	((Thelen2003Muscle *)(&osimModel.getForceSet()[0]))->setActivation(s, 0.01);

	osimModel.computeEquilibriumForAuxiliaryStates(s);

	int result = simulateToCheckMomentArms(osimModel, s);

	return result;
}

int main()
{
	testPointToPointMuscleAcrossPinJoint();
	cout << "Point to point muscle across PinJoint: PASSED"  << endl;

	testMomentArmDefinitionForModel("P2PBallJointMomentArmTest.osim");
	cout << "Point to point muscle across BallJoint: PASSED"  << endl;

	testMomentArmDefinitionForModel("P2PBallCustomJointMomentArmTest.osim");
	cout << "Point to point muscle across a ball implemented by CustomJoint: PASSED"  << endl;

	testMomentArmDefinitionForModel("MovingPathPointMomentArmTest.osim");
	cout << "Moving path point across PinJoint: PASSED"  << endl;

	testMomentArmDefinitionForModel("P2PCustomJointMomentArmTest.osim");
	cout << "Point to point muscle across CustomJoint: PASSED" << endl;

	testMomentArmDefinitionForModel("MovingPointCustomJointMomentArmTest.osim");
	cout << "Moving path point across CustomJoint: PASSED"  << endl;

	testMomentArmDefinitionForModel("WrapPathCustomJointMomentArmTest.osim");
	cout << "Path with wrapping across CustomJoint: PASSED"  << endl;
	
	testMomentArmDefinitionForModel("PathOnConstrainedBodyMomentArmTest.osim");
	cout << "Path on constrained body across CustomJoint: PASSED"  << endl;

	return 0;
}
