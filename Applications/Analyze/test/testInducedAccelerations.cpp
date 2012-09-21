/* -------------------------------------------------------------------------- *
 *                   OpenSim:  testInducedAccelerations.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

// INCLUDE
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

// Prototypes
void testDoublePendulum();
Vector calcDoublePendulumUdot(const Model &model, State &s, double Torq1, double Torq2, bool gravity, bool velocity);

int main()
{
	try {
		// First case is a simple torque driven double pendulum model
		// Tool results compared directly Simbody model computed results
		testDoublePendulum();

		AnalyzeTool analyze("subject02_Setup_IAA_02_232.xml");
		analyze.run();
		Storage result1("ResultsInducedAccelerations/subject02_running_arms_InducedAccelerations_center_of_mass.sto"), standard1("std_subject02_running_arms_InducedAccelerations_CENTER_OF_MASS.sto");
		CHECK_STORAGE_AGAINST_STANDARD(result1, standard1, Array<double>(0.15, result1.getSmallestNumberOfStates()), __FILE__, __LINE__, "Induced Accelerations of Running failed");
		cout << "Induced Accelerations of Running passed\n" << endl;
	}
	catch (const OpenSim::Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}


void testDoublePendulum()
{
	AnalyzeTool analyze("double_pendulum_Setup_IAA.xml");
	analyze.run();
	Storage statesStore("double_pendulum_states.sto");
	Array<double> time;
	Array< Array<double> > states;

	int nt = statesStore.getTimeColumn(time);
	statesStore.getDataForIdentifier("q", states);

	Storage q1_iaa("ResultsInducedAccelerations/double_pendulum_InducedAccelerations_q1.sto");
	Storage q2_iaa("ResultsInducedAccelerations/double_pendulum_InducedAccelerations_q2.sto");

	Array<double> u1dot_grav, u2dot_grav, u1dot_vel, u2dot_vel;
	Array<double> u1dot_torq1, u2dot_torq1, u1dot_torq2, u2dot_torq2;
	q1_iaa.getDataColumn("gravity", u1dot_grav);
	q2_iaa.getDataColumn("gravity", u2dot_grav);
	q1_iaa.getDataColumn("velocity", u1dot_vel);
	q2_iaa.getDataColumn("velocity", u2dot_vel);
	q1_iaa.getDataColumn("Torq1", u1dot_torq1);
	q2_iaa.getDataColumn("Torq1", u2dot_torq1);
	q1_iaa.getDataColumn("Torq2", u1dot_torq2);
	q2_iaa.getDataColumn("Torq2", u2dot_torq2);

	Model pendulum("double_pendulum.osim");
	State &s = pendulum.initSystem();

	for(int i=0; i<nt; ++i){
		Vector &q = s.updQ();
		Vector &u = s.updU();
		q[0]= (states[0])[i];
		q[1]= (states[1])[i];
		u[0]= (states[2])[i];
		u[1]= (states[3])[i];

		// Compute velocity contribution 
		// velocity first, since other contributors set u's to zero and the state is not restored until next iteration. 
		Vector udot = calcDoublePendulumUdot(pendulum, s, 0, 0, false, true);
		ASSERT_EQUAL(udot[0], u1dot_vel[i], 1e-5, __FILE__, __LINE__, "Induced Accelerations of velocity for double pendulum q1 FAILED");
		ASSERT_EQUAL(udot[1], u2dot_vel[i], 1e-5, __FILE__, __LINE__, "Induced Accelerations of velocity for double pendulum q2 FAILED");

		// Compute gravity contribution
		udot = calcDoublePendulumUdot(pendulum, s, 0, 0, true, false);
		ASSERT_EQUAL(udot[0], u1dot_grav[i], 1e-5, __FILE__, __LINE__, "Induced Accelerations of gravity for double pendulum q1 FAILED");
		ASSERT_EQUAL(udot[1], u2dot_grav[i], 1e-5, __FILE__, __LINE__, "Induced Accelerations of gravity for double pendulum q2 FAILED");

		// Compute Torq1 contribution
		udot = calcDoublePendulumUdot(pendulum, s, 0.75, 0, false, false);
		ASSERT_EQUAL(udot[0], u1dot_torq1[i], 1e-5, __FILE__, __LINE__, "Induced Accelerations of Torq1 for double pendulum q1 FAILED");
		ASSERT_EQUAL(udot[1], u2dot_torq1[i], 1e-5, __FILE__, __LINE__, "Induced Accelerations of Torq1 for double pendulum q2 FAILED");

		// Compute Torq1 contribution
		udot = calcDoublePendulumUdot(pendulum, s, 0, 0.50, false, false);
		ASSERT_EQUAL(udot[0], u1dot_torq2[i], 1e-5, __FILE__, __LINE__, "Induced Accelerations of Torq2 for double pendulum q1 FAILED");
		ASSERT_EQUAL(udot[1], u2dot_torq2[i], 1e-5, __FILE__, __LINE__, "Induced Accelerations of Torq2 for double pendulum q2 FAILED");
	}
	cout << "Induced Accelerations of double pendulum passed\n" << endl;
}


Vector calcDoublePendulumUdot(const Model &model, State &s, double Torq1, double Torq2, bool gravity, bool velocity)
{	
	if(gravity)
		model.getGravityForce().enable(s);
	else
		model.getGravityForce().disable(s);

	
	if(!velocity)
		s.updU() = 0.0;

	MultibodySystem &sys = model.updMultibodySystem();
	Vector &mobilityForces = sys.updMobilityForces(s, Stage::Dynamics);
	sys.realize(s, Stage::Dynamics);

	mobilityForces[0] = Torq1;
	mobilityForces[1] = Torq2;

	sys.realize(s, Stage::Acceleration);

	return s.getUDot();
}
