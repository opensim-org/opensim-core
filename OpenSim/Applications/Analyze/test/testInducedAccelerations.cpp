// testInducedAccelerations.cpp
// Author: Ajay Seth
/*
* Copyright (c)  2011, Stanford University. All rights reserved. 
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

// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/Common/IO.h>
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
		Storage result1("ResultsInducedAccelerations/subject02_running_arms_InducedAccelerations_center_of_mass.sto"), standard1("std_subject02_running_arms_InducedAccelerations_center_of_mass.sto");
		result1.checkAgainstStandard(standard1, Array<double>(0.15, result1.getSmallestNumberOfStates()), __FILE__, __LINE__, "Induced Accelerations of Running failed");
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