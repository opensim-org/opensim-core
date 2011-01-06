// testExternalLoads.cpp
// Author:  Ajay Seth
/*
* Copyright (c) 2010, Stanford University. All rights reserved. 
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

#include <iostream>
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}
#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

void addLoadToStorage(Storage &forceStore, Vec3 force, Vec3 point, Vec3 torque)
{
	int nLoads = forceStore.getColumnLabels().getSize()/9;
	string labels[9] = { "forceX", "forceY", "forceZ", "pointX", "pointY", "pointZ","torqueX", "torqueY", "torqueZ"};
	char suffix[2];
	sprintf(suffix, "%d", nLoads); 
	
	Array<string> col_labels;
	col_labels.append("time");
	StateVector dataRow;
	dataRow.setTime(0);
	double data[9];
	for(int i = 0; i<9; i++){
		col_labels.append(labels[i]);
		if(i<3){
			data[i] = force[i]; continue;
		}
		else if(i<6){
			data[i] = point[i-3]; continue;
		}
		else
			data[i] = torque[i-6];
	}

	dataRow.setStates(0, 9, data);

	Storage *forces = NULL;
	Storage tempStore;

	if(nLoads == 0)
		forces = &forceStore;
	else if (nLoads > 0)
		forces = &tempStore;
	else
		throw OpenSim::Exception("addLoadToStorage: ERROR");

	forces->setColumnLabels(col_labels);
	forces->append(dataRow);
	dataRow.setTime(1.0);
	forces->append(dataRow);
	dataRow.setTime(2.0);
	forces->append(dataRow);

	if (nLoads > 0)
		forces->addToRdStorage(forceStore, 0.0, 1.0);
}

void testForce(bool forceIsGlobal=true, bool pointIsGlobal=false, bool applyTorque=false)
{
	Model model("Pendulum.osim");
	State &s = model.initSystem();
	model.updGravityForce().disable(s);
	Storage forceStore;
	// Add an external load
	// Default is: 	_forceIsGlobal=true;
	//				_pointIsGlobal=false;

	// Simulate gravity g=-10, R=0.5 => Acc = -g/R*sin(theta)
	Vec3 applicationPoint(0.0, -0.5, 0);
	addLoadToStorage(forceStore, /* force */ Vec3(0.0, -10., 0.), /* point */ applicationPoint, /* torque*/ Vec3(0, 0, 0));
	forceStore.print("test_external_loads.sto");

	ExternalLoads* extLoads = new ExternalLoads(model);
	OpenSim::Array<std::string> forceFunctionNames("forceX", 1);
	OpenSim::Array<int> colummnCount(applyTorque?9:6, 1);
	OpenSim::Array<std::string> bodyNames("cylinder", 1);
	extLoads->createForcesFromFile("test_external_loads.sto", forceFunctionNames, colummnCount, bodyNames);

	for(int i=0; i<extLoads->getSize(); i++)
		model.addForce(&(*extLoads)[i]);

	// Create the force reporter
	ForceReporter* reporter = new ForceReporter(&model);
	model.addAnalysis(reporter);

	Kinematics* kin = new Kinematics(&model);
	kin->setInDegrees(true);
	kin->setRecordAccelerations(true);
	model.addAnalysis(kin);

	//PointKinematics* pKin = new PointKinematics(&model);
	//pKin->setBody(&model.updBodySet().get("cylinder"));
	//pKin->setPoint(applicationPoint);
	//model.addAnalysis(pKin);
	// set initial conditions
	CoordinateSet& pin_coords = model.getJointSet().get(0).getCoordinateSet();
	pin_coords[0].setValue(s, SimTK_PI/4.0);
	
	SimTK::State &osim_state = model.initSystem();
    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem() );
	integrator.setAccuracy(1e-6);
    Manager manager(model,  integrator);
    manager.setInitialTime(0.0);

	double final_t = 2.0;
	double nsteps = 10;
	double dt = final_t/nsteps;

	const OpenSim::Body& cylBody=model.getBodySet().get("cylinder");
	for(int i = 1; i <=nsteps; i++){
		manager.setFinalTime(dt*i);
		manager.integrate(osim_state);	
		model.getMultibodySystem().realize(osim_state, Stage::Acceleration);
		manager.setInitialTime(dt*i);
	}
	std::string desc = "Force"+ std::string(forceIsGlobal?"Global":"Local")+
						"Point"+std::string(pointIsGlobal?"Global":"Local")+
						std::string(applyTorque?"With":"No")+"Torque";
	reporter->printResults(desc);
	kin->printResults(desc);
	 
}

int main()
{
	testForce(true, false, false);
	
	return 0;
}
