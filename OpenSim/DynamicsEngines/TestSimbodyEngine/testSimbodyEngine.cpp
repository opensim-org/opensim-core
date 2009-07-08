// testSimbodyEngine.cpp
// Author:  Frank C. Anderson
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

//==============================================================================
//==============================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/GeneralizedForce.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/CustomJoint.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/TransformAxis.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/WeldConstraint.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Common/NatCubicSpline.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

// Helper class to construct bounds functions, which returns non-zero values for values
// outside the min and max range
class RangeFunction : public SimTK::Function<1> {
// returns q-qmax for q > qmax and q-qmin for q < qmin
private:
	const double _qmin;
	const double _qmax;

public:
	
	RangeFunction(const double qmin, const double qmax) : _qmin(qmin), _qmax(qmax){
	}

    Vec<1> calcValue(const Vector& x) const {
		double q = x[0];
		Vec1 val(((q-_qmin)-abs(q-_qmin))/2 + ((q-_qmax)+abs(q-_qmax))/2);
		return val;
    }
    Vec<1> calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const {
		if (derivComponents.size() == 1){
			return calcValue(x)/x.norm();
		}
        return Vec1(0);
    }

    int getArgumentSize() const {
        return 1;
    }
    int getMaxDerivativeOrder() const {
        return 2;
    }
};



//______________________________________________________________________________
/**
 * Run a simulation of a pendulum with Simbody as the underlying dynamics
 * engine.
 */
int main()
{
#ifndef STATIC_OSIM_LIBS
	//Make sure that the SimbodyEngine library is available
	LoadOpenSimLibrary("osimSimbodyEngine");
#endif

	// Set output precision
	IO::SetPrecision(10);

	// Test Model reading with constraints
	Model gaitModel("gait2354_simbody_patella_loop.osim");
	gaitModel.setup();
	gaitModel.printDetailedInfo(cout);

	
	// Add prescribed motion to the knees
	double t_k[] = {0.0000, 0.1000, 0.2000, 0.3000, 0.4000, 0.5000, 0.6000, 0.7000, 0.8000, 0.9000, 1.0000};
	double ang_k[] = {-1.5708, -0.6475, -0.0769, -0.0769, -0.6475, -1.5708, -2.4941, -3.0647, -3.0647, -2.4941, -1.5708};
	NatCubicSpline *knee_func = new NatCubicSpline(10, t_k, ang_k);
	//Coordinate *knee_r = dynamic_cast<Coordinate *>(gaitModel.getDynamicsEngine().getCoordinateSet()->get("knee_angle_r"));
	//knee_r->setPrescribedFunction(knee_func);
	
	Coordinate *knee_l = dynamic_cast<Coordinate *>(gaitModel.getDynamicsEngine().getCoordinateSet()->get("knee_angle_l"));
	knee_l->setPrescribedFunction(knee_func);
	
	//// Write to latest model file format
	//gaitModel.copy()->print("gait2354_simbody_patella_out.osim");

	// First weld the pelvis so model is not flying through space
	//Vec3 locInGround(0.0, 0.75, 0);
	//Vec3 locInPelvis(0.0);
	//WeldConstraint *aConstraint = new WeldConstraint();
	//aConstraint->setBody1ByName("ground");
	//aConstraint->setBody1WeldLocation(locInGround);
	//aConstraint->setBody2ByName("pelvis");
	//aConstraint->setBody2WeldLocation(locInPelvis);
	//gaitModel.getDynamicsEngine().getConstraintSet()->append(aConstraint);
	gaitModel.getDynamicsEngine().setup(&gaitModel);

	//Now that model is setup we should be to change the state of things
	//knee_r->setIsPrescribed(true);
	knee_l->setIsPrescribed(true);

	bool isConstrained = gaitModel.getDynamicsEngine().getCoordinateSet()->get("pelvis_tx")->isConstrained();
	cout<<"Pelvis_tx is constrained: "<< isConstrained <<endl;
	isConstrained = gaitModel.getDynamicsEngine().getCoordinateSet()->get("tib_tx_r")->isConstrained();
	cout<<"Tibia_tx is constrained: "<< isConstrained <<endl;
	isConstrained = gaitModel.getDynamicsEngine().getCoordinateSet()->get("pat_angle_r")->isConstrained();
	cout<<"Patella rotation is constrained: "<< isConstrained <<endl;

	// Set the initial states
	OpenSim::Array<double> yi(0.0,gaitModel.getNumStates());
	gaitModel.getInitialStates(&yi[0]);

	int nq = gaitModel.getDynamicsEngine().getCoordinateSet()->getSize();

	// get right knee angle index 
	int ind_r = gaitModel.getDynamicsEngine().getCoordinateSet()->getIndex("knee_angle_r"); 

	yi[ind_r] = -2.09439510/2; // angle
	// Provide initial states that satisfy the constraints
	yi[1] = 0.93;
	yi[nq+1] = 1;

	//Left knee index
	int ind_l = gaitModel.getDynamicsEngine().getCoordinateSet()->getIndex("knee_angle_l");
	yi[ind_l]= yi[ind_r];

	// initial knee angular velocities
	yi[ind_l+nq]= yi[ind_r+nq] = -1.0;


	//gaitModel.getDynamicsEngine().getCoordinateSet()->get("pelvis_ty")->setLocked(true);
	//gaitModel.getDynamicsEngine().getCoordinateSet()->get("knee_angle_r")->setDefaultValue(-2.09439510);
	//gaitModel.getDynamicsEngine().getCoordinateSet()->get("knee_angle_r")->setLocked(true);

	bool isLocked = gaitModel.getDynamicsEngine().getCoordinateSet()->get("pelvis_tx")->getLocked();
	cout<<"Pelvis_tx is locked: "<< isLocked <<endl;
	isLocked = gaitModel.getDynamicsEngine().getCoordinateSet()->get("pelvis_ty")->getLocked();
	cout<<"Pelvis_ty is locked: "<< isLocked <<endl;
	isLocked = gaitModel.getDynamicsEngine().getCoordinateSet()->get("knee_angle_r")->getLocked();
	cout<<"Knee_angle_r is locked: "<< isLocked <<endl;


	// Add analyses to the model->
	// These analyses will gather information during a simulation
	// without altering the simulation.
	// stepInterval specifies how frequently analyses will record info,
	// every four integration steps in this case.
	int stepInterval = 4;

	// Point Kinematics
	Vec3 point(0.0);
	BodySet *bodySet = gaitModel.getDynamicsEngine().getBodySet();
	int nb = bodySet->getSize();

	OpenSim::Array<std::string> tibias;
	tibias.append("tibia_r");
	tibias.append("tibia_l");

	for(int i=0; i<tibias.getSize(); i++) {
		PointKinematics *pointKin = new PointKinematics(&gaitModel);
		AbstractBody *body = bodySet->get(tibias.get(i));
		pointKin->setBodyPoint(body->getName(),point);
		pointKin->setPointName(body->getName().c_str());
		gaitModel.addAnalysis(pointKin);
	}
	Kinematics *modelKin = new Kinematics(&gaitModel);
	gaitModel.addAnalysis(modelKin);

	//Use DynamicsEngine to find initial values for constrained coupled-coordindates
	gaitModel.getDynamicsEngine().computeConstrainedCoordinates(&yi[0]);
	cout<<"\nInitial conditions:\n "<<yi<<endl;
	gaitModel.setInitialStates(&yi[0]);

	Vec3 grav; 
	gaitModel.getDynamicsEngine().getGravity(grav);
	cout << "gravity = " << grav << endl;

	SimTK::Vector_<Vec3> rForces, rTorques;
	rForces.resize(15);
	rTorques.resize(15);
	gaitModel.getDynamicsEngine().computeReactions(rForces, rTorques);

	gaitModel.getDynamicsEngine().getConfiguration(&yi[0]);
	cout<<"\nSimbody initial states:\n "<<yi<<endl;

	// Construct the integrand and the manager.
	// The model2 integrand is what is integrated during the simulation.
	// The manager takes care of a variety of low-level initializations.
	ModelIntegrand *integrand = new ModelIntegrand(&gaitModel);
	Manager manager(integrand);

	// Specify the initial and final times of the simulation.
	// In this case, the initial and final times are set based on
	// the range of times over which the controls are available.
	//Control *control;
	double ti=0.0,tf=1.0;
	manager.setInitialTime(ti);
	manager.setFinalTime(tf);

	// Set up the numerical integrator.
	int maxSteps = 50000;
	IntegRKF *integ = manager.getIntegrator();
	integ->setMaximumNumberOfSteps(maxSteps);
	integ->setMaxDT(1.0e-1);
	integ->setTolerance(1.0e-5);
	integ->setFineTolerance(1.0e-8);

	// Integrate
	cout<<"\n\nIntegrating from "<<ti<<" to "<<tf<<endl;
	manager.integrate();

	// Print the analysis results.
	gaitModel.getAnalysisSet()->printResults("test","./Results");

	gaitModel.print("gait2354_simbody_patella_out.osim");

	return 0;
}
