// testJoints.cpp
// Author:  Ajay Seth
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

//==========================================================================================================
//	testJoints builds OpenSim models using the OpenSim API and builds an equivalent
//  Simbody system using the Simbody API for each test case. A test fails if the
//  OpenSim and Simbody final states of the simulation are not equivelent (norm-err
//  less than 10x integration error tolerance)
//
//	Tests Include:
//      1. CustomJoint against Simbody built-in Pin and Universal mobilizers
//      2. CustomJoint versus Simbody FunctionBased with spline based functions
//		3. EllipsodJoint against Simbody built-in Ellipsoid mobilizer
//		4. WeldJoint versus Weld Mobilizer by welding bodies to those in test 1.
//		5. Randomized order of bodies in the BodySet (in 3.) to test connectBodies()
//		6. PinJoint against Simbody built-in Pin mobilizer
//		7. SliderJoint against Simbody built-in Pin mobilizer
//		8. FreeJoint against Simbody built-in Free mobilizer
//		9. BallJoint against Simbody built-in Ball mobilizer
//		
//     Add tests here as new joint types are added to OpenSim
//
//==========================================================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Simulation/Model/Model.h>

#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/EllipsoidJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/BallJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Common/NaturalCubicSpline.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/FunctionAdapter.h>
#include "SimTKsimbody.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==========================================================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-5;
const static double duration = 1.00;
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
const static Vec3 kneeInFemur(0.0033, -0.2294, 0);
const static Vec3 kneeInTibia(0.0, 0.1862, 0.0);
const Vec3 ankleInTibia(0.0, -0.243800, 0);
const Vec3 ankleInFoot(-0.035902, 0.051347, 0);
const Vec3 mtpInFoot(0.098032, -0.038000, 0);
const Vec3 mtpInToes(-0.035902, 0.051347, 0);
//==========================================================================================================

class MultidimensionalFunction : public OpenSim::Function
{
public:
	MultidimensionalFunction() {};

	virtual ~MultidimensionalFunction() {};

	virtual Object* copy() const { return new MultidimensionalFunction; }

	virtual double calcValue(const SimTK::Vector& x) const
	{
		return 2*x[0]*x[0] + x[1];
	}
	virtual double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const
	{
	   int nd = derivComponents.size();
	   if (nd < 1)
		   return SimTK::NaN;
	
	   if (derivComponents[0] == 0){
			if (nd == 1)
				return 4*x[0];
			else if(derivComponents[0] == 0)
				return 4;
	   }
	   else if (derivComponents[0] == 1){
		   if (nd == 1)
			   return 1;
	   }
	   return 0;
	}

	virtual int getArgumentSize() const {return 2;}
	virtual int getMaxDerivativeOrder() const { return 2;}

	virtual SimTK::Function* createSimTKFunction() const
	{
		return new FunctionAdapter(*this);
	}
}; // End of MultidimensionalFunction



//==========================================================================================================
// Common Functions 
//==========================================================================================================
int initTestStates(Vector &qi, Vector &ui)
{
	Random::Uniform randomAngle(-Pi/4, Pi/4);
	Random::Uniform randomSpeed(-1.0, 1.0);

	// Provide initial states as random angles and speeds for OpenSim and Simbody models
	for(int i = 0; i<qi.size(); i++)
		qi[i] = randomAngle.getValue();

	for(int i = 0; i<ui.size(); i++)
		ui[i] = randomSpeed.getValue();
    
	return qi.size();
}

void integrateSimbodySystem(MultibodySystem &system, SimTK::State &state)
{
	// realize simbody system to velocity stage
	system.realize(state, Stage::Velocity);

	RungeKuttaFeldbergIntegrator integ(system);
	integ.setAccuracy(integ_accuracy);
	integ.setAbsoluteTolerance(integ_accuracy);
	
    TimeStepper ts(system, integ);
    ts.initialize(state);
	ts.stepTo(duration);
	state = ts.getState();
}

void integrateOpenSimModel(Model *osimModel, SimTK::State &osim_state)
{
	// SETUP OpenSim SIMULATION Manager
	osimModel->getSystem().realize(osim_state, Stage::Velocity);
    RungeKuttaFeldbergIntegrator integrator(osimModel->getSystem() );
	integrator.setAccuracy(integ_accuracy);
	integrator.setAbsoluteTolerance(integ_accuracy);
    Manager manager(*osimModel, integrator);

	// Specify the initial and final times of the simulation.
	// In this case, the initial and final times are set based on
	// the range of times over which the controls are available.
	//Control *control;
	manager.setInitialTime(0.0);
	manager.setFinalTime(duration);

	// Integrate
    const SimbodyMatterSubsystem& matter2 = osimModel->getMultibodySystem().getMatterSubsystem();
 //   for (int i = 0; i < matter2.getNumConstraints(); i++)
 //       printf("%d: %d\n", i, matter2.isConstraintDisabled(osim_state, SimTK::ConstraintIndex(i)));
 //   cout << osim_state.getQ()<<endl;
	//cout << "\n\nOpenSim Integration 0.0 to " << duration << endl;

	manager.integrate(osim_state);
}

bool compareSimulationStates(Vector q_sb, Vector u_sb, Vector q_osim, Vector u_osim)
{
    bool status = true;
	
	Vector q_err = q_osim;

	int nq = q_osim.size();
	if(q_sb.size() > nq){ // we have an unused quaternion slot in Simbody

		q_sb.dump("Simbody q's:");
		q_osim.dump("OpenSim q's:");
		//This is a hack knowing the free and ball joint tests have the quaternion joint first
		// And that the q's are packed as qqqq or aaa* for a ball and qqqqxyz or aaaxyz* for a free joint
		int quat_ind = ((nq > 6) ? 6 : 3);
		int j = 0;
		for(int i=0; i< q_sb.size(); i++){
			if(i != quat_ind){
				q_err[j] = q_sb[i] - q_osim[j];
				j++;
			}
		}
	}
	else{
		q_err = q_sb - q_osim;
	}
    
	Vector u_err = u_sb - u_osim;

	//cout<<"\nSimbody - OpenSim:"<<endl;
	//q_err.dump("Diff q's:");
	//u_err.dump("Diff u's:");

	if(q_err.norm() > 10*integ_accuracy) {
         cout<<"testJoints compareSimulationStates failed q_err.norm = "<< q_err.norm() << endl;
         status = false;
     }
	if(u_err.norm() > 10*integ_accuracy) {
         cout<<"testJoints compareSimulationStates failed u_err.norm = "<< u_err.norm() << endl;
         status = false;
    }

    return(status);
}

bool compareSimulations(MultibodySystem &system, SimTK::State &state, Model *osimModel, SimTK::State &osim_state)
{
	// Set the initial states for both Simbody system and OpenSim model
	Vector& qi = state.updQ();
	Vector& ui = state.updU();
	int nq_sb = initTestStates(qi, ui);
	int nq = osim_state.getNQ();

	// Push down to OpenSim "state"
	if(system.getMatterSubsystem().getUseEulerAngles(state)){
		const Vector& y = state.getY();
		Vector y_osim(2*nq, 0.0);
		//This is a hack knowing the ball and free joint tests have the joint being tested, first.
		int quat_ind = ((nq > 6) ? 6 : 3);
		int j = 0;
		for(int i=0; i< state.getNY(); i++){
			if(i != quat_ind)
				y_osim[j++] = y[i];
		}
		osim_state.updY() = y_osim;

		y_osim.dump("osim:");
		y.dump("simbody:");
	}
	else{
		osim_state.updY() = state.getY();
	}

	//==========================================================================================================
	// Integrate Simbody system
	integrateSimbodySystem(system, state);

	// Simbody model final states
	qi = state.updQ();
	ui = state.updU();

	//qi.dump("\nSimbody Final q's:");
	//ui.dump("\nSimbody Final u's:");

	//==========================================================================================================
	// Integrate OpenSim model
	integrateOpenSimModel(osimModel, osim_state);

	// Get the state at the end of the integration from OpenSim.
	Vector& qf = osim_state.updQ();
	Vector& uf = osim_state.updU();
	//cout<<"\nOpenSim Final q's:\n "<<qf<<endl;
	//cout<<"\nOpenSim Final u's:\n "<<uf<<endl;

	//==========================================================================================================
	// Compare Simulation Results
	return( compareSimulationStates(qi, ui, qf, uf) );
}
//==========================================================================================================

//==========================================================================================================
// Test Cases
//==========================================================================================================
bool testCustomVsUniversalPin()
{
	cout << endl;
	cout << "==========================================================" << endl;
	cout << " OpenSim CustomJoint vs. Simbody Universal and Pin Joints " << endl;
	cout << "==========================================================" << endl;
	
	// Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
	SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

	// Thigh connected by hip
	MobilizedBody::Universal thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur));
	//Function-based knee connects shank
	MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, false);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model osimModel;

	//OpenSim bodies
    OpenSim::Body& ground = osimModel.getGroundBody();
	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// Define hip transform in terms of coordinates and axes for custom joint
	SpatialTransform hipTransform;
	hipTransform[0].setCoordinateNames(OpenSim::Array<std::string>("hip_q0", 1, 1));
	hipTransform[0].setFunction(new LinearFunction());
	hipTransform[1].setCoordinateNames(OpenSim::Array<std::string>("hip_q1", 1, 1));
	hipTransform[1].setFunction(new LinearFunction());

	// create custom hip joint
	CustomJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), hipTransform);

	// Add the thigh body which now also contains the hip joint to the model
	osimModel.addBody(&osim_thigh);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee coordinates and axes for custom joint spatial transform
	SpatialTransform kneeTransform;
	string knee_rot = "knee_ext";
	// Only knee flexion/extension
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>(knee_rot, 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());

	// create custom knee joint
	CustomJoint knee("", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	osimModel.addBody(&osim_shank);

	// Set gravity
	osimModel.setGravity(gravity_vec);

	//Add analyses before setting up the model for simulation
	Kinematics *kinAnalysis = new Kinematics(&osimModel);
	kinAnalysis->setInDegrees(false);
	osimModel.addAnalysis(kinAnalysis);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel.initSystem();
	osimModel.disownAllComponents();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	return (compareSimulations(system, state, &osimModel, osim_state));

} //end of testCustomVsUniversalPin


bool testCustomJointVsFunctionBased()
{
	cout << endl;
	cout << "==========================================================" << endl;
	cout << " OpenSim CustomJoint vs. Simbody FunctionBased Mobilizer  " << endl;
	cout << "==========================================================" << endl;

	// Define spline data for the custom knee joint
	int npx = 12;
	double angX[] = {-2.094395102393, -1.745329251994, -1.396263401595, -1.047197551197, -0.698131700798, -0.349065850399, -0.174532925199, 0.197344221443, 0.337394955864, 0.490177570472, 1.521460267071, 2.094395102393};
	double kneeX[] = {-0.003200000000, 0.001790000000, 0.004110000000, 0.004100000000, 0.002120000000, -0.001000000000, -0.003100000000, -0.005227000000, -0.005435000000, -0.005574000000, -0.005435000000, -0.005250000000};
	int npy = 7;
	double angY[] = {-2.094395102393, -1.221730476396, -0.523598775598, -0.349065850399, -0.174532925199, 0.159148563428, 2.094395102393};
	double kneeY[] = {-0.422600000000, -0.408200000000, -0.399000000000, -0.397600000000, -0.396600000000, -0.395264000000, -0.396000000000 };


	for(int i = 0; i<npy; i++) {
		// Spline data points from experiment w.r.t. hip location. Change to make it w.r.t knee location
		kneeY[i] += (-kneeInFemur[1]+hipInFemur[1]); 
	}

	NaturalCubicSpline tx(npx, angX, kneeX);
	NaturalCubicSpline ty(npy, angY, kneeY);;

	// Define the functions that specify the FunctionBased Mobilized Body.
	std::vector<std::vector<int> > coordIndices;
	std::vector<const SimTK::Function*> functions;
	std::vector<bool> isdof(6,false);

	// Set the 1 spatial rotation about Z-axis
	isdof[2] = true;  //rot Z
	int nm = 0;
	for(int i=0; i<6; i++){
		if(isdof[i]) {
			Vector coeff(2);
			coeff[0] = 1;
			coeff[1] = 0;
			std::vector<int> findex(1);
			findex[0] = nm++;
			functions.push_back(new SimTK::Function::Linear(coeff));
			coordIndices.push_back(findex);
		}
		else if(i==3 || i ==4){
			std::vector<int> findex(1,0);
			if(i==3)
				functions.push_back(tx.createSimTKFunction());
			else
				functions.push_back(ty.createSimTKFunction());

			coordIndices.push_back(findex);	
		}
		else{
			std::vector<int> findex(0);
			functions.push_back(new SimTK::Function::Constant(0, 0));
			coordIndices.push_back(findex);
		}
	}

	// Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
	SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);
	//system.updDefaultSubsystem().addEventReporter(new VTKEventReporter(system, 0.01));

	// Thigh connected by hip
	MobilizedBody::Pin thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur));
	//Function-based knee connects shank
	MobilizedBody::FunctionBased shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia), nm, functions, coordIndices);
	//MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, false);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();
	
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);
	//OpenSim::Body osim_thigh;
	//osim_thigh.setName("thigh");

	// Define hip coordinates and axes for custom joint
	SpatialTransform hipTransform;
	hipTransform[2].setCoordinateNames(OpenSim::Array<std::string>("hip_q0", 1, 1));
	hipTransform[2].setFunction(new LinearFunction());

	// create custom hip joint
	CustomJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), hipTransform);

	// Add the thigh body which now also contains the hip joint to the model
	osimModel->addBody(&osim_thigh);

	// Add another body via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee coordinates and axes for custom joint spatial transform
	SpatialTransform kneeTransform;
	string coord_name = "knee_q";
	//  knee flexion/extension
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>(coord_name, 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());
	// translation X
	kneeTransform[3].setCoordinateNames(OpenSim::Array<std::string>(coord_name, 1, 1));
	kneeTransform[3].setFunction(tx);
	// translation Y
	kneeTransform[4].setCoordinateNames(OpenSim::Array<std::string>(coord_name, 1, 1));
	kneeTransform[4].setFunction(ty);

	// create custom knee joint
	CustomJoint knee("", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	osimModel->addBody(&osim_shank);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	//Add analyses before setting up the model for simulation
	Kinematics *kinAnalysis = new Kinematics(osimModel);
	kinAnalysis->setInDegrees(false);
	osimModel->addAnalysis(kinAnalysis);

	// OpenSim model must realize the topology to get valid osim_state
	osimModel->setGravity(gravity_vec);

	PointKinematics *pointKin = new PointKinematics(osimModel);
	// Get the point location of the shank origin in space
	pointKin->setBodyPoint("shank", Vec3(0));
	osimModel->addAnalysis(pointKin);

	// write out the model to file
	osimModel->print("testCustomJoint.osim");

	//wipe-out the model just constructed
	delete osimModel;

	// reconstruct from the model file
	osimModel = new Model("testCustomJoint.osim");
	
	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel->initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	return( compareSimulations(system, state, osimModel, osim_state) );
}

bool testEllipsoidJoint()
{
	cout << endl;
	cout << "=============================================================" << endl;
	cout << " OpenSim EllipsoidJoint vs. Simbody MobilizedBody::Ellipsoid " << endl;
	cout << "=============================================================" << endl;
	
	//Ellipsoid radii
	Vec3 ellipsoidRadii(0.5, 0.33, 0.25);

	// Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
	SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

	// Thigh connected by hip
	MobilizedBody::Ellipsoid thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur));
	//Function-based knee connects shank
	MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

	thigh.setDefaultRadii(ellipsoidRadii);
	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, false);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create hip as an ellipsoid joint
	EllipsoidJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), ellipsoidRadii);

	// Rename hip coordinates for an ellipsoid joint
	CoordinateSet& hip_coords = hip.getCoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords[i].setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel->addBody(&osim_thigh);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee coordinates and axes for custom joint spatial transform
	SpatialTransform kneeTransform;
	string knee_q = "knee_q";
	// Only knee flexion/extension
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>(knee_q, 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());

	// create custom knee joint
	CustomJoint knee("", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	osimModel->addBody(&osim_shank);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	osimModel->setGravity(gravity_vec);

	//Write model to file
	osimModel->print("testEllipsoidJoint.osim");

	//Wipe out model
	delete osimModel;

	//Load model from file
	osimModel = new Model("testEllipsoidJoint.osim");

    //Add analyses before setting up the model for simulation
	Kinematics *kinAnalysis = new Kinematics(osimModel);
	kinAnalysis->setInDegrees(false);
	osimModel->addAnalysis(kinAnalysis);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel->initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	return (compareSimulations(system, state, osimModel, osim_state));

} // end testEllipsoidJoint


bool testWeldJoint(bool randomizeBodyOrder)
{
	cout << endl;
	cout << "================================================================" << endl;
	cout << "  OpenSim WeldJoint vs. Simbody's Weld Mobilizer " << endl;
	cout << "================================================================" << endl;
	
	// Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
	SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

	// Thigh connected by hip
	MobilizedBody::Universal thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur));
	//Function-based knee connects shank
	MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));
	// Weld foot to shank at ankle
	MobilizedBody::Weld foot(shank, SimTK::Transform(ankleInTibia), SimTK::Body::Rigid(footMass), SimTK::Transform(ankleInFoot));
	// fixed toes at right mtp 
	MobilizedBody::Weld toes(foot, SimTK::Transform(mtpInFoot), SimTK::Body::Rigid(toesMass), SimTK::Transform(mtpInToes));

	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, false);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// Use a temporary BodySet to hold bodies
	BodySet tempBodySet;
	tempBodySet.setMemoryOwner(false);

	// Define hip coordinates and axes for custom joint
	SpatialTransform hipTransform;
	hipTransform[0].setCoordinateNames(OpenSim::Array<std::string>("hip_q0", 1, 1));
	hipTransform[0].setFunction(new LinearFunction());
	hipTransform[1].setCoordinateNames(OpenSim::Array<std::string>("hip_q1", 1, 1));
	hipTransform[1].setFunction(new LinearFunction());	

	// create custom hip joint
	CustomJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), hipTransform);
	
	tempBodySet.append(&osim_thigh);

	// Add another body via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee transform for flexion/extension
	SpatialTransform kneeTransform;
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>("knee_q", 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());	

	// create custom knee joint
	CustomJoint knee("", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	//osim_shank.setJoint((Joint *)&knee);
	tempBodySet.append(&osim_shank);

	// Add foot body at ankle
	OpenSim::Body osim_foot("foot", footMass.getMass(), footMass.getMassCenter(), footMass.getInertia());
	WeldJoint ankle("", osim_shank, ankleInTibia, Vec3(0), osim_foot, ankleInFoot, Vec3(0));

	tempBodySet.append(&osim_foot);

	// Add toes body at mtp
	OpenSim::Body osim_toes ("toes", toesMass.getMass(), toesMass.getMassCenter(), toesMass.getInertia());
	WeldJoint mtp("", osim_foot, mtpInFoot, Vec3(0), osim_toes, mtpInToes, Vec3(0));

	osim_toes.setJoint(mtp);
	tempBodySet.append(&osim_toes);

	int order[] = {0, 1, 2, 3};
	if(randomizeBodyOrder){
		cout << " Randomizing Bodies to exercise SimbodyEngine:: connectBodies() " << endl;
		cout << "================================================================" << endl;
		Random::Uniform randomOrder(0, 4);
		randomOrder.setSeed(clock());
		order[0] = randomOrder.getIntValue();
		int index;
		for(int i=1; i<4; i++){
			bool found = true;
			// Keep sampling random numbers until index not found in the list
			while(found){
				index = randomOrder.getIntValue();
				found = false;
				// check if we can find this index in the order list already
				for(int j=0; j<i; j++)
					// if find a match, stop and try again for another
					if (index==order[j]){
						found = true;
						break;
					}
			}	
			order[i] = index;
		}

	}

	for(int i=0; i<4; i++){
		osimModel->addBody(&tempBodySet.get(order[i]));
	}

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	// OpenSim model must realize the topology to get valid osim_state
	osimModel->setGravity(gravity_vec);

	SimTK::State osim_state = osimModel->initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	return( compareSimulations(system, state, osimModel, osim_state) );
}

bool testFreeJoint(bool useEulerAngles)
{
	cout << endl;
	cout << "=============================================================" << endl;
	cout << " OpenSim FreeJoint vs. Simbody MobilizedBody::Free " << endl;
	cout << "=============================================================" << endl;
	
	// Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
	SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

	// Thigh connected by hip
	MobilizedBody::Free thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur));
	//Function-based knee connects shank
	MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, useEulerAngles);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create free hip joint
	FreeJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), useEulerAngles);

	// Rename hip coordinates for a free joint
	CoordinateSet& hip_coords = hip.getCoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords[i].setName(coord_name.str());
		hip_coords[i].setMotionType(((i<3) ? Coordinate::Rotational : Coordinate::Translational));
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel->addBody(&osim_thigh);

	// create OpenSim shank body
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee transform for flexion/extension
	SpatialTransform kneeTransform;
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>("knee_q", 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());

	// create custom knee joint
	CustomJoint knee("", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	osimModel->addBody(&osim_shank);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	osimModel->setGravity(gravity_vec);

    //Add analyses before setting up the model for simulation
	Kinematics *kinAnalysis = new Kinematics(osimModel);
	kinAnalysis->setInDegrees(false);
	osimModel->addAnalysis(kinAnalysis);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel->initSystem();

	cout << "NQ_osim = " << osim_state.getNQ() << "   NQ_simbody = " << state.getNQ() << endl;

	osimModel->print("testFreeJoint.osim");

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	return (compareSimulations(system, state, osimModel, osim_state));

} // end testFreeJoint


bool testBallJoint(bool useEulerAngles)
{
	cout << endl;
	cout << "=============================================================" << endl;
	cout << " OpenSim BallJoint vs. Simbody MobilizedBody::Ball " << endl;
	cout << "=============================================================" << endl;

	// Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
	SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

	// Thigh connected by hip
	MobilizedBody::Ball thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur));
	//Function-based knee connects shank
	MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, useEulerAngles);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create hip as an Ball joint
	BallJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), useEulerAngles);

	// Rename hip coordinates for a ball joint
	CoordinateSet hip_coords = hip.getCoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords.get(i).setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel->addBody(&osim_thigh);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// create custom knee joint
	PinJoint knee("", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0));

	// Rename knee coordinates for a pin joint
	int first = 0;
	knee.getCoordinateSet().get(first).setName("knee_q");

	// Add the shank body which now also contains the knee joint to the model
	osimModel->addBody(&osim_shank);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	osimModel->setGravity(gravity_vec);

    //Add analyses before setting up the model for simulation
	Kinematics *kinAnalysis = new Kinematics(osimModel);
	kinAnalysis->setInDegrees(false);
	osimModel->addAnalysis(kinAnalysis);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel->initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	return (compareSimulations(system, state, osimModel, osim_state));

} // end testBallJoint

bool testPinJoint()
{
	cout << endl;
	cout << "=============================================================" << endl;
	cout << " OpenSim PinJoint vs. Simbody MobilizedBody::Pin " << endl;
	cout << "=============================================================" << endl;

	Random::Uniform randomAngle(-Pi/2, Pi/2);
	Vec3 oInB(randomAngle.getValue(),  randomAngle.getValue(), randomAngle.getValue());
	Vec3 oInP(randomAngle.getValue(),  randomAngle.getValue(), randomAngle.getValue());

	// Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
	SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

	// Thigh connected by hip
	MobilizedBody::Pin thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur));
	//Pin knee connects shank
	MobilizedBody::Pin shank(thigh, SimTK::Transform(Rotation(BodyRotationSequence, oInP[0], XAxis, oInP[1], YAxis, oInP[2],ZAxis), kneeInFemur),
							 SimTK::Body::Rigid(tibiaMass), SimTK::Transform(Rotation(BodyRotationSequence, oInB[0], XAxis, oInB[1], YAxis, oInB[2],ZAxis), kneeInTibia));

	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, false);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create hip as a pin joint
	PinJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));

	// Rename hip coordinates for a pin joint
	CoordinateSet hip_coords = hip.getCoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords.get(i).setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel->addBody(&osim_thigh);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// create pin knee joint
	PinJoint knee("", osim_thigh, kneeInFemur, oInP, osim_shank, kneeInTibia, oInB);
	int first = 0;
	knee.getCoordinateSet().get(first).setName("knee_q");

	// Add the shank body which now also contains the knee joint to the model
	osimModel->addBody(&osim_shank);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	osimModel->setGravity(gravity_vec);

    //Add analyses before setting up the model for simulation
	Kinematics *kinAnalysis = new Kinematics(osimModel);
	kinAnalysis->setInDegrees(false);
	osimModel->addAnalysis(kinAnalysis);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel->initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	return (compareSimulations(system, state, osimModel, osim_state));

} // end testPinJoint


bool testSliderJoint()
{
	cout << endl;
	cout << "=============================================================" << endl;
	cout << " OpenSim SliderJoint vs. Simbody MobilizedBody::Slider " << endl;
	cout << "=============================================================" << endl;

	Random::Uniform randomAngle(-Pi/2, Pi/2);
	Vec3 oInB(randomAngle.getValue(),  randomAngle.getValue(), randomAngle.getValue());
	Vec3 oInP(randomAngle.getValue(),  randomAngle.getValue(), randomAngle.getValue());

	// Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
	SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

	// Thigh connected by hip
	MobilizedBody::Pin thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur));
	//Pin knee connects shank
	MobilizedBody::Slider shank(thigh, SimTK::Transform(Rotation(BodyRotationSequence, oInP[0], XAxis, oInP[1], YAxis, oInP[2],ZAxis), kneeInFemur),
							 SimTK::Body::Rigid(tibiaMass), SimTK::Transform(Rotation(BodyRotationSequence, oInB[0], XAxis, oInB[1], YAxis, oInB[2],ZAxis), kneeInTibia));

	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, false);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model osimModel; 
	//OpenSim bodies
    OpenSim::Body& ground = osimModel.getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create hip as an Ball joint
	PinJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));

	// Rename hip coordinates for a pin joint
	CoordinateSet& hip_coords = hip.getCoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords.get(i).setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel.addBody(&osim_thigh);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// create slider knee joint
	SliderJoint knee("", osim_thigh, kneeInFemur, oInP, osim_shank, kneeInTibia, oInB);
	CoordinateSet& kneeCoords =	knee.getCoordinateSet();
	kneeCoords[0].setName("knee_qx");
	kneeCoords[0].setMotionType(Coordinate::Translational);


	// Add the shank body which now also contains the knee joint to the model
	osimModel.addBody(&osim_shank);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel.updBodySet().setMemoryOwner(false);

	osimModel.setGravity(gravity_vec);

    //Add analyses before setting up the model for simulation
	Kinematics *kinAnalysis = new Kinematics(&osimModel);
	kinAnalysis->setInDegrees(false);
	osimModel.addAnalysis(kinAnalysis);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel.initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	return (compareSimulations(system, state, &osimModel, osim_state));
} // end testSliderJoint

bool testCustomWithMultidimFunction()
{
	cout << endl;
	cout << "==========================================================" << endl;
	cout << " OpenSim CustomJoint with Multidimensional Function " << endl;
	cout << "==========================================================" << endl;
	
	MultidimensionalFunction testFunction;
	// Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
	SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);


	// Define the functions that specify the FunctionBased Mobilized Body.
	std::vector<std::vector<int> > coordIndices;
	std::vector<const SimTK::Function*> functions;
	std::vector<bool> isdof(6,false);

	// Set the 1 spatial rotation about Z-axis
	isdof[2] = false;  // rot Z
	isdof[3] = true;  // trans X
	isdof[4] = true;  // trans Y

	int nm = 2;
	for(int i=0; i<6; i++){
		if(i == 2) { //rotation coupled to translation dofs
			std::vector<int> findex(2, 0);
			findex[1] = 1;
			functions.push_back(testFunction.createSimTKFunction());
			coordIndices.push_back(findex);
		}
		else if(isdof[i]){
			Vector coeff(2);
			coeff[0] = 1;
			coeff[1] = 0;
			std::vector<int> findex(1,i-3);
			functions.push_back(new SimTK::Function::Linear(coeff));
			coordIndices.push_back(findex);	
		}
		else{
			std::vector<int> findex(0);
			functions.push_back(new SimTK::Function::Constant(0, 0));
			coordIndices.push_back(findex);
		}
	}

	// Thigh connected by hip
	MobilizedBody::FunctionBased thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur),
		nm, functions, coordIndices);
	//Function-based knee connects shank
	MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, false);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model osimModel;// = new Model;

	//OpenSim bodies
    OpenSim::Body& ground = osimModel.getGroundBody();
	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// Define hip coordinates and axes for custom joint
	SpatialTransform hipTransform;
	OpenSim::Array<std::string> coordNames;
	coordNames.append("hip_qx");	coordNames.append("hip_qy");
	hipTransform[2].setCoordinateNames(coordNames);
	hipTransform[2].setFunction(new MultidimensionalFunction());
	hipTransform[3].setCoordinateNames(OpenSim::Array<std::string>(coordNames[0], 1, 1));
	hipTransform[3].setFunction(new LinearFunction());
	hipTransform[4].setCoordinateNames(OpenSim::Array<std::string>(coordNames[1], 1, 1));
	hipTransform[4].setFunction(new LinearFunction());

	// create custom hip joint
	CustomJoint hip("", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), hipTransform);

	// Add the thigh body which now also contains the hip joint to the model
	osimModel.addBody(&osim_thigh);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee coordinates and axes for custom joint spatial transform
	SpatialTransform kneeTransform;
	string coord_name = "knee_q";
	// Only knee flexion/extension
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>(coord_name, 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());

	// create custom knee joint
	CustomJoint knee("", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	osimModel.addBody(&osim_shank);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel.updBodySet().setMemoryOwner(false);
	osimModel.setGravity(gravity_vec);

	//Add analyses before setting up the model for simulation
	Kinematics *kinAnalysis = new Kinematics(&osimModel);
	kinAnalysis->setInDegrees(false);
	osimModel.addAnalysis(kinAnalysis);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel.initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	return (compareSimulations(system, state, &osimModel, osim_state));

} //end of testCustomWithMultidimFunction


int main()
{
    int  status = 0;

	// First compare behavior of a double pendulum with Universal hip and Pin-like knee 

	if(  !testCustomVsUniversalPin()) {
        status = 1;
        cout << " testCustomVsUniversalPin FAILED " << endl;
    }
	// Compare behavior of a double pendulum with pin hip and function-based translating tibia knee 
	if( !testCustomJointVsFunctionBased()) {
        status = 1;
        cout << " testCustomJointVsFunctionBased FAILED " << endl;
    }

	// Compare behavior of a double pendulum with an Ellipsoid hip and pin knee 
	if( !testEllipsoidJoint()) {
        status = 1;
        cout << " testEllipsoidJoint FAILED " << endl;
    }

	// Compare behavior of a double pendulum (1) with welded foot and toes 
	if( !testWeldJoint(false)) {
        status = 1;
        cout << " testWeldJoint FAILED " << endl;
    }

	// Compare previous OpenSim model but with randomized body order in BodySet to test connectBodies
	if( !testWeldJoint(true)) {
        status = 1;
        cout << " testWeldJoint with random body order  FAILED " << endl;
    }

	// Compare behavior of a double pendulum with OpenSim pin hip and pin knee 
	if( !testPinJoint()) {
        status = 1;
        cout << " testPinJoint FAILED " << endl;
	}

	// Compare behavior of a two body pendulum with OpenSim pin hip and slider knee 
	if( !testSliderJoint()) {
        status = 1;
        cout << " testSliderJoint FAILED " << endl;
	}

	// Compare behavior of a double pendulum with an OpenSim Ball hip and custom pin knee 
	if( !testBallJoint(false)) {
        status = 1;
        cout << " testBallJoint using quaternions FAILED " << endl;
    }

	//  Compare behavior of a double pendulum with an OpenSim Ball hip and custom pin knee 
	if( !testBallJoint(true)) {
        status = 1;
        cout << " testBallJoint using Euler angles FAILED " << endl;
	}

	// Compare behavior of a Free hip and pin knee 
	if( !testFreeJoint(false)) {
        status = 1;
        cout << " testFreeJoint using quaternions FAILED" << endl; 
    }

	// Compare behavior of a Free hip and pin knee 
/*	if( !testFreeJoint(true)) {
        status = 1;
        cout << " testFreeJoint using Euler angles FAILED" << endl; 
    }
*/
	// Compare behavior of a Free hip and pin knee 
	if( !testCustomWithMultidimFunction()) {
        status = 1;
        cout << " testCustomWithMultidimFunction  FAILED" << endl; 
    }
	

	return status;
}
