/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testJoints.cpp                          *
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

//=============================================================================
//	testJoints builds OpenSim models using the API and builds an equivalent
//  Simbody system using the Simbody API for each test case. A test fails if 
//  the OpenSim and Simbody final states of the simulation are not equivelent 
//  (norm(error) greater than 10x integration error tolerance)
//
//	Tests Include:
//      1. CustomJoint against Simbody built-in Pin and Universal mobilizers
//      2. CustomJoint versus Simbody FunctionBased with spline functions
//		3. EllipsoidJoint against Simbody built-in Ellipsoid mobilizer
//		4. WeldJoint versus Weld Mobilizer by welding bodies 
//		5. Randomized order of bodies in the BodySet (in 3.) to test connect()
//		6. PinJoint against Simbody built-in Pin mobilizer
//		7. SliderJoint against Simbody built-in Slider mobilizer
//		8. FreeJoint against Simbody built-in Free mobilizer
//		9. BallJoint against Simbody built-in Ball mobilizer
//	   10. Equivalent Spatial body force due to applied gen force.
//		
//     Add tests here as new joint types are added to OpenSim
//
//=============================================================================

#include <OpenSim/Simulation/Manager/Manager.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/EllipsoidJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/BallJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/UniversalJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>

#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/FunctionAdapter.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

//=============================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-6;
const static double duration = 0.50;
const static SimTK::Vec3 gravity_vec = SimTK::Vec3(0, -9.8065, 0);
//Thigh
const static double femurMass = 8.806;
const static SimTK::Vec3 femurCOM(0, 0.5, 0);
const static SimTK::Inertia femurInertiaAboutCOM(SimTK::Vec3(0.1268, 0.0332, 0.1337));
//Shank
const static SimTK::MassProperties tibiaMass(3.510, SimTK::Vec3(0), 
	                 SimTK::Inertia(SimTK::Vec3(0.0477, 0.0048, 0.0484)));
//Foot
const static SimTK::MassProperties footMass(1.20, SimTK::Vec3(0), 
	                 SimTK::Inertia(SimTK::Vec3(0.001361, 0.003709, 0.003916)));
//Toes
const static SimTK::MassProperties toesMass(0.205126, SimTK::Vec3(0), 
	                 SimTK::Inertia(SimTK::Vec3(0.000117, 0.000179, 0.000119)));

// Joint locations
const static SimTK::Vec3 hipInGround(0);
const static SimTK::Vec3 hipInFemur(0.0020, 0.1715, 0);
const static SimTK::Vec3 kneeInFemur(0.0033, -0.2294, 0);
const static SimTK::Vec3 kneeInTibia(0.0, 0.1862, 0.0);
const SimTK::Vec3 ankleInTibia(0.0, -0.243800, 0);
const SimTK::Vec3 ankleInFoot(-0.035902, 0.051347, 0);
const SimTK::Vec3 mtpInFoot(0.098032, -0.038000, 0);
const SimTK::Vec3 mtpInToes(-0.035902, 0.051347, 0);
//=============================================================================

// Class for testing joints with coupled coordinates
class MultidimensionalFunction : public OpenSim::Function {
OpenSim_DECLARE_CONCRETE_OBJECT(MultidimensionalFunction, OpenSim::Function);

public:
	MultidimensionalFunction() {};
	virtual ~MultidimensionalFunction() {};

    virtual double calcValue(const SimTK::Vector& x) const
	{
		return 2*x[0]*x[0] + x[1];
	}
	virtual double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const
	{
	   int nd = (int)derivComponents.size();
	   if (nd < 1)
		   return SimTK::NaN;
	
	   if (derivComponents[0] == 0){
			if (nd == 1)
				return 4*x[0];
			else if(derivComponents[1] == 0)
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


//=============================================================================
// CompoundJoint necessary for testing equivalent body force calculations
// for joints comprised by more than one mobilized body.
//=============================================================================
class CompoundJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(CompoundJoint, Joint);

private:
	static const int _numMobilities = 3;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	CompoundJoint() :Joint() { constructCoordinates(); }
	// convenience constructor
	CompoundJoint(const std::string &name, const Body& parent,
		const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
		const Body& child,
		const SimTK::Vec3& locationInchild, const SimTK::Vec3& orientationInChild,
		bool reverse = false) :
			Joint(name, parent, locationInParent,orientationInParent,
			child, locationInchild, orientationInChild, reverse)
	{
		constructCoordinates();
	}

	virtual int numCoordinates() const {return _numMobilities;};

protected:
	void extendAddToSystem(SimTK::MultibodySystem& system) const override
	{
		using namespace SimTK;

		const CoordinateSet& coordinateSet = get_CoordinateSet();
		string msg = getConcreteClassName() +
			"::extendAddToSystem() ERROR - number of DOFs does not match number of Coordinates specified.";
		SimTK_ASSERT(coordinateSet.getSize() == _numMobilities, msg.c_str());
		// Assign the underlying indices to access System resources (state values) for the Coordinate subcomponents

		// PARENT TRANSFORM
		const SimTK::Transform& parentTransform = getParentTransform();
		// CHILD TRANSFORM
		const SimTK::Transform& childTransform = getChildTransform();

		int coordinateIndexForMobility = 0;

		SimTK::Transform childTransform0(Rotation(), Vec3(0));

		SimTK::Body::Massless massless;

		// CREATE MOBILIZED BODY for body rotation about body Z
		MobilizedBody simtkMasslessBody1 = createMobilizedBody<MobilizedBody::Pin>(
			system.updMatterSubsystem().updMobilizedBody(getParentBody().getMobilizedBodyIndex()),
			parentTransform,
			massless,
			childTransform0,
			coordinateIndexForMobility);

		// Find the joint frame with Z aligned to body X
		Rotation rotToX(Pi/2, YAxis);
		SimTK::Transform parentTransform1(rotToX, Vec3(0));
		SimTK::Transform childTransform1(rotToX, Vec3(0));

		// CREATE MOBILIZED BODY for body rotation about body X
		MobilizedBody simtkMasslessBody2 = createMobilizedBody<MobilizedBody::Pin>(
			simtkMasslessBody1,
			parentTransform1,
			massless,
			childTransform1,
			coordinateIndexForMobility);

		// Now Find the joint frame with Z aligned to body Y
		Rotation rotToY(-Pi/2, XAxis);
		SimTK::Transform parentTransform2(rotToY, Vec3(0));
		SimTK::Transform childTransform2(childTransform.R()*rotToY, childTransform.p());
		
		// CREATE MOBILIZED BODY for body rotation about body Y
		MobilizedBody mobBod = createMobilizedBody<MobilizedBody::Pin>(
			simtkMasslessBody2,
			parentTransform2,
			SimTK::Body::Rigid(getChildBody().getMassProperties()),
			childTransform2,
			coordinateIndexForMobility, &getChildBody());
	}

//=============================================================================
};	// END of class CompoundJoint
//=============================================================================

void testCustomVsUniversalPin();
void testCustomJointVsFunctionBased();
void testEllipsoidJoint();
void testWeldJoint(bool randomizeBodyOrder);
void testPinJoint();
void testSliderJoint();
void testPlanarJoint();
void testBallJoint();
void testFreeJoint();
void testCustomWithMultidimFunction();
void testCustomVsCompoundJoint();
void testEquivalentBodyForceFromGeneralizedForce();
void testEquivalentBodyForceForGenForces(Model& model);

// Multibody tree constructions tests
void testAddedFreeJointForBodyWithoutJoint();
void testAutomaticJointReversal();
void testAutomaticLoopJointBreaker();

int main()
{
    try {
		//Register new Joint types for testing 
		Object::registerType(CompoundJoint());

		// model connect should create a FreeJoint for bodies that are not
		// connected by a Joint.
		testAddedFreeJointForBodyWithoutJoint();

		// model creation should automatically reverse joints to build a tree
		// but preserve the sense of the joint as specified by the user.
		testAutomaticJointReversal();

		// test that kinematic loops are broken to form a tree with constraints
		testAutomaticLoopJointBreaker();

		// Compare behavior of a double pendulum with OpenSim pin hip and pin knee
		testPinJoint();
		// Compare behavior of a two body pendulum with OpenSim pin hip and slider knee
		testSliderJoint();
		// Compare behavior of a two body model connected via planar joints
		testPlanarJoint();
		// First compare behavior of a double pendulum with Universal hip and Pin-like knee
		testCustomVsUniversalPin();
		// Compare behavior of a double pendulum with pin hip and function-based translating tibia knee
		testCustomJointVsFunctionBased();
		// Compare behavior of a double pendulum with an Ellipsoid hip and pin knee
		testEllipsoidJoint();
		// Compare behavior of a double pendulum (1) with welded foot and toes
		testWeldJoint(false);
		// Compare previous OpenSim model but with randomized body order in BodySet to test connectBodies
		testWeldJoint(true);

		// Compare behavior of a double pendulum with an OpenSim Ball hip and custom pin knee
		// OpenSim, system restricted to using euler angles exclusively to support EllipsoidJoint
		// and the fact that coordinates cannot map to/from quaternions
		testBallJoint();
		// Compare behavior of a Free hip and pin knee 
		// OpenSim, system restricted to using euelr angles exclusively to support EllipsoidJoint
		// and the fact that coordinates cannot map to/from quaternions
		testFreeJoint();
		// Compare behavior of a Free hip and pin knee
		testCustomWithMultidimFunction();
		// Compare custom implmentation of Gimbal to a Compund (multi-mobilizer) Joint version
		testCustomVsCompoundJoint();

		testEquivalentBodyForceFromGeneralizedForce();

	}
	catch(const OpenSim::Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

//==========================================================================================================
// Common Functions 
//==========================================================================================================
int initTestStates(SimTK::Vector &qi, SimTK::Vector &ui)
{
	using namespace SimTK;

	Random::Uniform randomAngle(-Pi/4, Pi/4);
	Random::Uniform randomSpeed(-1.0, 1.0);

	// Provide initial states as random angles and speeds for OpenSim and Simbody models
	for(int i = 0; i<qi.size(); i++)
		qi[i] = randomAngle.getValue();

	for(int i = 0; i<ui.size(); i++)
		ui[i] = randomSpeed.getValue();
    
	return qi.size();
}

void integrateSimbodySystem(SimTK::MultibodySystem &system, SimTK::State &state)
{
	using namespace SimTK;

	// realize simbody system to velocity stage
	system.realize(state, Stage::Velocity);

	RungeKuttaFeldbergIntegrator integ(system);
	integ.setAccuracy(integ_accuracy);
	
    TimeStepper ts(system, integ);
    ts.initialize(state);
	ts.stepTo(duration);
	state = ts.getState();
}

void integrateOpenSimModel(Model *osimModel, SimTK::State &osim_state)
{
	using namespace SimTK;

	// SETUP OpenSim SIMULATION Manager
	osimModel->getMultibodySystem().realize(osim_state, Stage::Velocity);
    RungeKuttaFeldbergIntegrator integrator(osimModel->getMultibodySystem() );
	integrator.setAccuracy(integ_accuracy);
    Manager manager(*osimModel, integrator);

	// Specify the initial and final times of the simulation.
	// In this case, the initial and final times are set based on
	// the range of times over which the controls are available.
	//Control *control;
	manager.setInitialTime(0.0);
	manager.setFinalTime(duration);

	// Integrate
    const SimbodyMatterSubsystem& matter2 = osimModel->getMultibodySystem().getMatterSubsystem();
	//for (int i = 0; i < matter2.getNumConstraints(); i++)
	//    printf("%d: %d\n", i, matter2.isConstraintDisabled(osim_state, SimTK::ConstraintIndex(i)));
	//cout << osim_state.getQ()<<endl;
	//cout << "\n\nOpenSim Integration 0.0 to " << duration << endl;

	manager.integrate(osim_state);
}

void compareSimulationStates(const SimTK::Vector &q_sb, const SimTK::Vector &u_sb, const SimTK::Vector &q_osim, const SimTK::Vector &u_osim, string errorMessagePrefix = "")
{
	using namespace SimTK;
	
	Vector q_err = q_osim;
	Vector u_err = u_sb - u_osim;

	int nq = q_osim.size();
	if(q_sb.size() > nq){ // we have an unused quaternion slot in Simbody

		q_sb.dump("Simbody q's:");
		q_osim.dump("OpenSim q's:");
		//This is a hack knowing the free and ball joint tests have the quaternion joint first
		// And that the q's are packed as qqqq or aaa* for a ball and qqqqxyz or aaaxyz* for a free joint
		int quat_ind = ((nq > 6) ? 6 : 3);
		int j = 0;
		if(quat_ind > 5){ // this is a free joint
			// OpenSim specifies Translation mobilizer first so first and second triplet of q's
			// have to be swapped
			for(int i=0; i<3; i++){
					q_err[i] = q_osim[i] - q_sb[i+3];
					q_err[i+3] = q_osim[i+3] -q_sb[i];
					u_err[i] = u_osim[i] - u_sb[i+3];
					u_err[i+3] = u_osim[i+3] - u_sb[i];
			}
			j = quat_ind;
		}
		for(int i=j; i< q_sb.size(); i++){
			if(i != quat_ind){
				q_err[j] = q_sb[i] - q_osim[j];
				j++;
			}
		}
	}
	else{
		q_err = q_sb - q_osim;
	}
    
	
	double qerrnorm = q_err.norm();
	double uerrnorm = u_err.norm();

	cout<< "\nSimbody - OpenSim:  |q_err| = " << qerrnorm << "  |u_err| =" << uerrnorm << endl;

	stringstream errorMessage1, errorMessage2;
	errorMessage1 << "testJoints::compareSimulationStates failed q_err.norm = " << qerrnorm;
	errorMessage2 << "testJoints::compareSimulationStates failed u_err.norm = " << uerrnorm;
	ASSERT(qerrnorm <= 10 * integ_accuracy, __FILE__, __LINE__, errorMessagePrefix + errorMessage1.str());
	ASSERT(uerrnorm <= 100 * integ_accuracy, __FILE__, __LINE__, errorMessagePrefix + errorMessage2.str());
}

void compareSimulations(SimTK::MultibodySystem &system, SimTK::State &state, Model *osimModel, SimTK::State &osim_state, string errorMessagePrefix = "")
{
	using namespace SimTK;

	// Set the initial states for both Simbody system and OpenSim model
	Vector& q = state.updQ();
	Vector& u = state.updU();
	int nq_sb = initTestStates(q, u);
	int nq = osim_state.getNQ();

	// Push down to OpenSim "state"
	osim_state.updY() = state.getY();

	Vector delta = osim_state.updY() - state.getY();
	double errnorm = delta.norm();
	cout << "osim_state - sb_state: " << delta;


	//==========================================================================================================
	// Integrate Simbody system
	integrateSimbodySystem(system, state);

	// Simbody model final states
	q = state.updQ();
	u = state.updU();

	cout << "\nSimbody Final q's: " << q << endl;
	cout << "Simbody Final u's: " << u << endl;

	//==========================================================================================================
	// Integrate OpenSim model
	integrateOpenSimModel(osimModel, osim_state);

	// Get the state at the end of the integration from OpenSim.
	Vector& qos = osim_state.updQ();
	Vector& uos = osim_state.updU();
	cout << "\nOpenSim Final q's: " << qos << endl;
	cout << "OpenSim Final u's: " << uos << endl;

	//==========================================================================================================
	// Compare Simulation Results
	compareSimulationStates(q, u, qos, uos, errorMessagePrefix);
}
//==========================================================================================================

//==========================================================================================================
// Test Cases
//==========================================================================================================
void testCustomVsUniversalPin()
{
	using namespace SimTK;

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
	matter.setUseEulerAngles(state, true);
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
	CustomJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), hipTransform);

	// Add the thigh body which now also contains the hip joint to the model
	osimModel.addBody(&osim_thigh);
	osimModel.addJoint(&hip);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee coordinates and axes for custom joint spatial transform
	SpatialTransform kneeTransform;
	string knee_rot = "knee_ext";
	// Only knee flexion/extension
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>(knee_rot, 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());

	// create custom knee joint
	CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	osimModel.addBody(&osim_shank);
	osimModel.addJoint(&knee);

	// Set gravity
	osimModel.setGravity(gravity_vec);

	osimModel.disownAllComponents();

	std::cout << osimModel.getCoordinateSet().getSize() << std::endl;

	osimModel.print("testCustomVsUniversalPin.osim");

	testEquivalentBodyForceForGenForces(osimModel);
	
	Model testModel("testCustomVsUniversalPin.osim");

	SimTK::State osim_state = testModel.initSystem();
	

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	compareSimulations(system, state, &osimModel, osim_state, "testCustomVsUniversalPin FAILED\n");

} //end of testCustomVsUniversalPin

void testCustomJointVsFunctionBased()
{
	using namespace SimTK;

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

	SimmSpline tx(npx, angX, kneeX);
	SimmSpline ty(npy, angY, kneeY);;

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
	matter.setUseEulerAngles(state, true);
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
	CustomJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), hipTransform);

	// Add the thigh body which now also contains the hip joint to the model
	osimModel->addBody(&osim_thigh);
	osimModel->addJoint(&hip);

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
	CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	osimModel->addBody(&osim_shank);
	osimModel->addJoint(&knee);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->disownAllComponents();

	// write out the model to file
	osimModel->print("testCustomJoint.osim");

	testEquivalentBodyForceForGenForces(*osimModel);

	//wipe-out the model just constructed
	//delete osimModel;

	// reconstruct from the model file
	//osimModel = new Model("testCustomJoint.osim");
	
	SimTK::State osim_state = osimModel->initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	compareSimulations(system, state, osimModel, osim_state, "testCustomJointVsFunctionBased FAILED\n");
}

void testEllipsoidJoint()
{
	using namespace SimTK;

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
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))),
		SimTK::Transform(hipInFemur));
	//Pin knee connects shank
	MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

	thigh.setDefaultRadii(ellipsoidRadii);
	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create hip as an ellipsoid joint
	EllipsoidJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), ellipsoidRadii);

	// Rename hip coordinates for an ellipsoid joint
	CoordinateSet& hip_coords = hip.upd_CoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords[i].setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel->addBody(&osim_thigh);
	osimModel->addJoint(&hip);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// create pin knee joint
	PinJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0));

	// Add the shank body which now also contains the knee joint to the model
	osimModel->addBody(&osim_shank);
	osimModel->addJoint(&knee);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->disownAllComponents();

	osimModel->setGravity(gravity_vec);

	//Write model to file
	osimModel->print("testEllipsoidJoint.osim");

	cout << "EllipsoidJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(*osimModel);

	//Wipe out model
	//delete osimModel;

	//Load model from file
	//osimModel = new Model("testEllipsoidJoint.osim");

	osimModel->setUseVisualizer(false);
	SimTK::State osim_state = osimModel->initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	compareSimulations(system, state, osimModel, osim_state, "testEllipsoidJoint FAILED\n");

} // end testEllipsoidJoint


void testWeldJoint(bool randomizeBodyOrder)
{
	using namespace SimTK;

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
	matter.setUseEulerAngles(state, true);
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

	// Use a temporary BodySet to hold bodies
	JointSet tempJointSet;
	tempJointSet.setMemoryOwner(false);

	// Define hip coordinates and axes for custom joint
	SpatialTransform hipTransform;
	hipTransform[0].setCoordinateNames(OpenSim::Array<std::string>("hip_q0", 1, 1));
	hipTransform[0].setFunction(new LinearFunction());
	hipTransform[1].setCoordinateNames(OpenSim::Array<std::string>("hip_q1", 1, 1));
	hipTransform[1].setFunction(new LinearFunction());	

	// create custom hip joint
	CustomJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), hipTransform);
	
	tempBodySet.adoptAndAppend(&osim_thigh);
	tempJointSet.adoptAndAppend(&hip);

	// Add another body via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee transform for flexion/extension
	SpatialTransform kneeTransform;
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>("knee_q", 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());	

	// create custom knee joint
	CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	tempBodySet.adoptAndAppend(&osim_shank);
	tempJointSet.adoptAndAppend(&knee);

	// Add foot body at ankle
	OpenSim::Body osim_foot("foot", footMass.getMass(), footMass.getMassCenter(), footMass.getInertia());
	WeldJoint ankle("ankle", osim_shank, ankleInTibia, Vec3(0), osim_foot, ankleInFoot, Vec3(0));

	tempBodySet.adoptAndAppend(&osim_foot);
	tempJointSet.adoptAndAppend(&ankle);

	// Add toes body at mtp
	OpenSim::Body osim_toes ("toes", toesMass.getMass(), toesMass.getMassCenter(), toesMass.getInertia());
	WeldJoint mtp("mtp", osim_foot, mtpInFoot, Vec3(0), osim_toes, mtpInToes, Vec3(0));

	tempBodySet.adoptAndAppend(&osim_toes);
	tempJointSet.adoptAndAppend(&mtp);

	int b_order[] = {0, 1, 2, 3};
	int j_order[] = { 0, 1, 2, 3 };
	if(randomizeBodyOrder){
		cout << " Randomizing Bodies to exercise SimbodyEngine:: connectBodies() " << endl;
		cout << "================================================================" << endl;
		Random::Uniform randomOrder(0, 4);
		randomOrder.setSeed(clock());

		int bx =-1, jx=-1;
		bool duplicate = false;
		for (int i = 0; i < 4; ++i){
			bx = randomOrder.getIntValue();
			duplicate = false;
			for (int j = 0; j < i; j++){
				// check if we can find this index in the order list already			
				if (bx == b_order[j]){
					duplicate = true;
					break;
				}
			}
			if (duplicate) --i; // try again
			else b_order[i] = bx;
		}
		for (int i = 0; i < 4; ++i){
			jx = randomOrder.getIntValue();
			duplicate = false;
			for (int j = 0; j < i; j++){
				// check if we can find this index in the order list already			
				if (jx == j_order[j]){
					duplicate = true;
					break; // if hit a duplicate stop
				}
			}
			if (duplicate) --i; // try again
			else j_order[i] = jx;
		}

	}

	for(int i=0; i<4; i++){
		osimModel->addBody(&tempBodySet[b_order[i]]);
		osimModel->addJoint(&tempJointSet[j_order[i]]);
	}
	

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->disownAllComponents();

	// OpenSim model must realize the topology to get valid osim_state
	osimModel->setGravity(gravity_vec);

	if (randomizeBodyOrder){
		osimModel->print("testRandomizedBodyAndJointOrder.osim");
	}

	cout << "testWeldJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(*osimModel);

	SimTK::State osim_state = osimModel->initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	stringstream errorMessage;
	errorMessage << "testWeldJoint " << (randomizeBodyOrder ? "with random body order " : "") << "FAILED\n";
	compareSimulations(system, state, osimModel, osim_state, errorMessage.str());
}

void testFreeJoint()
{
	using namespace SimTK;

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
	matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create free hip joint
	FreeJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));

	// Rename hip coordinates for a free joint
	CoordinateSet& hip_coords = hip.upd_CoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords[i].setName(coord_name.str());
		hip_coords[i].setMotionType(((i<3) ? Coordinate::Rotational : Coordinate::Translational));
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel->addBody(&osim_thigh);
	osimModel->addJoint(&hip);

	// create OpenSim shank body
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee transform for flexion/extension
	SpatialTransform kneeTransform;
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>("knee_q", 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());

	// create custom knee joint
	CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	osimModel->addBody(&osim_shank);
	osimModel->addJoint(&knee);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->disownAllComponents();

	osimModel->setGravity(gravity_vec);

	cout << "testFreeJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(*osimModel);

	SimTK::State osim_state = osimModel->initSystem();

	cout << "NQ_osim = " << osim_state.getNQ() << "   NQ_simbody = " << state.getNQ() << endl;

	osimModel->print("testFreeJoint.osim");

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	stringstream errorMessage;
	errorMessage << "testFreeJoint using Euler angles FAILED\n";
	compareSimulations(system, state, osimModel, osim_state, errorMessage.str());
} // end testFreeJoint

void testBallJoint()
{
	using namespace SimTK;

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
	matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model osimModel;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel.getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create hip as an Ball joint
	BallJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));

	// Rename hip coordinates for a ball joint
	CoordinateSet hip_coords = hip.getCoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords.get(i).setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel.addBody(&osim_thigh);
	osimModel.addJoint(&hip);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// create pin knee joint
	PinJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0));

	// Rename knee coordinates for a pin joint
	int first = 0;
	knee.getCoordinateSet().get(first).setName("knee_q");

	// Add the shank body which now also contains the knee joint to the model
	osimModel.addBody(&osim_shank);
	osimModel.addJoint(&knee);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel.disownAllComponents();

	osimModel.setGravity(gravity_vec);

	cout << "testBallJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(osimModel);

	SimTK::State osim_state = osimModel.initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	stringstream errorMessage;
	errorMessage << "testBallJoint using Euler angles FAILED\n";
	compareSimulations(system, state, &osimModel, osim_state, errorMessage.str());

} // end testBallJoint

void testPinJoint()
{
	using namespace SimTK;

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
	matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create hip as a pin joint
	PinJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));

	// Rename hip coordinates for a pin joint
	CoordinateSet hip_coords = hip.getCoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords.get(i).setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel->addBody(&osim_thigh);
	osimModel->addJoint(&hip);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// create pin knee joint
	PinJoint knee("knee", osim_thigh, kneeInFemur, oInP, osim_shank, kneeInTibia, oInB);
	int first = 0;
	knee.getCoordinateSet().get(first).setName("knee_q");

	// Add the shank body which now also contains the knee joint to the model
	osimModel->addBody(&osim_shank);
	osimModel->addJoint(&knee);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->disownAllComponents();

	osimModel->setGravity(gravity_vec);

	osimModel->print("testPinJointModel.osim");
	//delete osimModel;

	Model* osimModelcopy = new Model("testPinJointModel.osim");

	ASSERT(*osimModel == *osimModelcopy);

	osimModelcopy->initSystem();

    testEquivalentBodyForceForGenForces(*osimModel);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel->initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	compareSimulations(system, state, osimModel, osim_state, "testPinJoint FAILED\n");

} // end testPinJoint

void testSliderJoint()
{
	using namespace SimTK;

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
	matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model osimModel; 
	//OpenSim bodies
    OpenSim::Body& ground = osimModel.getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create hip as an Ball joint
	PinJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));

	// Rename hip coordinates for a pin joint
	CoordinateSet& hip_coords = hip.upd_CoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords.get(i).setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel.addBody(&osim_thigh);
	osimModel.addJoint(&hip);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// create slider knee joint
	SliderJoint knee("knee", osim_thigh, kneeInFemur, oInP, osim_shank, kneeInTibia, oInB);
	CoordinateSet& kneeCoords =	knee.upd_CoordinateSet();
	kneeCoords[0].setName("knee_qx");
	kneeCoords[0].setMotionType(Coordinate::Translational);

	// Add the shank body which now also contains the knee joint to the model
	osimModel.addBody(&osim_shank);
	osimModel.addJoint(&knee);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel.disownAllComponents();

	osimModel.setGravity(gravity_vec);

	cout << "testSliderJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(osimModel);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel.initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	compareSimulations(system, state, &osimModel, osim_state, "testSliderJoint FAILED\n");
} // end testSliderJoint

void testPlanarJoint()
{
	using namespace SimTK;

	cout << endl;
	cout << "=============================================================" << endl;
	cout << " OpenSim PlanarJoint vs. Simbody MobilizedBody::Planar " << endl;
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
	MobilizedBody::Planar thigh(matter.Ground(), SimTK::Transform(hipInGround), 
		SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), SimTK::Transform(hipInFemur));
	//Pin knee connects shank
	MobilizedBody::Planar shank(thigh, SimTK::Transform(Rotation(BodyRotationSequence, oInP[0], XAxis, oInP[1], YAxis, oInP[2],ZAxis), kneeInFemur),
							 SimTK::Body::Rigid(tibiaMass), SimTK::Transform(Rotation(BodyRotationSequence, oInB[0], XAxis, oInB[1], YAxis, oInB[2],ZAxis), kneeInTibia));

	// Simbody model state setup
	system.realizeTopology();
	State state = system.getDefaultState();
	matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

	//==========================================================================================================
	// Setup OpenSim model
	Model osimModel; 
	//OpenSim bodies
    OpenSim::Body& ground = osimModel.getGroundBody();

	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	// create hip as an Ball joint
	PlanarJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));

	// Rename hip coordinates for a pin joint
	CoordinateSet& hip_coords = hip.upd_CoordinateSet();
	for(int i=0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords.get(i).setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	osimModel.addBody(&osim_thigh);
	osimModel.addJoint(&hip);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// create planar knee joint
	PlanarJoint knee("knee", osim_thigh, kneeInFemur, oInP, osim_shank, kneeInTibia, oInB);
	CoordinateSet& kneeCoords =	knee.upd_CoordinateSet();
	kneeCoords[0].setName("knee_rz");
	kneeCoords[0].setName("knee_tx");
	kneeCoords[0].setMotionType(Coordinate::Translational);
	kneeCoords[0].setName("knee_ty");
	kneeCoords[0].setMotionType(Coordinate::Translational);

	// Add the shank body which now also contains the knee joint to the model
	osimModel.addBody(&osim_shank);
	osimModel.addJoint(&knee);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel.disownAllComponents();

	osimModel.setGravity(gravity_vec);

	cout << "testSliderJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(osimModel);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State osim_state = osimModel.initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	compareSimulations(system, state, &osimModel, osim_state, "testPlanarJoint FAILED\n");
} // end testSliderJoint


void testCustomWithMultidimFunction()
{
	using namespace SimTK;

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
	matter.setUseEulerAngles(state, true);
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
	CustomJoint hip("hip", ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0), hipTransform);

	// Add the thigh body which now also contains the hip joint to the model
	osimModel.addBody(&osim_thigh);
	osimModel.addJoint(&hip);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// Define knee coordinates and axes for custom joint spatial transform
	SpatialTransform kneeTransform;
	string coord_name = "knee_q";
	// Only knee flexion/extension
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>(coord_name, 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());

	// create custom knee joint
	CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	osimModel.addBody(&osim_shank);
	osimModel.addJoint(&knee);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel.disownAllComponents();
	osimModel.setGravity(gravity_vec);

	cout << "testCustomWithMultidimFunction: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(osimModel);

	SimTK::State osim_state = osimModel.initSystem();

	//==========================================================================================================
	// Compare Simbody system and OpenSim model simulations
	compareSimulations(system, state, &osimModel, osim_state, "testCustomWithMultidimFunction FAILED\n");

} //end of testCustomWithMultidimFunction



void testCustomVsCompoundJoint()
{
	using namespace SimTK;

	cout << endl;
	cout << "==========================================================" << endl;
	cout << " OpenSim CustomJoint vs. CompoundJoint for a ZXY Gimbal " << endl;
	cout << "==========================================================" << endl;
	
	//============================================================================================
	// Setup CustomJoint model
	//============================================================================================
	Model customModel;

	//OpenSim bodies
	OpenSim::Body& ground = customModel.getGroundBody();
	//OpenSim thigh
	OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);
	osim_thigh.addDisplayGeometry("femur.vtp");

	// Define hip transform in terms of coordinates and axes for custom joint
	SpatialTransform hipTransform;
	hipTransform[0].setCoordinateNames(OpenSim::Array<std::string>("hip_qZ", 1, 1));
	hipTransform[0].setFunction(new LinearFunction());
	hipTransform[0].setAxis(Vec3(0, 0, 1));
	hipTransform[1].setCoordinateNames(OpenSim::Array<std::string>("hip_qX", 1, 1));
	hipTransform[1].setFunction(new LinearFunction());
	hipTransform[1].setAxis(Vec3(1, 0, 0));
	hipTransform[2].setCoordinateNames(OpenSim::Array<std::string>("hip_qY", 1, 1));
	hipTransform[2].setFunction(new LinearFunction());
	hipTransform[2].setAxis(Vec3(0, 1, 0));

	Random::Uniform randomAngle(-Pi/2, Pi/2);
	Vec3 oInP(randomAngle.getValue(), randomAngle.getValue(), randomAngle.getValue());
	Vec3 oInB(randomAngle.getValue(), randomAngle.getValue(), randomAngle.getValue());

	// create custom hip joint
	CustomJoint hip("hip", ground, hipInGround, oInP, osim_thigh, hipInFemur, oInB, hipTransform);

	// Add the thigh body which now also contains the hip joint to the model
	customModel.addBody(&osim_thigh);
	customModel.addJoint(&hip);

	// Add OpenSim shank via a knee joint
	OpenSim::Body osim_shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());
	osim_shank.addDisplayGeometry("tibia.vtp");

	// Define knee coordinates and axes for custom joint spatial transform
	SpatialTransform kneeTransform;
	string knee_rot = "knee_ext";
	// Only knee flexion/extension
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>(knee_rot, 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());

	// create custom knee joint
	CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	customModel.addBody(&osim_shank);
	customModel.addJoint(&knee);

	// Set gravity
	customModel.setGravity(gravity_vec);
	customModel.disownAllComponents();

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State state1 = customModel.initSystem();

	//============================================================================================
	// Setup CompoundJointed model
	//============================================================================================
	Model compoundModel;

	//OpenSim bodies
    OpenSim::Body& ground2 = compoundModel.getGroundBody();

	//OpenSim thigh
	OpenSim::Body thigh2("thigh2", femurMass, femurCOM, femurInertiaAboutCOM);
	thigh2.addDisplayGeometry("femur.vtp");

	// create compound hip joint
	CompoundJoint hip2("hip2", ground2, hipInGround, oInP, thigh2, hipInFemur, oInB);

	// Add the thigh body which now also contains the hip joint to the model
	compoundModel.addBody(&thigh2);
	compoundModel.addJoint(&hip2);

	// Add OpenSim shank via a knee joint
	OpenSim::Body shank2("shank2", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());
	shank2.addDisplayGeometry("tibia.vtp");

	// create custom knee joint
	CustomJoint knee2("knee2", thigh2, kneeInFemur, Vec3(0), shank2, kneeInTibia, Vec3(0), kneeTransform);

	// Add the shank body which now also contains the knee joint to the model
	compoundModel.addBody(&shank2);
	compoundModel.addJoint(&knee2);

	// Set gravity
	compoundModel.setGravity(gravity_vec);

	compoundModel.disownAllComponents();

	//testEquivalentBodyForceForGenForces(compoundModel);

	// Need to setup model before adding an analysis since it creates the AnalysisSet
	// for the model if it does not exist.
	SimTK::State state2 = compoundModel.initSystem();
	SimTK::Vec3 com2 = compoundModel.calcMassCenterPosition(state2);

	// Test how default states are updated and serialized as part of properties
	state1.updQ()[0] = 0.1;
	state1.updQ()[1] = 0.2;
	state1.updQ()[2] = 0.3;
	state1.updQ()[3] = -Pi / 2;

	state2.updQ() = state1.getQ();

    customModel.setPropertiesFromState(state1);
    compoundModel.setPropertiesFromState(state2);

	customModel.print("Gimbal_CustomZXY_test.osim");
	compoundModel.print("Gimbal_CompoundPinsZXY_test.osim");	

	SimTK::Vec3 com1 = customModel.calcMassCenterPosition(state1);
	com2 = compoundModel.calcMassCenterPosition(state2);

	//==========================================================================================================
	// Compare compound and custom joint model simulations
	compareSimulations(compoundModel.updMultibodySystem(), state2, &customModel, state1, "testCustomVsCoupleJoint FAILED\n");

} //end of testCustomVsCompoundJoint


void testEquivalentBodyForceFromGeneralizedForce()
{
	using namespace SimTK;

	cout << endl;
	cout << "================================================================================" << endl;
	cout << " OpenSim calcualation of equivalent spatial body force from applied gen. force. " << endl;
	cout << " Applied to all the joints of a gait model with coupler contraints.  " << endl; 
	cout << "================================================================================" << endl;

	Model gaitModel("testJointConstraints.osim");
	testEquivalentBodyForceForGenForces(gaitModel);
	
} // end testEquivalentBodyForceFromGeneralizedForce


void testEquivalentBodyForceForGenForces(Model &model)
{
	using namespace SimTK;

	State &state = model.initSystem();
	Vector& qi = state.updQ();
	Vector& ui = state.updU();
	// Randomly select the initial state of this model
	int nq = initTestStates(qi, ui);

	const SimbodyMatterSubsystem& matter = model.getMatterSubsystem();

	// The number of mobilities for the entire system.
	int nm = matter.getNumMobilities();
	
	Vector genForces(nm, 0.0);
	Random::Uniform genForceRandom(-1000, 1000);
	for(int i=0; i<nm; ++i){
		genForces[i] = genForceRandom.getValue();
	}

	int nb = matter.getNumBodies();
	Vector_<SpatialVec> bodyForces(nb, SpatialVec(Vec3(0), Vec3(0)));

	Vector udot1(nm);
	Vector_<SpatialVec> bodyAccs(nb);

	model.getMultibodySystem().realize(state, SimTK::Stage::Acceleration);
	matter.calcAcceleration(state, genForces, bodyForces, udot1, bodyAccs);

	// Construct the system vector of body forces from a Joint's  equivalence to generalized force calculations
	for(int j=0; j < model.getJointSet().getSize(); ++j){
		Joint &joint = model.getJointSet()[j];
		const OpenSim::Body &body = joint.getChildBody();
		MobilizedBodyIndex mbx = body.getMobilizedBodyIndex();

		const OpenSim::Body &parent = joint.getParentBody();
		MobilizedBodyIndex mpx = parent.getMobilizedBodyIndex();

		Vec3 rB_Bo(0), rB_Po(0);
		rB_Bo = joint.getLocationInChild();

		//Get Joint frame B location in parent, Po, to apply to parent Body
		model.getSimbodyEngine().transformPosition(state, body, rB_Bo, parent, rB_Po);

		// get the equivalent spatial force on the joint frame of the (child) body expressed in ground
		SpatialVec FB_G = joint.calcEquivalentSpatialForce(state, genForces);

		cout << joint.getName() << " equivalent FB_G = " << FB_G << endl;

		// Apply spatial forces at joint to the body
		matter.addInStationForce(state, mbx, rB_Bo, FB_G[1], bodyForces);
		matter.addInBodyTorque(state, mbx, FB_G[0], bodyForces);

		// Apply equal and opposite spatial forces at joint to the parent body
		matter.addInStationForce(state, mpx, rB_Po, -FB_G[1], bodyForces);
		matter.addInBodyTorque(state, mpx, -FB_G[0], bodyForces);
	}

	Vector udot2(nm);
	matter.calcAcceleration(state, 0.0*genForces, bodyForces, udot2, bodyAccs);
	
	// If calcEquivalentSpatialForce is correct then the two methods of applying forces to the
	// model should be equivalent and the accelerations should be identical 
	Vector error = udot2-udot1;
	double norm_rel_error = error.norm()/udot1.norm();

	cout << "****************************************************************************" << endl;
	cout << "uDot Error = " << norm_rel_error << ": from body forces vs. mobility forces." << endl;
	cout << "****************************************************************************" << endl;

	ASSERT(!SimTK::isNaN(norm_rel_error), __FILE__, __LINE__, "testEquivalentBodyForceForGenForces FAILED, udot_error = NaN");
	ASSERT(norm_rel_error <= SimTK::SignificantReal,  __FILE__, __LINE__, "testEquivalentBodyForceForGenForces FAILED, udot_error > SimTK::SignificantReal"); 
}


void testAddedFreeJointForBodyWithoutJoint()
{
	using namespace OpenSim;

	Model model;
	SimTK::Inertia inertia(SimTK::Inertia::brick(SimTK::Vec3(0.5, 0.15, 0.2)));
	Body* block = new Body("block", 1.0, SimTK::Vec3(0.0), inertia);
	model.addBody(block);

	model.initSystem();

	ASSERT(model.getNumCoordinates() == 6);
	model.printBasicInfo(cout);
}

void testAutomaticJointReversal()
{
	using namespace OpenSim;

	//==========================================================================================================
	// Setup new OpenSim model
	Model model;

	//OpenSim bodies
	Body& ground = model.getGroundBody();
	Body pelvis("pelvis", 10.0, SimTK::Vec3(0), SimTK::Inertia::brick(SimTK::Vec3(0.1, 0.15, 0.25)));
	Body thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);
	Body shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());
	Body foot("foot", footMass.getMass(), footMass.getMassCenter(), footMass.getInertia());

	// add them to the model
	model.addBody(&foot);
	model.addBody(&shank);
	model.addBody(&thigh);
	model.addBody(&pelvis);

	//define some joints
	// Define hip coordinates and axes for custom joint
	SpatialTransform hipTransform;
	hipTransform[0].setCoordinateNames(OpenSim::Array<std::string>("hip_q0", 1, 1));
	hipTransform[0].setFunction(new LinearFunction());
	hipTransform[1].setCoordinateNames(OpenSim::Array<std::string>("hip_q1", 1, 1));
	hipTransform[1].setFunction(new LinearFunction());

	SimTK::Vec3 zvec(0);
	// create custom hip joint
	CustomJoint hip("hip", pelvis, hipInGround, zvec, thigh, hipInFemur, zvec, hipTransform);
	
	// Define knee transform for flexion/extension
	SpatialTransform kneeTransform;
	kneeTransform[2].setCoordinateNames(OpenSim::Array<std::string>("knee_q", 1, 1));
	kneeTransform[2].setFunction(new LinearFunction());

	// create custom knee joint
	CustomJoint knee("knee", thigh, kneeInFemur, zvec, 
		                     shank, kneeInTibia, zvec, kneeTransform);

	PinJoint ankle("ankle", shank, ankleInTibia, zvec, 
		                    foot, ankleInFoot, zvec);

	WeldJoint footToFloor("footToFloor", foot, zvec, zvec, ground, zvec, zvec);

	//add joints to the model
	model.addJoint(&hip);
	model.addJoint(&knee);
	model.addJoint(&ankle);
	model.addJoint(&footToFloor);
	model.disownAllComponents();

	SimTK::State& s = model.initSystem();

	ASSERT(knee.get_reverse() == true);

	// Test against an equivalen model with the foot constrained to ground
	// instead of having a WeldJoint to ground
	Model modelConstrained(model);
	int rix = modelConstrained.updJointSet().getIndex("footToFloor");
	modelConstrained.updJointSet().remove(rix);
	modelConstrained.finalizeFromProperties();

	Body* cpelvis = dynamic_cast<Body*>(&modelConstrained.updComponent("pelvis"));
	Body* cfoot = dynamic_cast<Body*>(&modelConstrained.updComponent("foot"));
	Body& cground = modelConstrained.getGroundBody();

	// free the pelvis
	FreeJoint pelvisFree("pelvisFree", cground, zvec, zvec,
		                        *cpelvis, zvec, zvec);

	for (int i = 0; i < pelvisFree.getCoordinateSet().getSize(); ++i) {
		// don't tie assembly to coordinate default values
		pelvisFree.upd_CoordinateSet()[i].set_is_free_to_satisfy_constraints(true);
	}
	modelConstrained.addJoint(&pelvisFree);
	
	WeldConstraint footConstraint("footConstraint",
		                          *cfoot, zvec, zvec, cground, zvec, zvec);
	modelConstrained.addConstraint(&footConstraint);
	modelConstrained.disownAllComponents();

	SimTK::State& sc = modelConstrained.initSystem();

	//The two systems should yield equivalent model dynamics
	integrateOpenSimModel(&model, s);
	integrateOpenSimModel(&modelConstrained, sc);

	double qknee = model.getCoordinateSet().get("knee_q").getValue(s);
	double qkneec = modelConstrained.getCoordinateSet().get("knee_q").getValue(sc);

	SimTK::Vec3 com = model.calcMassCenterPosition(s);
	SimTK::Vec3 comc = modelConstrained.calcMassCenterPosition(sc);

	ASSERT_EQUAL(qknee, qkneec, 10*integ_accuracy);

	double distErr = (com - comc).norm();
	ASSERT_EQUAL(distErr, 0.0, 10*integ_accuracy);

	SimTK::Vec3 acom = model.calcMassCenterAcceleration(s);
	SimTK::Vec3 acomc = modelConstrained.calcMassCenterAcceleration(sc);

	double accErr = ((acom - acomc).norm())/acom.norm();
	ASSERT_EQUAL(accErr, 0.0, 10*integ_accuracy);
}

void testAutomaticLoopJointBreaker()
{
	// Setup OpenSim model
	Model model;
	//OpenSim bodies
	Body& ground = model.getGroundBody();

	//OpenSim thigh
	Body thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

	SimTK::Vec3 zvec(0);

	// create hip as an Ball joint
	BallJoint hip("hip", ground, hipInGround, zvec, thigh, hipInFemur, zvec);

	// Rename hip coordinates for a ball joint
	CoordinateSet hip_coords = hip.getCoordinateSet();
	for (int i = 0; i<hip_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "hip_q" << i;
		hip_coords.get(i).setName(coord_name.str());
	}

	// Add the thigh body which now also contains the hip joint to the model
	model.addBody(&thigh);
	model.addJoint(&hip);

	// Add OpenSim shank via a knee joint
	Body shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(), tibiaMass.getInertia());

	// create pin knee joint
	PinJoint knee("knee", thigh, kneeInFemur, zvec, shank, kneeInTibia, zvec);

	// Rename knee coordinates for a pin joint
	int first = 0;
	knee.getCoordinateSet().get(first).setName("knee_q");

	// Add the shank body and the knee joint to the model
	model.addBody(&shank);
	model.addJoint(&knee);

	// Add foot body at ankle
	Body foot("foot", footMass.getMass(), footMass.getMassCenter(), footMass.getInertia());
	UniversalJoint ankle("ankle", shank, ankleInTibia, zvec, foot, ankleInFoot, zvec);

	// Join the foot to the floor via Weld
	PinJoint footToFloor("footToFloor", foot, zvec, zvec, ground, zvec, zvec);

	// Add foot and joints
	model.addBody(&foot);
	model.addJoint(&ankle);
	// This forms the closed loop kinematic chain
	model.addJoint(&footToFloor);

	model.disownAllComponents();
	model.setGravity(gravity_vec);

	SimTK::State &s = model.initSystem();

	int ncoords = model.getNumCoordinates();
	int nconstraints = model.getNumConstraints();

	int nu = model.getMatterSubsystem().getNumMobilities();
	
	//User should get the dofs they requested
	ASSERT(ncoords == nu, __FILE__, __LINE__,
		"Multibody tree failed to preserve user-specified coordinates.");

	SimTK::Vec3 acc = model.calcMassCenterAcceleration(s);
	// number of active constraints
	int nc = model.getMatterSubsystem().getConstraintMultipliers(s).size();

	ASSERT(nc == 6, __FILE__, __LINE__,
		"Loop closure failed to adequately constrain tree.");

	std::string file("testModelWithLoopJoint.osim");
	model.print(file);

	
	Model loadedModel(file);

	SimTK::State &s2 = loadedModel.initSystem();

	ASSERT(model == loadedModel);

	SimTK::Vec3 acc2 = model.calcMassCenterAcceleration(s2);

	ASSERT_EQUAL(acc2, acc, SimTK::Vec3(SimTK::Eps));
	
}