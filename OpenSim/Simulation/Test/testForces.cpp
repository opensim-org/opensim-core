// testForces.cpp
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
//
//	Tests Include:
//      1. PointToPointSpring
//		2. BushingForce
//		3. ElasticFoundationForce
//		4. HuntCrossleyForce
//		5. CoordinateLimitForce
//		6. CoupledBushingForce
//		
//     Add tests here as Forces are added to OpenSim
//
//==========================================================================================================
#include <OpenSim/OpenSim.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC

#include <OpenSim/Simulation/Model/CoupledBushingForce.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}
#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

//==========================================================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-4;
const static double duration = 1.0;
const static Vec3 gravity_vec = Vec3(0, -9.8065, 0);

//==========================================================================================================
static int counter=0;
//==========================================================================================================
// Test Cases
//==========================================================================================================

int testSpringMass()
{

	double mass = 1;
	double stiffness = 10;
	double restlength = 1.0;
	double h0 = 0;
	double start_h = 0.5;
	double ball_radius = 0.25;

	double omega = sqrt(stiffness/mass);

	double dh = mass*gravity_vec(1)/stiffness;

	// Setup OpenSim model
	Model *osimModel = new Model;
	osimModel->setName("SpringMass");
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();
	OpenSim::Body ball("ball", mass ,Vec3(0),  mass*SimTK::Inertia::sphere(0.1));
	ball.addDisplayGeometry("sphere.vtp");
	ball.scale(Vec3(ball_radius), false);

	// Add joints
	SliderJoint slider("", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));

	double positionRange[2] = {-10, 10};
	// Rename coordinates for a slider joint
	CoordinateSet &slider_coords = slider.getCoordinateSet();
	slider_coords[0].setName("ball_h");
	slider_coords[0].setRange(positionRange);
	slider_coords[0].setMotionType(Coordinate::Translational);

	osimModel->addBody(&ball);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	osimModel->setGravity(gravity_vec);

	PointToPointSpring spring("ground", 
		Vec3(0.,restlength,0.), 
		"ball", 
		Vec3(0.), 
		stiffness, 
		restlength);

	osimModel->addForce(&spring);

	//osimModel->print("SpringMassModel.osim");

	// Create the force reporter
	ForceReporter* reporter = new ForceReporter(osimModel);
	osimModel->addAnalysis(reporter);

	SimTK::State osim_state = osimModel->initSystem();

	slider_coords[0].setValue(osim_state, start_h);
    osimModel->getMultibodySystem().realize(osim_state, Stage::Position );

	//==========================================================================================================
	// Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem() );
	integrator.setAccuracy(1e-6);
    Manager manager(*osimModel,  integrator);
    manager.setInitialTime(0.0);

	double final_t = 2.0;
	double nsteps = 10;
	double dt = final_t/nsteps;

	for(int i = 1; i <=nsteps; i++){
		manager.setFinalTime(dt*i);
		manager.integrate(osim_state);
		osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos;
        osimModel->updSimbodyEngine().getPosition(osim_state, ball, Vec3(0), pos);
		
		double height = (start_h-dh)*cos(omega*osim_state.getTime())+dh;
		ASSERT_EQUAL(height, pos(1), 1e-5);

		//Now check that the force reported by spring
		Array<double> model_force = spring.getRecordValues(osim_state);

		// get the forces applied to the ground and ball
		double analytical_force = -stiffness*height;
		// analytical force corresponds in direction to the force on the ball Y index = 7
		ASSERT_EQUAL(analytical_force, model_force[7], 1e-4);

		manager.setInitialTime(dt*i);
	}

	// Save the forces
	//reporter->getForceStorage().print("spring_mass_forces.mot");  

	// Before exiting lets see if copying the spring works
	PointToPointSpring *copyOfSpring = (PointToPointSpring *)spring.copy();

	bool isEqual = (*copyOfSpring == spring);
	ASSERT(isEqual);

	return 0;
}

int testBushingForce()
{
	double mass = 1;
	double stiffness = 10;
	double restlength = 0.0;
	double h0 = 0;
	double start_h = 0.5;
	double ball_radius = 0.25;

	double omega = sqrt(stiffness/mass);

	double dh = mass*gravity_vec(1)/stiffness;

	// Setup OpenSim model
	Model *osimModel = new Model;
	osimModel->setName("BushingTest");
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();
	OpenSim::Body ball("ball", mass ,Vec3(0),  mass*SimTK::Inertia::sphere(0.1));
	ball.addDisplayGeometry("sphere.vtp");
	ball.scale(Vec3(ball_radius), false);

	// Add joints
	SliderJoint slider("", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));

	double positionRange[2] = {-10, 10};
	// Rename coordinates for a slider joint
	CoordinateSet &slider_coords = slider.getCoordinateSet();
	slider_coords[0].setName("ball_h");
	slider_coords[0].setRange(positionRange);
	slider_coords[0].setMotionType(Coordinate::Translational);

	osimModel->addBody(&ball);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	Vec3 rotStiffness(0);
	Vec3 transStiffness(stiffness);
	Vec3 rotDamping(0);
	Vec3 transDamping(0);

	osimModel->setGravity(gravity_vec);

	BushingForce spring("ground", Vec3(0), Vec3(0), "ball", Vec3(0), Vec3(0), transStiffness, rotStiffness, transDamping, rotDamping);

	osimModel->addForce(&spring);

	osimModel->print("BushingForceModel.osim");

	// Create the force reporter
	ForceReporter* reporter = new ForceReporter(osimModel);
	osimModel->addAnalysis(reporter);

	SimTK::State &osim_state = osimModel->initSystem();

	slider_coords[0].setValue(osim_state, start_h);
    osimModel->getMultibodySystem().realize(osim_state, Stage::Position );

	//==========================================================================================================
	// Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem() );
	integrator.setAccuracy(1e-6);
    Manager manager(*osimModel,  integrator);
    manager.setInitialTime(0.0);

	double final_t = 2.0;
	double nsteps = 10;
	double dt = final_t/nsteps;

	for(int i = 1; i <=nsteps; i++){
		manager.setFinalTime(dt*i);
		manager.integrate(osim_state);
		osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos;
        osimModel->updSimbodyEngine().getPosition(osim_state, ball, Vec3(0), pos);
		
		double height = (start_h-dh)*cos(omega*osim_state.getTime())+dh;
		ASSERT_EQUAL(height, pos(1), 1e-4);

		//Now check that the force reported by spring
		Array<double> model_force = spring.getRecordValues(osim_state);

		// get the forces applied to the ground and ball
		double analytical_force = -stiffness*height;
		// analytical force corresponds in direction to the force on the ball Y index = 7
		ASSERT_EQUAL(analytical_force, model_force[7], 1e-4);

		manager.setInitialTime(dt*i);
	}

	manager.getStateStorage().print("bushing_model_states.sto");

	// Save the forces
	reporter->getForceStorage().print("bushing_forces.mot");  

	// Before exiting lets see if copying the spring works
	BushingForce *copyOfSpring = (BushingForce *)spring.copy();

	bool isEqual = (*copyOfSpring == spring);
	ASSERT(isEqual);

	return 0;
}


// Test our wraapping of elastic foundation in OpenSim
// Simple simulation of bouncing ball with dissipation should generate contact
// forces that settle to ball weight.
int testElasticFoundation()
{
	double start_h = 0.5;

	// Setup OpenSim model
	Model *osimModel = new Model("BouncingBallModelEF.osim");
	
	// Create the force reporter
	ForceReporter* reporter = new ForceReporter(osimModel);
	osimModel->addAnalysis(reporter);

	SimTK::State osim_state = osimModel->initSystem();

	osimModel->getCoordinateSet().get("ball_ty").setValue(osim_state, start_h);
    osimModel->getMultibodySystem().realize(osim_state, Stage::Position );

	const OpenSim::Body &ball = osimModel->getBodySet().get("ball"); 

	//==========================================================================================================
	// Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem() );
	integrator.setAccuracy(1e-6);
    Manager manager(*osimModel,  integrator);
    manager.setInitialTime(0.0);

	double final_t = 2.0;

	manager.setFinalTime(final_t);

	// start timing
	clock_t startTime = clock();

	manager.integrate(osim_state);

	// end timing
	cout << "Elastic Foundation simulation time = " << 1.e3*(clock()-startTime)/CLOCKS_PER_SEC << "ms" << endl;;

	//make sure we can access dynamic variables
	osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);

	// Print out the motion for visualizing/debugging
	manager.getStateStorage().print("bouncing_ball_states.sto");
		
	// Save the forces
	reporter->getForceStorage().print("elastic_contact_forces.mot"); 

	// Bouncing ball should have settled to rest on groun due to dissipation
	// In that case the force generated by contact should be identically body weight
	// in vertical and zero else where.
	OpenSim::ElasticFoundationForce &contact = (OpenSim::ElasticFoundationForce &)osimModel->getForceSet().get("contact");

	Array<double> contact_force = contact.getRecordValues(osim_state);
	ASSERT_EQUAL(contact_force[0], 0.0, 1e-4); // no horizontal force on the ball
	ASSERT_EQUAL(contact_force[1], -ball.getMass()*gravity_vec[1], 1e-3); // vertical is weight
	ASSERT_EQUAL(contact_force[2], 0.0, 1e-4); // no horizontal force on the ball
	ASSERT_EQUAL(contact_force[3], 0.0, 1e-4); // no torque on the ball
	ASSERT_EQUAL(contact_force[4], 0.0, 1e-4); // no torque on the ball
	ASSERT_EQUAL(contact_force[5], 0.0, 1e-4); // no torque on the ball

	// Before exiting lets see if copying the spring works
	OpenSim::ElasticFoundationForce *copyOfForce = (OpenSim::ElasticFoundationForce *)contact.copy();

	bool isEqual = (*copyOfForce == contact);
	
	if(!isEqual){
		contact.print("originalForce.xml");
		copyOfForce->print("copyOfForce.xml");
	}

	ASSERT(isEqual);

	return 0;
}

// Test our wraapping of Hunt-Crossley force in OpenSim
// Simple simulation of bouncing ball with dissipation should generate contact
// forces that settle to ball weight.
int testHuntCrossleyForce()
{
	double start_h = 0.5;

	// Setup OpenSim model
	Model *osimModel = new Model("BouncingBall_HuntCrossley.osim");
	
	// Create the force reporter
	ForceReporter* reporter = new ForceReporter(osimModel);
	osimModel->addAnalysis(reporter);

	SimTK::State osim_state = osimModel->initSystem();

	osimModel->getCoordinateSet()[4].setValue(osim_state, start_h);
    osimModel->getMultibodySystem().realize(osim_state, Stage::Position );

	const OpenSim::Body &ball = osimModel->getBodySet().get("ball"); 

	//==========================================================================================================
	// Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem() );
	integrator.setAccuracy(1e-6);
    Manager manager(*osimModel,  integrator);
    manager.setInitialTime(0.0);

	double final_t = 2.0;

	manager.setFinalTime(final_t);
	
	// start timing
	clock_t startTime = clock();

	manager.integrate(osim_state);

	// end timing
	cout << "Hunt Crossley simulation time = " << 1.e3*(clock()-startTime)/CLOCKS_PER_SEC << "ms" << endl;
	
	//make sure we can access dynamic variables
	osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);

	// Print out the motion for visualizing/debugging
	manager.getStateStorage().print("bouncing_ball_HC_states.sto");
		
	// Save the forces
	reporter->getForceStorage().print("HuntCrossley_contact_forces.mot"); 

	// Bouncing ball should have settled to rest on groun due to dissipation
	// In that case the force generated by contact should be identically body weight
	// in vertical and zero else where.
	OpenSim::HuntCrossleyForce &contact = (OpenSim::HuntCrossleyForce &)osimModel->getForceSet().get("contact");

	Array<double> contact_force = contact.getRecordValues(osim_state);
	ASSERT_EQUAL(contact_force[0], 0.0, 1e-4); // no horizontal force on the ball
	ASSERT_EQUAL(contact_force[1], -ball.getMass()*gravity_vec[1], 1e-3); // vertical is weight
	ASSERT_EQUAL(contact_force[2], 0.0, 1e-4); // no horizontal force on the ball
	ASSERT_EQUAL(contact_force[3], 0.0, 1e-4); // no torque on the ball
	ASSERT_EQUAL(contact_force[4], 0.0, 1e-4); // no torque on the ball
	ASSERT_EQUAL(contact_force[5], 0.0, 1e-4); // no torque on the ball

	// Before exiting lets see if copying the force works
	OpenSim::HuntCrossleyForce *copyOfForce = (OpenSim::HuntCrossleyForce *)contact.copy();

	bool isEqual = (*copyOfForce == contact);
	
	if(!isEqual){
		contact.print("originalForce.xml");
		copyOfForce->print("copyOfForce.xml");
	}

	ASSERT(isEqual);

	return 0;
}


int testCoordinateLimitForce()
{
	double mass = 1;
	double ball_radius = 0.25;

	// Setup OpenSim model
	Model *osimModel = new Model;
	osimModel->setName("CoordinateLimitForceTest");
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();
	OpenSim::Body ball("ball", mass ,Vec3(0),  mass*SimTK::Inertia::sphere(0.1));
	ball.addDisplayGeometry("sphere.vtp");
	ball.scale(Vec3(ball_radius), false);

	// Add joints
	SliderJoint slider("", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));

	double positionRange[2] = {0.1, 2};
	// Rename coordinates for a slider joint
	CoordinateSet &slider_coords = slider.getCoordinateSet();
	slider_coords[0].setName("ball_h");
	slider_coords[0].setRange(positionRange);
	slider_coords[0].setMotionType(Coordinate::Translational);

	osimModel->addBody(&ball);

	osimModel->setGravity(gravity_vec);

	// Define the parameters of the Coordinate Limit Force
	double K_upper = 10.0;
	double K_lower = 100.0;
	double damping = 0.01;
	double trans = 0.001;
	CoordinateLimitForce limitForce("ball_h", positionRange[1],  K_upper, 
			 positionRange[0], K_lower,  damping, trans);

	osimModel->addForce(&limitForce);

		// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->disownAllComponents();

	osimModel->print("CoordinateLimitForceTest.osim");

	// Check serialization and deserilaization
	delete osimModel;
	osimModel = new Model("CoordinateLimitForceTest.osim");

	// check copy
	Model *copyModel = (Model *)osimModel->copy();
	delete osimModel;

	osimModel = copyModel;

	// Create the force reporter
	ForceReporter* reporter = new ForceReporter(osimModel);
	osimModel->addAnalysis(reporter);

	SimTK::State &osim_state = osimModel->initSystem();

	double dh = 0.1;
	double start_h = positionRange[1] + dh;
	const Coordinate &q_h = osimModel->getCoordinateSet()[0];
	q_h.setValue(osim_state, start_h);

	// initial energy of the system;
	double energy = 0.5*K_upper*dh*dh + mass*(-gravity_vec[1])*start_h;

    osimModel->getMultibodySystem().realize(osim_state, Stage::Position );

	//==========================================================================================================
	// Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem() );
	integrator.setAccuracy(1e-6);
    Manager manager(*osimModel,  integrator);
    manager.setInitialTime(0.0);

	double final_t = 2.0;
	double nsteps = 10;
	double dt = final_t/nsteps;

	for(int i = 0; i <=nsteps; i++){
		manager.setFinalTime(dt*i);
		manager.integrate(osim_state);
		osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);
		
		double h = q_h.getValue(osim_state);
		double v = q_h.getSpeedValue(osim_state);

		//Now check that the force reported by spring
		Array<double> model_force = osimModel->getForceSet()[0].getRecordValues(osim_state);

		// get the forces applied to the ball by the limit force
		double analytical_force = 0;
		if(h > (positionRange[1]+trans)){
			ASSERT_EQUAL(-K_upper*(h-positionRange[1])-damping*v, model_force[0], 1e-4);
		}
		else if( h < (positionRange[0]-trans)){
			ASSERT_EQUAL(K_lower*(positionRange[0]-h)-damping*v, model_force[0], 1e-4);
		}
		else{
			// Verify no force is being applied by limiting force when not exceeding limits 
			ASSERT_EQUAL(0, model_force[0], 1e-5);
			//also that the kinetic & potential energy of the system is going down due to damping at limits;
			double e = 0.5*mass*v*v - mass*gravity_vec[1]*h;
			
			if (damping == 0.0){
				ASSERT_EQUAL(e, energy, 1e-4);
			}else{
				ASSERT(e < energy);
			}
		}

		manager.setInitialTime(dt*i);
	}

	manager.getStateStorage().print("coordinte_limit_force_model_states.sto");

	// Save the forces
	reporter->getForceStorage().print("limit_forces.mot");  

	return 0;
}

int testCoupledBushingForce()
{
	double mass = 1;
	double stiffness = 10;
	double restlength = 0.0;
	double h0 = 0;
	double start_h = 0.5;
	double ball_radius = 0.25;

	double omega = sqrt(stiffness/mass);

	double dh = mass*gravity_vec(1)/stiffness;

	// Setup OpenSim model
	Model *osimModel = new Model;
	osimModel->setName("CoupledBushingTest");
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();
	OpenSim::Body ball("ball", mass ,Vec3(0),  mass*SimTK::Inertia::sphere(0.1));
	ball.addDisplayGeometry("sphere.vtp");
	ball.scale(Vec3(ball_radius), false);

	// Add joints
	SliderJoint slider("", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));

	double positionRange[2] = {-10, 10};
	// Rename coordinates for a slider joint
	CoordinateSet &slider_coords = slider.getCoordinateSet();
	slider_coords[0].setName("ball_h");
	slider_coords[0].setRange(positionRange);
	slider_coords[0].setMotionType(Coordinate::Translational);

	osimModel->addBody(&ball);

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	Mat66 stiffnessMat(0);
	stiffnessMat(3,3) = stiffness; stiffnessMat(4,4) = stiffness; stiffnessMat(5,5) = stiffness;
	Mat66 dampingMat(0);

	osimModel->setGravity(gravity_vec);

	CoupledBushingForce spring("ground", Vec3(0), Vec3(0), "ball", Vec3(0), Vec3(0), stiffnessMat, dampingMat);

	osimModel->addForce(&spring);

	osimModel->print("CoupledBushingForceModel.osim");

	// Create the force reporter
	ForceReporter* reporter = new ForceReporter(osimModel);
	osimModel->addAnalysis(reporter);

	SimTK::State &osim_state = osimModel->initSystem();

	slider_coords[0].setValue(osim_state, start_h);
    osimModel->getMultibodySystem().realize(osim_state, Stage::Position );

	//==========================================================================================================
	// Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem() );
	integrator.setAccuracy(1e-6);
    Manager manager(*osimModel,  integrator);
    manager.setInitialTime(0.0);

	double final_t = 2.0;
	double nsteps = 10;
	double dt = final_t/nsteps;

	for(int i = 1; i <=nsteps; i++){
		manager.setFinalTime(dt*i);
		manager.integrate(osim_state);
		osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos;
        osimModel->updSimbodyEngine().getPosition(osim_state, ball, Vec3(0), pos);
		
		double height = (start_h-dh)*cos(omega*osim_state.getTime())+dh;
		ASSERT_EQUAL(height, pos(1), 1e-4);

		//Now check that the force reported by spring
		Array<double> model_force = spring.getRecordValues(osim_state);

		// get the forces applied to the ground and ball
		double analytical_force = -stiffness*height;
		// analytical force corresponds in direction to the force on the ball Y index = 7
		ASSERT_EQUAL(analytical_force, model_force[7], 1e-4);

		manager.setInitialTime(dt*i);
	}

	manager.getStateStorage().print("coupled_bushing_model_states.sto");

	// Save the forces
	reporter->getForceStorage().print("coupled_bushing_forces.mot");  

	// Before exiting lets see if copying the spring works
	CoupledBushingForce *copyOfSpring = (CoupledBushingForce *)spring.copy();

	bool isEqual = (*copyOfSpring == spring);
	//ASSERT(isEqual);

	return 0;
}

int testCoupledBushingForceEnergy()
{
	//define a new model properties
	double mass = 10.0;
	double angRange[2] = {-Pi, Pi};
	double posRange[2] = {-1, 1};
	
	// construct a new OpenSim model
	Model model;
	model.setName("CoupledBushingEnergyTest");
	//OpenSim bodies
    OpenSim::Body& ground = model.getGroundBody();
	OpenSim::Body tower("tower", mass ,Vec3(0),  mass*SimTK::Inertia::brick(0.1, 1.0, 0.2));
	tower.addDisplayGeometry("box.vtp");
	tower.scale(Vec3(0.1, 1.0, 0.2));

	// Add joint connecting the tower to the ground and associate joint to tower body
	FreeJoint freeJoint("groundTower", ground, Vec3(0), Vec3(0), tower, Vec3(0, -0.5, 0), Vec3(0));
	// Rename coordinates for the free joint
	CoordinateSet &freeCoords = freeJoint.getCoordinateSet();
	for(int i=0; i< freeCoords.getSize(); ++i){
		if(freeCoords[i].getMotionType() == Coordinate::Translational){
			freeCoords[i].setRange(posRange);
		}
		else{
			freeCoords[i].setRange(angRange);
		}	
	}

	// add the tower body to the model
	model.addBody(&tower);

	Random::Uniform diagStiffness(100, 5000);
	Random::Uniform offDiagStiffness(-20, 20);

	Mat66 stiffnessMat(0);
	for(int i=0; i<6; ++i){
		for(int j=i; j<6; ++j){
			stiffnessMat(i,j) = i==j ? diagStiffness.getValue(): offDiagStiffness.getValue();
			if(i==j)
				continue;
			stiffnessMat(j,i) = stiffnessMat(i,j);
		}
	}
	Mat66 dampingMat(0);
	Vec6 stiffnessVec = stiffnessMat.diag();
	Vec3 transStiff = stiffnessVec.getSubVec<3>(3);
	Vec3 rotStiff = stiffnessVec.getSubVec<3>(0);

	model.setGravity(gravity_vec);

	CoupledBushingForce spring("ground", Vec3(0), Vec3(0), "tower", Vec3(0, -0.5, 0), Vec3(0), stiffnessMat, dampingMat);
	//BushingForce spring("ground", Vec3(0), Vec3(0), "tower", Vec3(0, -0.5, 0), Vec3(0), transStiff, rotStiff, Vec3(0), Vec3(0));

	model.addForce(&spring);

	model.print("CoupledBushingEnergyTest.osim");

	model.disownAllComponents();

	SimTK::State &s = model.initSystem();

	Random::Uniform qValues(-0.1, 0.1);
	Random::Uniform qdotValues(-1.0, 1.0);
	for(int i=0; i< model.getNumCoordinates(); ++i){
		freeCoords[i].setValue(s, qValues.getValue());
		freeCoords[i].setSpeedValue(s, qdotValues.getValue());
	}

	model.getMultibodySystem().realize(s, SimTK::Stage::Velocity);
	double ke = model.getMultibodySystem().calcKineticEnergy(s);
	double pes = spring.computePotentialEnergy(s);
	double peg = model.getGravityForce().calcPotentialEnergyContribution(s);
	double E = ke+pes+peg;

    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	integrator.setAccuracy(1e-6);
	integrator.setAbsoluteTolerance(1e-6);
    Manager manager(model,  integrator);

	// Specify the initial and final times of the simulation.
	// In this case, the initial and final times are set based on
	// the range of times over which the controls are available.
	//Control *control;
	manager.setInitialTime(0.0);
	manager.setFinalTime(5.0);

	manager.integrate(s);

	manager.getStateStorage().print("coupled_bushing_energy_model_states.sto");

	model.getMultibodySystem().realize(s, SimTK::Stage::Velocity);
	double ke_f = model.getMultibodySystem().calcKineticEnergy(s);
	double pes_f = spring.computePotentialEnergy(s);
	double peg_f = model.getGravityForce().calcPotentialEnergyContribution(s);
	double E_f = ke_f+pes_f+peg_f;

	ASSERT_EQUAL(E, E_f, 0.001);

	return 0;
}


int testExternalForce()
{
	//define a new model properties
	double mass = 1;
	double angRange[2] = {-Pi, Pi};
	double posRange[2] = {-1, 1};
	
	// construct a new OpenSim model
	Model model;
	model.setName("ExternalForceTest");
	//OpenSim bodies
    OpenSim::Body& ground = model.getGroundBody();
	OpenSim::Body tower("tower", mass ,Vec3(0),  mass*SimTK::Inertia::brick(0.1, 1.0, 0.2));
	tower.addDisplayGeometry("box.vtp");
	tower.scale(Vec3(0.1, 1.0, 0.2));

	// Add joint connecting the tower to the ground and associate joint to tower body
	FreeJoint freeJoint("groundTower", ground, Vec3(0), Vec3(0), tower, Vec3(0, -0.5, 0), Vec3(0));
	// Rename coordinates for the free joint
	CoordinateSet &freeCoords = freeJoint.getCoordinateSet();
	for(int i=0; i< freeCoords.getSize(); ++i){
		if(freeCoords[i].getMotionType() == Coordinate::Translational){
			freeCoords[i].setRange(posRange);
		}
		else{
			freeCoords[i].setRange(angRange);
		}
		freeCoords[i].setDefaultValue(0);
	}

	// add the tower body to the model
	model.addBody(&tower);

	// Force is 10N in Y, Torque is 2Nm about Z and Point is 0.1m in X
	Vec3 force(0,10,0), torque(0, 0, 2), point(0.1, 0, 0);
	Storage forces("external_force_data.sto");
	forces.setName("ExternalForcesData");

	/***************************** CASE 1 ************************************/
	// Apply force with both force and point specified in ground and no torque
	ExternalForce xf(forces, "force", "point", "", "tower", "ground", "ground");
	
	model.addForce(&xf);
	model.print("ExternalForceTest.osim");
	ForceReporter frp;
	model.addAnalysis(&frp);
	// Everything allocated on the stack, so no need for model to own to free
	model.disownAllComponents();

	SimTK::State &s = model.initSystem();

	// set the starting location of the tower to be right over the point
	freeCoords[3].setValue(s, 0.1);

	double accuracy = 1e-6;
    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	integrator.setAccuracy(accuracy);
	integrator.setAbsoluteTolerance(accuracy);
    Manager manager(model,  integrator);

	// Specify the initial and final times of the simulation.
	double tf = 2.0;
	manager.setInitialTime(0.0);
	manager.setFinalTime(tf);
	manager.integrate(s);

	manager.getStateStorage().print("external_force_test_model_states.sto");;

	Vec3 a_y = force/mass + model.getGravity();
	double d_y = 0.5*(a_y.norm())*(tf*tf);

	double y_sim = model.getCoordinateSet()[4].getValue(s);

	// Vertical displacement
	ASSERT_EQUAL(d_y, y_sim, 10*accuracy);
	// all rotations should remain zero
	for(int i=0; i<3; i++){
		double val = model.getCoordinateSet()[i].getValue(s);
		ASSERT_EQUAL(0.0, val, 10*accuracy);
	}
	ASSERT_EQUAL(point[0], model.getCoordinateSet()[3].getValue(s), 10*accuracy);

	model.updForceSet().setSize(0);

	/***************************** CASE 2 ************************************/
	// Apply force with both force and point specified in ground as well as torque
	ExternalForce xf2(forces, "force", "point", "torque", "tower", "ground", "ground");

	model.addForce(&xf2);
	model.print("ExternalForceTest.osim");
	// Everything allocated on the stack, so no need for model to own to free
	model.disownAllComponents();

	// recreate a new underlying system with corresponding state
	SimTK::State s2 = model.initSystem();

	// set the starting location of the tower to be offset as to counter-balance the torque
	// point is 0.1, by moving fwd to 0.3, force has -0.2m moment-arm to generate -2Nm
	freeCoords[3].setValue(s2, 0.3);

    RungeKuttaMersonIntegrator integrator2(model.getMultibodySystem());
	integrator2.setAccuracy(accuracy);
	integrator2.setAbsoluteTolerance(accuracy);
    manager.setIntegrator(&integrator2);
	manager.integrate(s2);

	// all dofs should remain constant
	for(int i=0; i<model.getCoordinateSet().getSize(); i++){
		double val = model.getCoordinateSet()[i].getValue(s2);
		double def = model.getCoordinateSet()[i].getDefaultValue();
		if(i==4){ //Y-direction
			ASSERT_EQUAL(d_y, val, 10*accuracy);
		}else{
			ASSERT_EQUAL(def, val, 10*accuracy);
		}
	}

	model.updForceSet().setSize(0);
	/***************************** CASE 3 ************************************/
	// Apply force with only force (and torque) in ground but point on body
	ExternalForce xf3(forces, "force", "point", "torque", "tower", "ground", "tower");
	// Also Apply force with both force (and torque) and point in ground 
	ExternalForce xf4(forces, "force", "point", "", "tower", "ground", "ground");
	
	model.addForce(&xf3);
	model.addForce(&xf4);
	model.print("ExternalForceTest.osim");
	// Everything allocated on the stack, so no need for model to own to free
	model.disownAllComponents();

	// recreate a new underlying system with corresponding state
	SimTK::State s3 = model.initSystem();

	// only xf4 is should be affected and set it to offset Tz+px*Fy = 2+0.1*10 = 3.
	freeCoords[3].setValue(s3, 0.4); // yield -3Nm for force only

    RungeKuttaMersonIntegrator integrator3(model.getMultibodySystem());
	integrator3.setAccuracy(accuracy);
	integrator3.setAbsoluteTolerance(accuracy);
    manager.setIntegrator(&integrator3);
	manager.integrate(s3);

	// all dofs should remain constant except Y
	for(int i=0; i<model.getCoordinateSet().getSize(); i++){
		double val = model.getCoordinateSet()[i].getValue(s3);
		double def = model.getCoordinateSet()[i].getDefaultValue();
		if(i!=4){ //ignore Y-direction
			ASSERT_EQUAL(def, val, 10*accuracy);
		}
	}

	model.updForceSet().setSize(0);
	/***************************** CASE 4 ************************************/
	// Add joint connecting a "sensor" reference to the ground in which to describe
	// the applied external force
	OpenSim::Body sensor("sensor", 1 ,Vec3(0),  SimTK::Inertia::brick(0.1, 0.1, 0.1));
	sensor.addDisplayGeometry("box.vtp");
	sensor.scale(Vec3(0.02, 0.1, 0.01));

	// locate joint at 0.3m above tower COM
	WeldJoint WeldJoint("sensorWeld", ground, Vec3(0, 0.8, 0), Vec3(0), sensor, Vec3(0), Vec3(0, 0, Pi/2));
	// add the sensor body to the model
	model.addBody(&sensor);

	// Apply force with both force and point in sensor body 
	ExternalForce xf5(forces, "force", "point", "", "tower", "sensor", "sensor");
	// Counter-balance with torque only in tower body 
	ExternalForce xf6(forces, "", "", "torque", "tower", "tower", "");

	model.addForce(&xf5);
	model.addForce(&xf6);
	model.print("ExternalForceTest4.osim");
	// Everything allocated on the stack, so no need for model to own to free
	model.disownAllComponents();

	// recreate a new underlying system with corresponding state
	SimTK::State s4 = model.initSystem();
	model.getGravityForce().disable(s4);

	// only xf4 is should be affected and set it to offset Tz+px*Fy = 2+0.1*10 = 3.
	freeCoords[3].setValue(s4, 0);

    RungeKuttaMersonIntegrator integrator4(model.getMultibodySystem());
	integrator4.setAccuracy(accuracy);
	integrator4.setAbsoluteTolerance(accuracy);
    manager.setIntegrator(&integrator4);
	manager.integrate(s4);

	// all dofs should remain constant except X-translation
	for(int i=0; i<model.getCoordinateSet().getSize(); i++){
		double val = model.getCoordinateSet()[i].getValue(s4);
		double def = model.getCoordinateSet()[i].getDefaultValue();
		if(i !=3)
			ASSERT_EQUAL(def, val, 10*accuracy);
	}

	return 0;
}






int main()
{
	testExternalForce();

	testCoupledBushingForceEnergy();

	testCoupledBushingForce();

	testSpringMass();
	cout << "spring passed"  << endl;
	
	testBushingForce();
	cout << "bushing passed"  << endl;

	testElasticFoundation();
	cout << "elastic foundation force passed"  << endl;
	testHuntCrossleyForce();
	cout << "Hunt-Crossley force passed"  << endl;

	testCoordinateLimitForce();
	cout << "coordinate limit force passed"  << endl;

	return 0;
}
