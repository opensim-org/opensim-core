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
//		2. BusingForce
//		3. ElasticFoundationForce
//		4. HuntCrossleyForce
//		5. CoordinateLimitForce
//		
//     Add tests here as Forces are added to OpenSim
//
//==========================================================================================================
#include <OpenSim/OpenSim.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC


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
	Model *osimModel = new Model("BouncingBallModel.osim");
	
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

int main()
{
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
