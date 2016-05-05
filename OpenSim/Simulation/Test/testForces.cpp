/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testForces.cpp                          *
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

//==============================================================================
//
//  Tests Include:
//      1. PointToPointSpring
//      2. BushingForce
//      3. ElasticFoundationForce
//      4. HuntCrossleyForce
//      5. CoordinateLimitForce
//      6. RotationalCoordinateLimitForce
//      7. ExternalForce
//      8. PathSpring
//      9. ExpressionBasedPointToPointForce
//      
//     Add tests here as Forces are added to OpenSim
//
//==============================================================================
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Analyses/osimAnalyses.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

//==============================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-4;
const static SimTK::Vec3 gravity_vec = SimTK::Vec3(0, -9.8065, 0);
//==============================================================================

void testPathSpring();
void testExternalForce();
void testSpringMass();
void testBushingForce();
void testFunctionBasedBushingForce();
void testExpressionBasedBushingForceTranslational();
void testExpressionBasedBushingForceRotational();
void testElasticFoundation();
void testHuntCrossleyForce();
void testCoordinateLimitForce();
void testCoordinateLimitForceRotational();
void testExpressionBasedPointToPointForce();
void testExpressionBasedCoordinateForce();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testPathSpring(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testPathSpring");
    }
        
    try { testExternalForce(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testExternalForce");
    }

    try { testSpringMass(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testP2PSpringMass");
    }

    try { testBushingForce(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testBushingForce");
    }

    try { testFunctionBasedBushingForce(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; 
        failures.push_back("testFunctionBasedBushingForce");
    }

    try { testExpressionBasedBushingForceTranslational(); }
    catch (const std::exception& e){
        cout << e.what() <<endl;
        failures.push_back("testExpressionBasedBushingForceTranslational");
    }

    try { testExpressionBasedBushingForceRotational(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testExpressionBasedBushingForceRotational");
    }

    try { testElasticFoundation(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testElasticFoundation");
    }

    try { testHuntCrossleyForce(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testHuntCrossleyForce");
    }

    try { testCoordinateLimitForce(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testCoordinateLimitForce");
    }

    try { testCoordinateLimitForceRotational(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; 
        failures.push_back("testCoordinateLimitForceRotational");
    }

    try { testExpressionBasedPointToPointForce(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; 
        failures.push_back("testExpressionBasedPointToPointForce");
    }

    try { testExpressionBasedCoordinateForce(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; 
        failures.push_back("testExpressionBasedCoordinateForce");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done. All cases passed." << endl;

    return 0;
}

//==============================================================================
// Test Cases
//==============================================================================

void testExpressionBasedCoordinateForce()
{
    using namespace SimTK;

    double mass = 1;
    double stiffness = 10;
    double damp_coeff = 5;
    double start_h = 0.5;
    double start_v = 0;
    double ball_radius = 0.25;

    double omega = sqrt(stiffness/mass);
    // note: test case designed for 0 <= zeta < 1 (under damped system) 
    double zeta = damp_coeff / (2*sqrt(mass*stiffness));
    double damp_freq = omega*sqrt(1-pow(zeta, 2));

    double dh = mass*gravity_vec(1)/stiffness;

    // Setup OpenSim model
    Model osimModel{};
    osimModel.setName("SpringMass");
    //OpenSim bodies
    const Ground& ground = osimModel.getGround();;
    OpenSim::Body ball("ball", mass ,Vec3(0),  mass*SimTK::Inertia::sphere(0.1));
    ball.attachGeometry(Sphere{0.5});
    ball.scale(Vec3(ball_radius), false);

    // Add joints
    SliderJoint slider("slider", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));

    double positionRange[2] = {-10, 10};
    // Rename coordinates for a slider joint
    CoordinateSet &slider_coords = slider.upd_CoordinateSet();
    slider_coords[0].setName("ball_h");
    slider_coords[0].setRange(positionRange);

    osimModel.addBody(&ball);
    osimModel.addJoint(&slider);

    osimModel.setGravity(gravity_vec);

    // ode for basic mass-spring-dampener system
    ExpressionBasedCoordinateForce spring("ball_h", "-10*q-5*qdot");

    osimModel.addForce(&spring);

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);

    SimTK::State& osim_state = osimModel.initSystem();

    // move ball to initial conditions
    slider_coords[0].setValue(osim_state, start_h);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Position );

    //==========================================================================
    // Compute the force at the specified times.
    
    double final_t = 2.0;
    double nsteps = 10;
    double dt = final_t/nsteps;

    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem() );
    integrator.setAccuracy(1e-7);
    Manager manager(osimModel,  integrator);
    manager.setInitialTime(0.0);

    for(int i = 1; i <=nsteps; i++){
        manager.setFinalTime(dt*i);
        manager.integrate(osim_state);
        osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos;
        osimModel.updSimbodyEngine().getPosition(osim_state, ball, Vec3(0), pos);
        
        double height = exp(-1*zeta*omega*osim_state.getTime()) *
                        (
                            (start_h-dh)*cos(damp_freq*osim_state.getTime())
                            +
                            (
                                (1/damp_freq)*(zeta*omega*(start_h-dh)+start_v)
                                *
                                sin(damp_freq*osim_state.getTime())
                            )
                        )  
                        + dh;

        ASSERT_EQUAL(height, pos(1), 1e-6);

        manager.setInitialTime(dt*i);
    }
    
    // Test copying
    ExpressionBasedCoordinateForce *copyOfSpring = spring.clone();

    ASSERT(*copyOfSpring == spring);

    osimModel.print("ExpressionBasedCoordinateForceModel.osim");

    osimModel.disownAllComponents();
}

void testExpressionBasedPointToPointForce()
{
    using namespace SimTK;

    double mass = 100;
    double ball_radius = 0.25;

    Random::Uniform rand;
    Vec3 p1(rand.getValue(), rand.getValue(), rand.getValue());
    Vec3 p2(rand.getValue(), rand.getValue(), rand.getValue());

    // Setup OpenSim model
    Model model{};
    model.setName("ExpressionBasedPointToPointForce");
    //OpenSim bodies
    const Ground& ground = model.getGround();
    OpenSim::Body ball("ball", mass, Vec3(0), mass*SimTK::Inertia::sphere(ball_radius));
    ball.attachGeometry(Sphere{0.5});
    ball.scale(Vec3(ball_radius), false);

    // define body's joint
    FreeJoint free("free", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));
    
    model.addBody(&ball);
    model.addJoint(&free);

    string expression("2/(d^2)-3.0*(d-0.2)*(1+0.0123456789*ddot)");

    ExpressionBasedPointToPointForce* p2pForce = 
        new ExpressionBasedPointToPointForce("ground", p1, "ball", p2, expression);
    p2pForce->setName("P2PTestForce");

    model.addForce(p2pForce);

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&model);
    model.addAnalysis(reporter);

    //model.setUseVisualizer(true);
    SimTK::State& state = model.initSystem();

    model.print("ExpressionBasedPointToPointForceModel.osim");

    Vector& q = state.updQ();
    Vector& u = state.updU();

    for(int i=0; i < state.getNU(); ++i){
        q[i] = rand.getValue();
        u[i] = rand.getValue();
    }

    //==========================================================================
    // Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem() );
    integrator.setAccuracy(1e-6);
    Manager manager(model,  integrator);
    manager.setInitialTime(0.0);

    double final_t = 1.0;

    manager.setFinalTime(final_t);
    manager.integrate(state);

    //manager.getStateStorage().print("testExpressionBasedPointToPointForce.sto");

    // force is only velocity dependent but is only compute in Dynamics
    model.getMultibodySystem().realize(state, Stage::Dynamics);

    // Now check that the force reported by spring
    double model_force = p2pForce->getForceMagnitude(state);
    
    // Save the forces
    //reporter->getForceStorage().print("path_spring_forces.mot");
    double d = model.getSimbodyEngine().calcDistance(state, ground, p1, ball, p2);
    const MobilizedBody& b1 = ground.getMobilizedBody();
    const MobilizedBody& b2 = ball.getMobilizedBody();

    double ddot = b1.calcStationToStationDistanceTimeDerivative(state, p1, b2, p2);


    //string expression("2/(d^2)-3.0*(d-0.2)*(1+0.0123456789*ddot)");
    double analytical_force = 2/(d*d)-3.0*(d-0.2)*(1+0.0123456789*ddot);
    
    // something is wrong if the block does not reach equilibrium
    ASSERT_EQUAL(analytical_force, model_force, 1e-5);

    // Before exiting lets see if copying the P2P force works
    ExpressionBasedPointToPointForce *copyOfP2pForce = p2pForce->clone();
    ASSERT(*copyOfP2pForce == *p2pForce);

    model.disownAllComponents();
}

void testPathSpring()
{
    using namespace SimTK;

    double mass = 1;
    double stiffness = 10;
    double restlength = 0.5;
    double dissipation = 0.1;
    double start_h = 0.5;

    // Setup OpenSim model
    Model osimModel{};
    osimModel.setName("PathSpring");
    //OpenSim bodies
    const Ground& ground = osimModel.getGround();;
    OpenSim::Body pulleyBody("PulleyBody", mass ,Vec3(0),  mass*SimTK::Inertia::brick(0.1, 0.1, 0.1));
    OpenSim::Body block("block", mass ,Vec3(0),  mass*SimTK::Inertia::brick(0.2, 0.1, 0.1));
    block.attachGeometry(Sphere{0.5});
    block.scale(Vec3(0.2, 0.1, 0.1), false);
    
    WrapCylinder* pulley = new WrapCylinder();
    pulley->setRadius(0.1);
    pulley->setLength(0.05);

    // Add the wrap object to the body, which takes ownership of it
    pulleyBody.addWrapObject(pulley);

    // Add joints
    WeldJoint weld("pulley", ground, Vec3(0, 1.0, 0), Vec3(0), pulleyBody, Vec3(0), Vec3(0));
    SliderJoint slider("slider", ground, Vec3(0), Vec3(0,0,Pi/2), block, Vec3(0), Vec3(0,0,Pi/2));

    double positionRange[2] = {-10, 10};
    // Rename coordinates for a slider joint
    CoordinateSet &slider_coords = slider.upd_CoordinateSet();
    slider_coords[0].setName("block_h");
    slider_coords[0].setRange(positionRange);

    osimModel.addBody(&block);
    osimModel.addJoint(&weld);

    osimModel.addBody(&pulleyBody);
    osimModel.addJoint(&slider);

    osimModel.setGravity(gravity_vec);

    PathSpring spring("spring", restlength, stiffness, dissipation);
    spring.updGeometryPath().appendNewPathPoint("origin", block, Vec3(-0.1, 0.0 ,0.0));
    
    int N = 10;
    for(int i=1; i<N; ++i){
        double angle = i*Pi/N;
        double x = 0.1*cos(angle);
        double y = 0.1*sin(angle);
        spring.updGeometryPath().appendNewPathPoint("", pulleyBody, Vec3(-x, y ,0.0));
    }

    spring.updGeometryPath().appendNewPathPoint("insertion", block, Vec3(0.1, 0.0 ,0.0));

    // BUG in defining wrapping API requires that the Force containing the GeometryPath be
    // connected to the model before the wrap can be added
    osimModel.addForce(&spring);

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);

    //osimModel.setUseVisualizer(true);
    SimTK::State& osim_state = osimModel.initSystem();

    slider_coords[0].setValue(osim_state, start_h);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Position );

    //==========================================================================
    // Compute the force and torque at the specified times.
    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem() );
    integrator.setAccuracy(1e-6);
    Manager manager(osimModel,  integrator);
    manager.setInitialTime(0.0);

    double final_t = 10.0;

    manager.setFinalTime(final_t);
    manager.integrate(osim_state);

    // tension should only be velocity dependent
    osimModel.getMultibodySystem().realize(osim_state, Stage::Velocity);

    // Now check that the force reported by spring
    double model_force = spring.getTension(osim_state);

    // get acceleration of the block
    osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);
    double hddot = osimModel.getCoordinateSet().get("block_h").getAccelerationValue(osim_state);

    // the tension should be half the weight of the block
    double analytical_force = -0.5*(gravity_vec(1)-hddot)*mass;

    // Save the forces
    reporter->getForceStorage().print("path_spring_forces.mot");  
    
    // something is wrong if the block does not reach equilibrium
    ASSERT_EQUAL(analytical_force, model_force, 1e-3);

    // Before exiting lets see if copying the spring works
    PathSpring *copyOfSpring = spring.clone();
    ASSERT(*copyOfSpring == spring);
    
    osimModel.disownAllComponents();
}

void testSpringMass()
{
    using namespace SimTK;

    double mass = 1;
    double stiffness = 10;
    double restlength = 1.0;
    double start_h = 0.5;
    double ball_radius = 0.25;

    double omega = sqrt(stiffness/mass);

    double dh = mass*gravity_vec(1)/stiffness;

    // Setup OpenSim model
    Model osimModel{};
    osimModel.setName("SpringMass");
    //OpenSim bodies
    const Ground& ground = osimModel.getGround();;
    OpenSim::Body ball("ball", mass ,Vec3(0),  mass*SimTK::Inertia::sphere(0.1));
    ball.attachGeometry(Sphere{0.5});
    ball.scale(Vec3(ball_radius), false);

    // Add joints
    SliderJoint slider("slider", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));

    double positionRange[2] = {-10, 10};
    // Rename coordinates for a slider joint
    CoordinateSet &slider_coords = slider.upd_CoordinateSet();
    slider_coords[0].setName("ball_h");
    slider_coords[0].setRange(positionRange);

    osimModel.addBody(&ball);
    osimModel.addJoint(&slider);

    osimModel.setGravity(gravity_vec);

    PointToPointSpring spring(osimModel.updGround(), 
        Vec3(0.,restlength,0.), 
        ball, 
        Vec3(0.), 
        stiffness, 
        restlength);

    osimModel.addForce(&spring);

    //osimModel.print("SpringMassModel.osim");

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);

    SimTK::State& osim_state = osimModel.initSystem();

    slider_coords[0].setValue(osim_state, start_h);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Position );

    //==========================================================================
    // Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem() );
    integrator.setAccuracy(1e-7);
    Manager manager(osimModel,  integrator);
    manager.setInitialTime(0.0);

    double final_t = 2.0;
    double nsteps = 10;
    double dt = final_t/nsteps;

    for(int i = 1; i <=nsteps; i++){
        manager.setFinalTime(dt*i);
        manager.integrate(osim_state);
        osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos;
        osimModel.updSimbodyEngine().getPosition(osim_state, ball, Vec3(0), pos);
        
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
    osimModel.disownAllComponents();

    // Before exiting lets see if copying the spring works
    PointToPointSpring *copyOfSpring = spring.clone();

    ASSERT(*copyOfSpring == spring);
}

void testBushingForce()
{
    using namespace SimTK;

    double mass = 1;
    double stiffness = 10;
    double start_h = 0.5;
    double ball_radius = 0.25;

    double omega = sqrt(stiffness/mass);

    double dh = mass*gravity_vec(1)/stiffness;

    // Setup OpenSim model
    Model osimModel{};
    osimModel.setName("BushingTest");
    //OpenSim bodies
    const Ground& ground = osimModel.getGround();;
    OpenSim::Body ball("ball", mass, Vec3(0), mass*SimTK::Inertia::sphere(0.1));
    ball.attachGeometry(Sphere{0.5});
    ball.scale(Vec3(ball_radius), false);

    // Add joints
    SliderJoint slider("slider", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));

    double positionRange[2] = {-10, 10};
    // Rename coordinates for a slider joint
    CoordinateSet &slider_coords = slider.upd_CoordinateSet();
    slider_coords[0].setName("ball_h");
    slider_coords[0].setRange(positionRange);

    osimModel.addBody(&ball);
    osimModel.addJoint(&slider);

    Vec3 rotStiffness(0);
    Vec3 transStiffness(stiffness);
    Vec3 rotDamping(0);
    Vec3 transDamping(0);

    osimModel.setGravity(gravity_vec);

    BushingForce spring("bushing", "ground", "ball",
        transStiffness, rotStiffness, transDamping, rotDamping);

    osimModel.addForce(&spring);
    const BushingForce& bushingForce =
        osimModel.getComponent<BushingForce>("bushing");

    osimModel.print("BushingForceModel.osim");

    Model previousVersionModel("BushingForceModel_30000.osim", true);
    previousVersionModel.print("BushingForceModel_30000_in_Latest.osim");

    const BushingForce& bushingForceFromPrevious =
        previousVersionModel.getComponent<BushingForce>("bushing");

    ASSERT(bushingForce == bushingForceFromPrevious, __FILE__, __LINE__,
        "current bushing force FAILED to match bushing force from previous model.");

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);

    SimTK::State& osim_state = osimModel.initSystem();

    slider_coords[0].setValue(osim_state, start_h);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Position );

    //==========================================================================
    // Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem() );
    integrator.setAccuracy(1e-6);
    Manager manager(osimModel,  integrator);
    manager.setInitialTime(0.0);

    double final_t = 2.0;
    double nsteps = 10;
    double dt = final_t/nsteps;

    for(int i = 1; i <=nsteps; i++){
        manager.setFinalTime(dt*i);
        manager.integrate(osim_state);
        osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos;
        osimModel.updSimbodyEngine().getPosition(osim_state, ball, Vec3(0), pos);
        
        double height = (start_h-dh)*cos(omega*osim_state.getTime())+dh;
        ASSERT_EQUAL(height, pos(1), 1e-4);

        //Now check that the force reported by spring
        Array<double> model_force = spring.getRecordValues(osim_state);

        // get the forces applied to the ground and ball
        double analytical_force = -stiffness*height;
        // analytical force corresponds in direction to the force on the ball Y index = 7
        ASSERT_EQUAL(analytical_force, model_force[7], 2e-4);

        manager.setInitialTime(dt*i);
    }

    manager.getStateStorage().print("bushing_model_states.sto");

    // Save the forces
    reporter->getForceStorage().print("bushing_forces.mot");  

    // Before exiting lets see if copying the spring works
    BushingForce *copyOfSpring = spring.clone();

    ASSERT(*copyOfSpring == spring);

    osimModel.disownAllComponents();
}

void testFunctionBasedBushingForce()
{
    using namespace SimTK;

    double mass = 1;
    double stiffness = 10;
    double start_h = 0.5;
    double ball_radius = 0.25;

    double omega = sqrt(stiffness/mass);

    double dh = mass*gravity_vec(1)/stiffness;

    // Setup OpenSim model
    Model osimModel{};
    osimModel.setName("FunctionBasedBushingTest");
    //OpenSim bodies
    const Ground& ground = osimModel.getGround();;
    OpenSim::Body ball("ball", mass, Vec3(0), mass*SimTK::Inertia::sphere(0.1));
    ball.attachGeometry(Sphere{0.5});
    ball.scale(Vec3(ball_radius), false);

    // Add joints
    SliderJoint slider("slider", ground, Vec3(0), Vec3(0,0,Pi/2), 
                                 ball, Vec3(0), Vec3(0,0,Pi/2));

    double positionRange[2] = {-10, 10};
    // Rename coordinates for a slider joint
    CoordinateSet &slider_coords = slider.upd_CoordinateSet();
    slider_coords[0].setName("ball_h");
    slider_coords[0].setRange(positionRange);

    osimModel.addBody(&ball);
    osimModel.addJoint(&slider);

    Vec3 rotStiffness(0);
    Vec3 transStiffness(stiffness);
    Vec3 rotDamping(0);
    Vec3 transDamping(0);

    osimModel.setGravity(gravity_vec);

    FunctionBasedBushingForce spring("linear_bushing",
                    "ground", Vec3(0), Vec3(0), 
                    "ball", Vec3(0), Vec3(0),
                    transStiffness, rotStiffness, transDamping, rotDamping);

    osimModel.addForce(&spring);

    osimModel.print("FunctionBasedBushingForceModel.osim");

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);

    SimTK::State& osim_state = osimModel.initSystem();

    slider_coords[0].setValue(osim_state, start_h);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Position );

    //==========================================================================
    // Compute the force and torque at the specified times.
    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem() );
    integrator.setAccuracy(1e-6);
    Manager manager(osimModel,  integrator);
    manager.setInitialTime(0.0);

    double final_t = 2.0;
    double nsteps = 10;
    double dt = final_t/nsteps;

    for(int i = 1; i <=nsteps; i++){
        manager.setFinalTime(dt*i);
        manager.integrate(osim_state);
        osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos;
        osimModel.updSimbodyEngine().getPosition(osim_state, ball, Vec3(0), pos);
        
        double height = (start_h-dh)*cos(omega*osim_state.getTime())+dh;
        ASSERT_EQUAL(height, pos(1), 1e-4);

        //Now check that the force reported by spring
        Array<double> model_force = spring.getRecordValues(osim_state);

        // get the forces applied to the ground and ball
        double analytical_force = -stiffness*height;
        // analytical force corresponds in direction to the force on the ball Y index = 7
        ASSERT_EQUAL(analytical_force, model_force[7], 2e-4);

        manager.setInitialTime(dt*i);
    }

    osimModel.disownAllComponents();

    manager.getStateStorage().print("function_based_bushing_model_states.sto");

    // Save the forces
    reporter->getForceStorage().print("function_based_bushing_forces.mot");  

    // Before exiting lets see if copying the spring works
    FunctionBasedBushingForce *copyOfSpring = spring.clone();

    ASSERT(*copyOfSpring == spring);
}

void testExpressionBasedBushingForceTranslational()
{
    using namespace SimTK;
    
    double mass = 1;
    double stiffness = 10;
    double start_h = 0.5;
    double ball_radius = 0.25;
    
    double omega = sqrt(stiffness/mass);
    
    double dh = mass*gravity_vec(1)/stiffness;
    
    // Setup OpenSim model
    Model osimModel{};
    osimModel.setName("ExpressionBasedBushingTranslationTest");
    osimModel.setGravity(gravity_vec);

    // Create ball body and attach it to ground
    // with a vertical slider

    const Ground& ground = osimModel.getGround();

    OpenSim::Body ball("ball", mass, Vec3(0), mass*SimTK::Inertia::sphere(0.1));
    ball.attachGeometry(Sphere{0.5});
    ball.scale(Vec3(ball_radius), false);
    
    SliderJoint sliderY("slider", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));
    
    double positionRange[2] = {-10, 10};
    // Rename coordinates for a slider joint
    CoordinateSet &slider_coords = sliderY.upd_CoordinateSet();
    slider_coords[0].setName("ball_h");
    slider_coords[0].setRange(positionRange);
    
    osimModel.addBody(&ball);
    osimModel.addJoint(&sliderY);
    
    // Create base body and attach it to ground with a weld

    OpenSim::Body base("base_body", mass, Vec3(0), mass*SimTK::Inertia::sphere(0.1));
    base.attachGeometry(Sphere{0.5});
    base.scale(Vec3(ball_radius), false);
    
    WeldJoint weld("weld", ground, Vec3(0), Vec3(0), base, Vec3(0), Vec3(0));
    osimModel.addBody(&base);
    osimModel.addJoint(&weld);
    
    // create an ExpressionBasedBushingForce that represents an
    // uncoupled, linear bushing between the ball body and welded base body
    Vec3 rotStiffness(0);
    Vec3 transStiffness(stiffness);
    Vec3 rotDamping(0);
    Vec3 transDamping(0);
    
    ExpressionBasedBushingForce spring("linear_bushing",
        "base_body", Vec3(0), Vec3(0), 
        "ball", Vec3(0), Vec3(0), 
        transStiffness, rotStiffness, transDamping, rotDamping);
    
    spring.setName("translational_linear_bushing");
    
    osimModel.addForce(&spring);
    
    osimModel.print("ExpressionBasedBushingForceTranslationalModel.osim");
    
    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);
    
    SimTK::State& osim_state = osimModel.initSystem();
    
    // set the initial height of the ball on slider
    slider_coords[0].setValue(osim_state, start_h);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Position );
    
    //==========================================================================
    // Compute the force and torque at the specified times.
    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem() );
    integrator.setAccuracy(1e-6);
    Manager manager(osimModel,  integrator);
    manager.setInitialTime(0.0);
    
    double final_t = 2.0;
    double nsteps = 10;
    double dt = final_t/nsteps;
    
    for(int i = 1; i <=nsteps; ++i){
        manager.setFinalTime(dt*i);
        manager.integrate(osim_state);
        osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos;
        osimModel.updSimbodyEngine().getPosition(osim_state, ball, Vec3(0), pos);
        
        // compute the height based on the analytic solution for 1-D spring-mass
        // system with zero-velocity at initial offset.
        double height = (start_h-dh)*cos(omega*osim_state.getTime())+dh;

        // check that the simulated solution is equivalent to the analytic solution
        ASSERT_EQUAL(height, pos(1), 1e-4);
        
        // get the forces applied to the base and ball
        Array<double> model_force = spring.getRecordValues(osim_state);
        
        // compute the expected force on the ball
        double analytical_force = -stiffness*height;

        // check analytical force corresponds to the force on the ball 
        // in the Y direction, index = 7
        ASSERT_EQUAL(analytical_force, model_force[7], 2e-4);
        
        manager.setInitialTime(dt*i);
    }
    
    osimModel.disownAllComponents();
    
    manager.getStateStorage().print("expression_based_bushing_translational_model_states.sto");
    
    // Save the forces
    reporter->getForceStorage().print("expression_based_bushing_translational_model_forces.mot");
    
    // Before exiting lets see if copying the spring works
    ExpressionBasedBushingForce *copyOfSpring = spring.clone();
    
    ASSERT(*copyOfSpring == spring);
}

void testExpressionBasedBushingForceRotational()
{
    using namespace SimTK;

    double mass = 5;
    double stiffness = 2;
    double start_theta = Pi/6;
    double ball_radius = 0.25;

    // Setup OpenSim model
    Model osimModel{};
    osimModel.setName("ExpressionBasedBushingRotationalTest");
    const Ground& ground = osimModel.getGround();

    // Create base body and attach it to ground with a weld

    OpenSim::Body base("base_body", mass, Vec3(0),
        mass*SimTK::Inertia::sphere(ball_radius));

    WeldJoint weld("weld", ground, Vec3(0), Vec3(0), base, Vec3(0), Vec3(0));
    osimModel.addBody(&base);
    osimModel.addJoint(&weld);

    // Create ball body and attach it to ground
    // with a pin joint

    OpenSim::Body ball("ball", mass, Vec3(0), 
        mass*SimTK::Inertia::sphere(ball_radius));

    PinJoint pin("pin", ground, Vec3(0), Vec3( Pi / 2, 0, 0), 
        ball, Vec3(0), Vec3(Pi / 2, 0, 0));

    double thetaRange[2] = { -2*Pi, 2*Pi };
    // Rename coordinates for a pin joint
    CoordinateSet &pin_coords = pin.upd_CoordinateSet();
    pin_coords[0].setName("ball_theta");
    pin_coords[0].setRange(thetaRange);

    osimModel.addBody(&ball);
    osimModel.addJoint(&pin);

    

    // create an ExpressionBasedBushingForce that represents an
    // uncoupled, linear bushing between the ball body and welded base body
    Vec3 rotStiffness(stiffness);
    Vec3 transStiffness(0);
    Vec3 rotDamping(0);
    Vec3 transDamping(0);

    ExpressionBasedBushingForce spring("rotatinal_spring", 
        "base_body", Vec3(0), Vec3(0),
        "ball", Vec3(0), Vec3(0),
        transStiffness, rotStiffness, transDamping, rotDamping);

    spring.setName("rotational_linear_bushing");

    osimModel.addForce(&spring);

    osimModel.print("ExpressionBasedBushingForceRotationalModel.osim");

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);

    SimTK::State& osim_state = osimModel.initSystem();

    // set the initial pin joint angle
    pin_coords[0].setValue(osim_state, start_theta);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Position);

    //=========================================================================
    // Compute the force and torque at the specified times.
    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
    integrator.setAccuracy(1e-6);
    Manager manager(osimModel, integrator);
    manager.setInitialTime(0.0);

    double final_t = 2.0;
    double nsteps = 10;
    double dt = final_t / nsteps;

    double I_y = ball.getInertia().getMoments()[1];
    
    double omega = sqrt(stiffness / I_y);

    for (int i = 1; i <= nsteps; ++i) {
        manager.setFinalTime(dt*i);
        manager.integrate(osim_state);
        osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);

        // compute the current rotation about the y axis
        double simulated_theta = pin_coords[0].getValue(osim_state);

        // compute the rotation about y-axis from the analytic solution 
        // for 1-D spring-mass system with zero-velocity at initial offset.
        double analytical_theta = start_theta*cos(omega*osim_state.getTime());

        // check that the simulated solution is 
        //  equivalent to the analytic solution
        ASSERT_EQUAL(analytical_theta, simulated_theta, 1e-4);

        // get the forces applied to the base and ball
        Array<double> model_forces = spring.getRecordValues(osim_state);

        // compute the expected force on the ball
        double analytical_moment = -stiffness*analytical_theta;

        // check analytical moment corresponds to the moment on the ball 
        // in the Y direction, index = 4
        ASSERT_EQUAL(analytical_moment, model_forces[4], 2e-4);

        manager.setInitialTime(dt*i);
    }

    osimModel.disownAllComponents();

    manager.getStateStorage().print("expression_based_bushing_rotational_model_states.sto");

    // Save the forces
    reporter->getForceStorage().print("expression_based_bushing_rotational_model_forces.mot");

    // Before exiting lets see if copying the spring works
    ExpressionBasedBushingForce *copyOfSpring = spring.clone();

    ASSERT(*copyOfSpring == spring);
}

// Test our wrapping of elastic foundation in OpenSim
// Simple simulation of bouncing ball with dissipation should generate contact
// forces that settle to ball weight.
void testElasticFoundation()
{
    using namespace SimTK;

    double start_h = 0.5;

    // Setup OpenSim model
    Model osimModel{"BouncingBallModelEF.osim"};
    
    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);

    SimTK::State& osim_state = osimModel.initSystem();

    osimModel.getCoordinateSet().get("ball_ty").setValue(osim_state, start_h);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Position );

    const OpenSim::Body &ball = osimModel.getBodySet().get("ball"); 

    //==========================================================================
    // Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem() );
    integrator.setAccuracy(1e-6);
    Manager manager(osimModel,  integrator);
    manager.setInitialTime(0.0);

    double final_t = 2.0;

    manager.setFinalTime(final_t);

    // start timing
    clock_t startTime = clock();

    manager.integrate(osim_state);

    // end timing
    cout << "Elastic Foundation simulation time = " << 1.e3*(clock()-startTime)/CLOCKS_PER_SEC << "ms" << endl;;

    //make sure we can access dynamic variables
    osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);

    // Print out the motion for visualizing/debugging
    manager.getStateStorage().print("bouncing_ball_states.sto");
        
    // Save the forces
    reporter->getForceStorage().print("elastic_contact_forces.mot"); 

    // Bouncing ball should have settled to rest on ground due to dissipation
    // In that case the force generated by contact should be identically body weight
    // in vertical and zero else where.
    OpenSim::ElasticFoundationForce &contact = 
        (OpenSim::ElasticFoundationForce &)osimModel.getForceSet().get("contact");

    Array<double> contact_force = contact.getRecordValues(osim_state);
    ASSERT_EQUAL(contact_force[0], 0.0, 1e-4); // no horizontal force on the ball
    ASSERT_EQUAL(contact_force[1], -ball.getMass()*gravity_vec[1], 2e-3); // vertical is weight
    ASSERT_EQUAL(contact_force[2], 0.0, 1e-4); // no horizontal force on the ball
    ASSERT_EQUAL(contact_force[3], 0.0, 1e-4); // no torque on the ball
    ASSERT_EQUAL(contact_force[4], 0.0, 1e-4); // no torque on the ball
    ASSERT_EQUAL(contact_force[5], 0.0, 1e-4); // no torque on the ball

    // Before exiting lets see if copying the spring works
    OpenSim::ElasticFoundationForce *copyOfForce = contact.clone();

    bool isEqual = (*copyOfForce == contact);
    
    if(!isEqual){
        contact.print("originalForce.xml");
        copyOfForce->print("copyOfForce.xml");
    }

    ASSERT(isEqual);
}

// Test our wrapping of Hunt-Crossley force in OpenSim
// Simple simulation of bouncing ball with dissipation should generate contact
// forces that settle to ball weight.
void testHuntCrossleyForce()
{
    using namespace SimTK;

    double start_h = 0.5;

    // Setup OpenSim model
    Model osimModel{"BouncingBall_HuntCrossley.osim"};
    
    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);

    SimTK::State& osim_state = osimModel.initSystem();

    osimModel.getCoordinateSet()[4].setValue(osim_state, start_h);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Position );

    const OpenSim::Body &ball = osimModel.getBodySet().get("ball"); 

    //==========================================================================
    // Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem() );
    integrator.setAccuracy(1e-6);
    Manager manager(osimModel,  integrator);
    manager.setInitialTime(0.0);

    double final_t = 2.0;

    manager.setFinalTime(final_t);
    
    // start timing
    clock_t startTime = clock();

    manager.integrate(osim_state);

    // end timing
    cout << "Hunt Crossley simulation time = " << 1.e3*(clock()-startTime)/CLOCKS_PER_SEC << "ms" << endl;
    
    //make sure we can access dynamic variables
    osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);

    // Print out the motion for visualizing/debugging
    manager.getStateStorage().print("bouncing_ball_HC_states.sto");
        
    // Save the forces
    reporter->getForceStorage().print("HuntCrossley_contact_forces.mot"); 

    // Bouncing ball should have settled to rest on ground due to dissipation
    // In that case the force generated by contact should be identically body weight
    // in vertical and zero else where.
    OpenSim::HuntCrossleyForce &contact = (OpenSim::HuntCrossleyForce &)osimModel.getForceSet().get("contact");

    Array<double> contact_force = contact.getRecordValues(osim_state);
    ASSERT_EQUAL(contact_force[0], 0.0, 1e-4); // no horizontal force on the ball
    ASSERT_EQUAL(contact_force[1], -ball.getMass()*gravity_vec[1], 1e-3); // vertical is weight
    ASSERT_EQUAL(contact_force[2], 0.0, 1e-4); // no horizontal force on the ball
    ASSERT_EQUAL(contact_force[3], 0.0, 1e-4); // no torque on the ball
    ASSERT_EQUAL(contact_force[4], 0.0, 1e-4); // no torque on the ball
    ASSERT_EQUAL(contact_force[5], 0.0, 1e-4); // no torque on the ball

    // Before exiting lets see if copying the force works
    OpenSim::HuntCrossleyForce *copyOfForce = contact.clone();

    bool isEqual = (*copyOfForce == contact);
    
    if(!isEqual){
        contact.print("originalForce.xml");
        copyOfForce->print("copyOfForce.xml");
    }

    ASSERT(isEqual);
}


void testCoordinateLimitForce()
{
    using namespace SimTK;

    double mass = 1;
    double ball_radius = 0.25;

    // Setup OpenSim model
    auto osimModel = std::unique_ptr<Model>{new Model};
    osimModel->setName("CoordinateLimitForceTest");
    //OpenSim bodies
    const Ground& ground = osimModel->getGround();;
    OpenSim::Body ball("ball", mass ,Vec3(0),  mass*SimTK::Inertia::sphere(0.1));
    ball.attachGeometry(Sphere{0.5});
    ball.scale(Vec3(ball_radius), false);

    // Add joints
    SliderJoint slider("slider", ground, Vec3(0), Vec3(0,0,Pi/2), ball, Vec3(0), Vec3(0,0,Pi/2));

    double positionRange[2] = {0.1, 2};
    // Rename coordinates for a slider joint
    CoordinateSet &slider_coords = slider.upd_CoordinateSet();
    slider_coords[0].setName("ball_h");
    slider_coords[0].setRange(positionRange);

    osimModel->addBody(&ball);
    osimModel->addJoint(&slider);

    osimModel->setGravity(gravity_vec);

    // Define the parameters of the Coordinate Limit Force
    double K_upper = 200.0;
    double K_lower = 1000.0;
    double damping = 0.01;
    double trans = 0.05;
    CoordinateLimitForce limitForce("ball_h", positionRange[1],  K_upper, 
             positionRange[0], K_lower,  damping, trans, true);

    osimModel->addForce(&limitForce);

    // BAD: have to set memoryOwner to false or program will crash when this test is complete.
    osimModel->disownAllComponents();

    osimModel->print("CoordinateLimitForceTest.osim");

    // Check serialization and deserialization
    Model loadedModel{"CoordinateLimitForceTest.osim"};

    ASSERT(loadedModel == *osimModel,
        "Deserialized CoordinateLimitForceTest failed to be equivalent to original.");

    // check copy
    auto copyModel = std::unique_ptr<Model>{osimModel->clone()};

    ASSERT(*copyModel == loadedModel,
        "Clone of CoordinateLimitForceTest failed to be equivalent to original.");

    copyModel->print("cloneCoordinateLimitForceTest.osim");

    osimModel = std::move(copyModel);
    
    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(osimModel.get());
    osimModel->addAnalysis(reporter);

    SimTK::State& osim_state = osimModel->initSystem();

    double dh = 0.2;
    double start_h = positionRange[1];
    double start_v = 2.0;
    const Coordinate &q_h = osimModel->getCoordinateSet()[0];
    q_h.setValue(osim_state, start_h);
    q_h.setSpeedValue(osim_state, start_v);

    osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration );

    CoordinateLimitForce *clf = dynamic_cast<CoordinateLimitForce *>(&osimModel->getForceSet()[0]);

    // initial energy of the system;
    double clfPE = clf->computePotentialEnergy(osim_state);
    double constStiffnessPE = 0.5*K_upper*dh*dh;
    ASSERT(clfPE < constStiffnessPE);
    double energy0 = clfPE + mass*(-gravity_vec[1])*start_h + 0.5*mass*start_v*start_v;
    // system KE + PE including strain energy in CLF
    double eSys0 = osimModel->getMultibodySystem().calcEnergy(osim_state);


    //==========================================================================
    // Compute the force and torque at the specified times.
    RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem() );
    integrator.setAccuracy(1e-6);
    Manager manager(*osimModel,  integrator);
    manager.setInitialTime(0.0);

    double final_t = 1.0;
    double nsteps = 20;
    double dt = final_t/nsteps;

    for(int i = 1; i <=nsteps; i++){
        manager.setFinalTime(dt*i);
        manager.integrate(osim_state);
        osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);
        
        double h = q_h.getValue(osim_state);
        double v = q_h.getSpeedValue(osim_state);

        //Now check that the force reported by spring
        Array<double> model_force = clf->getRecordValues(osim_state);

        double ediss = clf->getDissipatedEnergy(osim_state);
        double clfE = clf->computePotentialEnergy(osim_state) + ediss;

        // EK + EM of mass alone
        double eMass = 0.5*mass*v*v - mass*gravity_vec[1]*h;
        double e = eMass+clfE; 
        // system KE + PE including strain energy in CLF
        double eSys = osimModel->getMultibodySystem().calcEnergy(osim_state)+ ediss;

        ASSERT_EQUAL(1.0, e/energy0, integ_accuracy, "CoordinateLimitForce Failed to conserve energy");
        ASSERT_EQUAL(1.0, eSys/eSys0, integ_accuracy, "CoordinateLimitForce Failed to conserve system energy");

        // get the forces applied to the ball by the limit force
        if(h > (positionRange[1]+trans)){
            ASSERT_EQUAL(-K_upper*(h-positionRange[1])-damping*v, model_force[0], 1e-4);
        }
        else if( h < (positionRange[0]-trans)){
            ASSERT_EQUAL(K_lower*(positionRange[0]-h)-damping*v, model_force[0], 1e-4);
        }
        else if( (h < positionRange[1]) && (h > positionRange[0])){
            // Verify no force is being applied when within limits 
            ASSERT_EQUAL(0.0, model_force[0], 1e-5);
        }

        manager.setInitialTime(dt*i);
    }

    manager.getStateStorage().print("coordinte_limit_force_model_states.sto");

    // Save the forces
    reporter->getForceStorage().print("limit_forces.mot");
}

void testCoordinateLimitForceRotational()
{
    using namespace SimTK;

    double mass = 1;
    double edge = 0.2;

    // Setup OpenSim model
    Model osimModel{};
    osimModel.setName("RotationalCoordinateLimitForceTest");
    //OpenSim bodies
    const Ground& ground = osimModel.getGround();;
    OpenSim::Body block("block", mass ,Vec3(0),  mass*SimTK::Inertia::brick(edge,edge,edge));
    block.attachGeometry(Sphere{0.5});
    block.scale(Vec3(edge), false);

    // Add joints
    PinJoint pin("pin", ground, Vec3(0), Vec3(0,0,0), block, Vec3(0,-edge,0), Vec3(0,0,0));

    // NOTE: Angular limits are in degrees NOT radians
    double positionRange[2] = {-30, 90};
    // Rename coordinates for a slider joint
    CoordinateSet &pin_coords = pin.upd_CoordinateSet();
    pin_coords[0].setName("theta");
    pin_coords[0].setRange(positionRange);

    osimModel.addBody(&block);
    osimModel.addJoint(&pin);

    osimModel.setGravity(Vec3(0));

    // Define the parameters of the Coordinate Limit Force
    // For rotational coordinates, these are in Nm/degree
    double K_upper = 10.0;
    double K_lower = 20.0;
    double damping = 0.1;
    double trans = 0.05;
    CoordinateLimitForce limitForce("theta", positionRange[1],  K_upper, 
             positionRange[0], K_lower,  damping, trans, true);

    osimModel.addForce(&limitForce);

    // BAD: have to set memoryOwner to false or program will crash when this test is complete.
    osimModel.disownAllComponents();

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(&osimModel);
    osimModel.addAnalysis(reporter);

    SimTK::State& osim_state = osimModel.initSystem();

    // Start 2 degrees beyond the upper limit
    double start_q = SimTK_DEGREE_TO_RADIAN*positionRange[1] + SimTK::Pi/90;
    double start_v = 0.0;
    const Coordinate &coord = osimModel.getCoordinateSet()[0];
    coord.setValue(osim_state, start_q);
    coord.setSpeedValue(osim_state, start_v);

    osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration );

    CoordinateLimitForce *clf = dynamic_cast<CoordinateLimitForce *>(&osimModel.getForceSet()[0]);

    //Now check that the force reported by spring
    Array<double> model_force = clf->getRecordValues(osim_state);

    ASSERT_EQUAL(model_force[0]/(-2*K_upper), 1.0, integ_accuracy);

    double clfPE = clf->computePotentialEnergy(osim_state);
    double constSpringPE = 0.5*(K_upper*2.0)*2.0*SimTK_DEGREE_TO_RADIAN;
    ASSERT_EQUAL(clfPE/constSpringPE, 1.0, 0.001, "Specified upper rotational stiffness not met.");
    ASSERT(clfPE < constSpringPE);

    //Now test lower bound
    start_q = SimTK_DEGREE_TO_RADIAN*positionRange[0] - SimTK::Pi/90;
    coord.setValue(osim_state, start_q);
    osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration );
    model_force = clf->getRecordValues(osim_state);

    ASSERT_EQUAL(model_force[0]/(2*K_lower), 1.0, integ_accuracy);

    clfPE = clf->computePotentialEnergy(osim_state);
    constSpringPE = 0.5*(K_lower*2.0)*2.0*SimTK_DEGREE_TO_RADIAN;
    ASSERT_EQUAL(clfPE/constSpringPE, 1.0, 0.001);
    ASSERT(clfPE < constSpringPE);

    // total system energy prior to simulation
    double eSys0 = osimModel.getMultibodySystem().calcEnergy(osim_state);

    //==========================================================================
    // Perform a simulation an monitor energy conservation

    RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem() );
    integrator.setAccuracy(1e-8);
    Manager manager(osimModel,  integrator);
    manager.setInitialTime(0.0);

    double final_t = 1.0;
    double nsteps = 20;
    double dt = final_t/nsteps;

    for(int i = 1; i <=nsteps; i++){
        manager.setFinalTime(dt*i);
        manager.integrate(osim_state);
        osimModel.getMultibodySystem().realize(osim_state, Stage::Acceleration);

        double ediss = clf->getDissipatedEnergy(osim_state);
        // system KE + PE including strain energy in CLF
        double eSys = osimModel.getMultibodySystem().calcEnergy(osim_state)+ ediss;
        /*double EKsys = */osimModel.getMultibodySystem().calcKineticEnergy(osim_state);

        ASSERT_EQUAL(eSys/eSys0, 1.0, integ_accuracy);

        manager.setInitialTime(dt*i);
    }

    manager.getStateStorage().print("rotational_coordinte_limit_force_states.sto");

    // Save the forces
    reporter->getForceStorage().print("rotational_limit_forces.mot");
}



void testExternalForce()
{
    using namespace SimTK;

    //define a new model properties
    double mass = 1;
    double angRange[2] = {-Pi, Pi};
    double posRange[2] = {-1, 1};
    
    // construct a new OpenSim model
    Model model;
    model.setName("ExternalForceTest");
    //OpenSim bodies
    const Ground& ground = model.getGround();
    OpenSim::Body tower("tower", mass, Vec3(0), mass*SimTK::Inertia::brick(0.1, 1.0, 0.2));
    tower.attachGeometry(Sphere{0.5});
    tower.scale(Vec3(0.1, 1.0, 0.2));

    // Add joint connecting the tower to the ground and associate joint to tower body
    FreeJoint freeJoint("groundTower", ground, Vec3(0), Vec3(0), tower, Vec3(0, -0.5, 0), Vec3(0));
    // Rename coordinates for the free joint
    CoordinateSet &freeCoords = freeJoint.upd_CoordinateSet();
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
    model.addJoint(&freeJoint);

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

    SimTK::State& s = model.initSystem();

    // set the starting location of the tower to be right over the point
    freeCoords[3].setValue(s, 0.1);

    double accuracy = 1e-6;
    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    integrator.setAccuracy(accuracy);
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
    SimTK::State& s2 = model.initSystem();

    // set the starting location of the tower to be offset as to counter-balance the torque
    // point is 0.1, by moving fwd to 0.3, force has -0.2m moment-arm to generate -2Nm
    freeCoords[3].setValue(s2, 0.3);
    model.setPropertiesFromState(s2);

    RungeKuttaMersonIntegrator integrator2(model.getMultibodySystem());
    integrator2.setAccuracy(accuracy);

    manager.setIntegrator(integrator2);
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

    // Everything allocated on the stack, so no need for model to own to free
    model.disownAllComponents();

    // recreate a new underlying system with corresponding state
    SimTK::State& s3 = model.initSystem();

    // only xf4 is should be affected and set it to offset Tz+px*Fy = 2+0.1*10 = 3.
    freeCoords[3].setValue(s3, 0.4); // yield -3Nm for force only
    model.setPropertiesFromState(s3);

    RungeKuttaMersonIntegrator integrator3(model.getMultibodySystem());
    integrator3.setAccuracy(accuracy);

    manager.setIntegrator(integrator3);
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
    sensor.attachGeometry(Sphere{0.5});
    sensor.scale(Vec3(0.02, 0.1, 0.01));

    // locate joint at 0.3m above tower COM
    WeldJoint weldJoint("sensorWeld", ground, Vec3(0, 0.8, 0), Vec3(0), sensor, Vec3(0), Vec3(0, 0, Pi/2));
    
    // add the sensor body to the model
    model.addBody(&sensor);
    model.addJoint(&weldJoint);

    // Apply force with both force and point in sensor body 
    ExternalForce xf5(forces, "force", "point", "", "tower", "sensor", "sensor");
    // Counter-balance with torque only in tower body 
    ExternalForce xf6(forces, "", "", "torque", "tower", "tower", "");

    model.addForce(&xf5);
    model.addForce(&xf6);
    // Everything allocated on the stack, so no need for model to own to free
    model.disownAllComponents();

    // recreate a new underlying system with corresponding state
    SimTK::State& s4 = model.initSystem();
    model.getGravityForce().disable(s4);

    // only xf4 is should be affected and set it to offset Tz+px*Fy = 2+0.1*10 = 3.
    freeCoords[3].setValue(s4, 0);
    model.setPropertiesFromState(s4);

    RungeKuttaMersonIntegrator integrator4(model.getMultibodySystem());
    integrator4.setAccuracy(accuracy);

    manager.setIntegrator(integrator4);
    manager.integrate(s4);

    // all dofs should remain constant except X-translation
    for(int i=0; i<model.getCoordinateSet().getSize(); i++){
        double val = model.getCoordinateSet()[i].getValue(s4);
        double def = model.getCoordinateSet()[i].getDefaultValue();
        if(i !=3)
            ASSERT_EQUAL(def, val, 10*accuracy);
    }
}
