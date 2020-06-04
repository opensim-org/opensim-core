/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testActuators.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Soha Pouya                                           *
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
//========================  Actuators Tested ==================================
//
//  Tests Include:
//    1. testTorqueActuator()
//    2. testBodyActuator()
//    3. testClutchedPathSpring()
//    4. testMcKibbenActuator()
//    5. testActuatorsCombination()
//    6. testActivationCoordinateActuator()
//
//     Add tests here as Actuators are added to OpenSim
//
//=============================================================================
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Analyses/osimAnalyses.h>
#include <OpenSim/Simulation/Model/ControllerSet.h>

using namespace OpenSim;
using namespace std;

//==============================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-4;
const static double duration = 1.0;
const static SimTK::Vec3 gravity_vec = SimTK::Vec3(0, -9.8065, 0);
//==============================================================================


void testTorqueActuator();
void testBodyActuator();
void testClutchedPathSpring();
void testMcKibbenActuator();
void testActuatorsCombination();
void testActivationCoordinateActuator();


int main()
{
    SimTK::Array_<std::string> failures;

    try { testTorqueActuator(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testTorqueActuator");
    }
    try { testClutchedPathSpring(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testClutchedPathSpring");
    }
    try { testMcKibbenActuator(); }
    catch (const std::exception& e) {
        cout << e.what() << endl; failures.push_back("testMcKibbenActuator");
    }
    try { testBodyActuator(); }
    catch (const std::exception& e) {
        cout << e.what() << endl; failures.push_back("testBodyActuator");
    }
    try { testActuatorsCombination(); }
    catch (const std::exception& e) {
        cout << e.what() << endl; failures.push_back("testActuatorsCombination");
    }
    try { testActivationCoordinateActuator(); }
    catch (const std::exception& e) {
        cout << e.what() << endl; failures.push_back("testActivationCoordinateActuator");
    }
    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done, testActuators passed." << endl;
}


//==============================================================================
// Test Cases
//==============================================================================
void testTorqueActuator()
{
    using namespace SimTK;
    // start timing
    std::clock_t startTime = std::clock();

    // Setup OpenSim model
    Model *model = new Model;

    // turn off gravity
    model->setGravity(Vec3(0));

    //OpenSim bodies
    const Ground& ground = model->getGround();

    //Cylindrical bodies
    double r = 0.25, h = 1.0;
    double m1 = 1.0, m2 = 2.0;
    Inertia j1 = m1*Inertia::cylinderAlongY(r, h);
    Inertia j2 = m2*Inertia::cylinderAlongY(r, h);

    //OpenSim bodies
    OpenSim::Body* bodyA 
        = new OpenSim::Body("A", m1, Vec3(0), j1);
    
    OpenSim::Body* bodyB 
        = new OpenSim::Body("B", m2, Vec3(0), j2);

    // connect bodyA to ground with 6dofs
    FreeJoint* base = 
        new FreeJoint("base", ground, Vec3(0), Vec3(0), *bodyA, Vec3(0), Vec3(0));

    model->addBody(bodyA);
    model->addJoint(base);

    // connect bodyA to bodyB by a Ball joint
    BallJoint* bInA = new BallJoint("bInA", *bodyA, Vec3(0,-h/2, 0), Vec3(0), 
                           *bodyB, Vec3(0, h/2, 0), Vec3(0));

    model->addBody(bodyB);
    model->addJoint(bInA);

    // specify magnitude and direction of applied torque
    double torqueMag = 2.1234567890;
    Vec3 torqueAxis(1/sqrt(2.0), 0, 1/sqrt(2.0));
    Vec3 torqueInG = torqueMag*torqueAxis;

    model->print("testTorqueActuator.osim");

    State state = model->initSystem();

    model->getMultibodySystem().realize(state, Stage::Dynamics);
    Vector_<SpatialVec>& bodyForces = 
        model->getMultibodySystem().updRigidBodyForces(state, Stage::Dynamics);
    bodyForces.dump("Body Forces before applying torque");
    model->getMatterSubsystem().addInBodyTorque(state, bodyA->getMobilizedBodyIndex(),
        torqueMag*torqueAxis, bodyForces);
    model->getMatterSubsystem().addInBodyTorque(state, bodyB->getMobilizedBodyIndex(),
        -torqueMag*torqueAxis, bodyForces);
    bodyForces.dump("Body Forces after applying torque to bodyA and bodyB");

    model->getMultibodySystem().realize(state, Stage::Acceleration);
    const Vector& udotBody = state.getUDot();
    udotBody.dump("Accelerations due to body forces");

    // clear body forces
    bodyForces *= 0;

    // update mobility forces
    Vector& mobilityForces = model->getMultibodySystem()
        .updMobilityForces(state, Stage::Dynamics);

    // Apply torques as mobility forces of the ball joint
    for(int i=0; i<3; ++i){
        mobilityForces[6+i] = torqueInG[i]; 
    }

    model->getMultibodySystem().realize(state, Stage::Acceleration);
    const Vector& udotMobility = state.getUDot();
    udotMobility.dump("Accelerations due to mobility forces");

    // First make sure that accelerations are not zero accidentally
    ASSERT(udotMobility.norm() != 0.0 || udotBody.norm() != 0.0);
    // Then check if they are equal
    for(int i=0; i<udotMobility.size(); ++i){
        ASSERT_EQUAL(udotMobility[i], udotBody[i], 1.0e-12);
    }

    // clear the mobility forces
    mobilityForces = 0;

    //Now add the actuator to the model and control it to generate the same
    //torque as applied directly to the multibody system (above)

    // Create and add the torque actuator to the model
    TorqueActuator* actuator =
        new TorqueActuator(*bodyA, *bodyB, torqueAxis, true);
    actuator->setName("torque");
    model->addForce(actuator);

    // Create and add a controller to control the actuator
    PrescribedController* controller =  new PrescribedController();
    controller->addActuator(*actuator);
    // Apply torque about torqueAxis
    controller->prescribeControlForActuator("torque", new Constant(torqueMag));

    model->addController(controller);

    /*
    ActuatorPowerProbe* powerProbe = new ActuatorPowerProbe(Array<string>("torque",1),false, 1); 
    powerProbe->setOperation("integrate");
    powerProbe->setInitialConditions(Vector(1, 0.0));
    */

    //model->addProbe(powerProbe);

    model->finalizeConnections();
    model->print("TestTorqueActuatorModel.osim");
    model->setUseVisualizer(false);

    // get a new system and state to reflect additions to the model
    state = model->initSystem();

    model->computeStateVariableDerivatives(state);

    const Vector &udotTorqueActuator = state.getUDot();

    // First make sure that accelerations are not zero accidentally
    ASSERT(udotMobility.norm() != 0.0 || udotTorqueActuator.norm() != 0.0);

    // Then verify that the TorqueActuator also generates the same acceleration
    // as the equivalent applied mobility force
    for(int i=0; i<udotMobility.size(); ++i){
        ASSERT_EQUAL(udotMobility[i], udotTorqueActuator[i], 1.0e-12);
    }

    // determine the initial kinetic energy of the system
    /*double iKE = */model->getMatterSubsystem().calcKineticEnergy(state);

    Manager manager(*model);
    manager.setIntegratorAccuracy(integ_accuracy);

    state.setTime(0.0);
    manager.initialize(state);

    double final_t = 1.00;
    state = manager.integrate(final_t);

    model->computeStateVariableDerivatives(state);

    /*double fKE = */model->getMatterSubsystem().calcKineticEnergy(state);

    // Change in system kinetic energy can only be attributable to actuator work
    //double actuatorWork = (powerProbe->getProbeOutputs(state))[0];
    // test that this is true
    //ASSERT_EQUAL(actuatorWork, fKE-iKE, integ_accuracy);

    // Before exiting lets see if copying the spring works
    TorqueActuator* copyOfActuator = actuator->clone();
    ASSERT(*copyOfActuator == *actuator);
    
    // Check that de/serialization works
    Model modelFromFile("TestTorqueActuatorModel.osim");
    ASSERT(modelFromFile == *model, __FILE__, __LINE__,
        "Model from file FAILED to match model in memory.");

    std::cout << " ********** Test TorqueActuator time =  ********** " << 
        1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n" << endl;
}


void testClutchedPathSpring()
{
    using namespace SimTK;

    // start timing
    std::clock_t startTime = std::clock();

    double mass = 1;
    double stiffness = 100;
    double dissipation = 0.3;
    double start_h = 0.5;
    //double ball_radius = 0.25;

    //double omega = sqrt(stiffness/mass);

    // Setup OpenSim model
    Model* model = new Model;
    model->setName("ClutchedPathSpringModel");
    model->setGravity(gravity_vec);

    //OpenSim bodies
    const Ground* ground = &model->getGround();
    
    // body that acts as the pulley that the path wraps over
    OpenSim::Body* pulleyBody =
        new OpenSim::Body("PulleyBody", mass ,Vec3(0),  mass*Inertia::brick(0.1, 0.1, 0.1));
    
    // body the path spring is connected to at both ends
    OpenSim::Body* block =
        new OpenSim::Body("block", mass ,Vec3(0),  mass*Inertia::brick(0.2, 0.1, 0.1));
    block->attachGeometry(new Brick(Vec3(0.2, 0.1, 0.1)));
    block->scale(Vec3(0.2, 0.1, 0.1), false);

    //double dh = mass*gravity_vec(1)/stiffness;
    
    WrapCylinder* pulley = new WrapCylinder();
    pulley->set_radius(0.1);
    pulley->set_length(0.05);

    // Add the wrap object to the body, which takes ownership of it
    pulleyBody->addWrapObject(pulley);

    // Add joints
    WeldJoint* weld = 
        new WeldJoint("weld", *ground, Vec3(0, 1.0, 0), Vec3(0), *pulleyBody, Vec3(0), Vec3(0));
    
    SliderJoint* slider =
        new SliderJoint("slider", *ground, Vec3(0), Vec3(0,0,Pi/2),*block, Vec3(0), Vec3(0,0,Pi/2));

    double positionRange[2] = {-10, 10};
    // Rename coordinates for a slider joint
    slider->updCoordinate().setName("block_h");
    slider->updCoordinate().setRange(positionRange);

    model->addBody(pulleyBody);
    model->addJoint(weld);

    model->addBody(block);
    model->addJoint(slider);

    ClutchedPathSpring* spring = 
        new ClutchedPathSpring("clutch_spring", stiffness, dissipation, 0.01);

    spring->updGeometryPath().appendNewPathPoint("origin", *block, Vec3(-0.1, 0.0 ,0.0));
    
    int N = 10;
    for(int i=1; i<N; ++i){
        double angle = i*Pi/N;
        double x = 0.1*cos(angle);
        double y = 0.1*sin(angle);
        spring->updGeometryPath().appendNewPathPoint("", *pulleyBody, Vec3(-x, y ,0.0));
    }

    spring->updGeometryPath().appendNewPathPoint("insertion", *block, Vec3(0.1, 0.0 ,0.0));

    // BUG in defining wrapping API requires that the Force containing the GeometryPath be
    // connected to the model before the wrap can be added
    model->addForce(spring);

    PrescribedController* controller = new PrescribedController();
    controller->addActuator(*spring);
    
    // Control greater than 1 or less than 0 should be treated as 1 and 0 respectively.
    double     timePts[4] = {0.0,  5.0, 6.0, 10.0};
    double clutchOnPts[4] = {1.5, -2.0, 0.5,  0.5};

    PiecewiseConstantFunction* controlfunc = 
        new PiecewiseConstantFunction(4, timePts, clutchOnPts);

    controller->prescribeControlForActuator("clutch_spring", controlfunc);
    model->addController(controller);

    model->finalizeConnections();
    model->print("ClutchedPathSpringModel.osim");

    //Test deserialization
    delete model;
    model = new Model("ClutchedPathSpringModel.osim");

    // Create the force reporter
    ForceReporter* reporter = new ForceReporter(model);
    model->addAnalysis(reporter);

    model->setUseVisualizer(false);
    SimTK::State& state = model->initSystem();

    CoordinateSet& coords = model->updCoordinateSet();
    coords[0].setValue(state, start_h);
    model->getMultibodySystem().realize(state, Stage::Position );

    //==========================================================================
    // Compute the force and torque at the specified times.
    Manager manager(*model);
    manager.setIntegratorAccuracy(integ_accuracy);
    manager.setWriteToStorage(true);

    state.setTime(0.0);
    manager.initialize(state);

    double final_t = 4.99999;
    state = manager.integrate(final_t);

    // tension is dynamics dependent because controls must be computed
    model->getMultibodySystem().realize(state, Stage::Dynamics);

    spring = dynamic_cast<ClutchedPathSpring*>(
                &model->updForceSet().get("clutch_spring"));
    // Now check that the force reported by spring
    double model_force = spring->getTension(state);
    double stretch0 = spring->getStretch(state);

    // the tension should be half the weight of the block
    double analytical_force = -0.5*(gravity_vec(1))*mass;

    cout << "Tension is: " << model_force << " and should be: " << analytical_force << endl;

    // error if the block does not reach equilibrium since spring is clamped
    ASSERT_EQUAL(model_force, analytical_force, 10*integ_accuracy);

    // unclamp and continue integrating
    final_t = 5.99999;
    state = manager.integrate(final_t);

    // tension is dynamics dependent because controls must be computed
    model->getMultibodySystem().realize(state, Stage::Dynamics);

    // tension should go to zero quickly
    model_force = spring->getTension(state);

    cout << "Tension is: " << model_force << " and should be: 0.0" << endl;
    // is unclamped and block should be in free-fall
    ASSERT_EQUAL(model_force, 0.0, 10*integ_accuracy);

    // spring is reclamped at 7s so keep integrating
    final_t = 10.0;
    state = manager.integrate(final_t);

    // tension is dynamics dependent because controls must be computed
    model->getMultibodySystem().realize(state, Stage::Dynamics);

    // tension should build to support the block again
    model_force = spring->getTension(state);
    double stretch1 = spring->getStretch(state);

    cout << "Tension is: " << model_force << " and should be: "<< analytical_force << endl;

    // is unclamped and block should be in free-fall
    ASSERT_EQUAL(model_force, analytical_force, 10*integ_accuracy);

    cout << "Steady stretch at control = 1.0 is " << stretch0 << " m." << endl;
    cout << "Steady stretch at control = 0.5 is " << stretch1 << " m." << endl;

    ASSERT_EQUAL(2*stretch0, stretch1, 10*integ_accuracy);

    manager.getStateStorage().print("clutched_path_spring_states.sto");
    model->getControllerSet().printControlStorage("clutched_path_spring_controls.sto");

    // Save the forces
    reporter->getForceStorage().print("clutched_path_spring_forces.mot"); 

    model->disownAllComponents();

    cout << " ********** Test clutched spring time = ********** " << 
        1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n" << endl;
}


void testMcKibbenActuator()
{

    double pressure = 5 * 10e5; // 5 bars
    double num_turns = 1.5;     // 1.5 turns
    double B = 277.1 * 10e-4;  // 277.1 mm

    using namespace SimTK;
    std::clock_t startTime = std::clock();

    double mass = 1;
    double ball_radius = 10e-6;

    Model *model = new Model;
    model->setGravity(Vec3(0));

    Ground& ground = model->updGround();

    McKibbenActuator *actuator = new McKibbenActuator("mckibben", num_turns, B);

    OpenSim::Body* ball = new OpenSim::Body("ball", mass, Vec3(0), mass*SimTK::Inertia::sphere(0.1));
    ball->scale(Vec3(ball_radius), false);

    actuator->addNewPathPoint("mck_ground", ground, Vec3(0));
    actuator->addNewPathPoint("mck_ball", *ball, Vec3(ball_radius));

    Vec3 locationInParent(0, ball_radius, 0), orientationInParent(0), locationInBody(0), orientationInBody(0);
    SliderJoint *ballToGround = new SliderJoint("ballToGround", ground, locationInParent, orientationInParent, *ball, locationInBody, orientationInBody);

    ballToGround->updCoordinate().setName("ball_d");
    ballToGround->updCoordinate().setPrescribedFunction(LinearFunction(20 * 10e-4, 0.5 * 264.1 * 10e-4));
    ballToGround->updCoordinate().set_prescribed(true);

    model->addBody(ball);
    model->addJoint(ballToGround);
    model->addForce(actuator);

    PrescribedController* controller = new PrescribedController();
    controller->addActuator(*actuator);
    controller->prescribeControlForActuator("mckibben", new Constant(pressure));

    model->addController(controller);

    ForceReporter* reporter = new ForceReporter(model);
    model->addAnalysis(reporter);

    SimTK::State& si = model->initSystem();

    model->getMultibodySystem().realize(si, Stage::Position);

    double final_t = 10.0;
    double nsteps = 10;
    double dt = final_t / nsteps;

    Manager manager(*model);
    manager.setIntegratorAccuracy(1e-7);
    si.setTime(0.0);
    manager.initialize(si);

    for (int i = 1; i <= nsteps; i++) {
        si = manager.integrate(dt*i);
        model->getMultibodySystem().realize(si, Stage::Velocity);
        Vec3 pos = ball->findStationLocationInGround(si, Vec3(0));

        double applied = actuator->computeActuation(si);;

        double theoretical = (pressure / (4 * pow(num_turns, 2) * SimTK::Pi)) * (3 * pow(pos(0), 2) - pow(B, 2));

        ASSERT_EQUAL(applied, theoretical, 10.0);
    }


    std::cout << " ******** Test McKibbenActuator time = ********" <<
        1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC << "ms\n" << endl;
}

//====================================================================================
//                              TEST BODY ACTUATOR
//====================================================================================
/**
* This test verifies the use of BodyActuator for applying spatial forces to a selected
* body. It checks if using a BodyActuator generates equivalent acceleration compared 
* to when applying the forces via mobilityForce.
*
* @author Soha Pouya
*/
void testBodyActuator()
{
    using namespace SimTK;
    // start timing
    std::clock_t startTime = std::clock();

    // Setup OpenSim model
    Model *model = new Model;

    // turn off gravity
    model->setGravity(Vec3(0));

    //OpenSim body 1: Ground
    const Ground& ground = model->getGround();

    // OpenSim body 2: A Block
    // Geometrical/Inertial properties for the block
    double blockMass = 1.0, blockSideLength = 1;
    Vec3 blockMassCenter(0);
    Inertia blockInertia = blockMass*Inertia::brick(blockSideLength/2,
        blockSideLength/2, blockSideLength/2); // for the halves see doxygen for brick 

    OpenSim::Body *block = new OpenSim::Body("block", blockMass, 
                                             blockMassCenter, blockInertia);

    // Add display geometry to the block to visualize in the GUI
    block->attachGeometry(new Brick(Vec3(blockSideLength/2,
                                     blockSideLength/2, 
                                     blockSideLength/2)));

    Vec3 locationInParent(0, blockSideLength / 2, 0), orientationInParent(0), 
        locationInBody(0), orientationInBody(0);
    FreeJoint *blockToGroundFree = new FreeJoint("blockToGroundBall", 
        ground, locationInParent, orientationInParent, 
        *block, locationInBody, orientationInBody);
    
    model->addBody(block);
    model->addJoint(blockToGroundFree);
    
    // specify magnitude and direction of applied force and torque
    double forceMag = 1.0;
    Vec3 forceAxis(1, 1, 1);
    Vec3 forceInG = forceMag * forceAxis;

    double torqueMag = 1.0;
    Vec3 torqueAxis(1, 1, 1);
    Vec3 torqueInG = torqueMag*torqueAxis;

    // ---------------------------------------------------------------------------
    // Use MobilityForces to Apply the given Torques and Forces to the body
    // ---------------------------------------------------------------------------
    State& state = model->initSystem();

    model->getMultibodySystem().realize(state, Stage::Dynamics);
    Vector_<SpatialVec>& bodyForces =
        model->getMultibodySystem().updRigidBodyForces(state, Stage::Dynamics);
    bodyForces.dump("Body Forces before applying 6D spatial force:");

    model->getMatterSubsystem().addInBodyTorque(state, block->getMobilizedBodyIndex(),
        torqueInG, bodyForces);
    model->getMatterSubsystem().addInStationForce(state, block->getMobilizedBodyIndex(),
        Vec3(0), forceInG, bodyForces);

    bodyForces.dump("Body Forces after applying 6D spatial force to the block");

    model->getMultibodySystem().realize(state, Stage::Acceleration);
    Vector udotBody = state.getUDot();
    udotBody.dump("Accelerations due to body forces");

    // clear body forces
    bodyForces *= 0;

    // update mobility forces
    Vector& mobilityForces = model->getMultibodySystem()
        .updMobilityForces(state, Stage::Dynamics);

    // Apply torques as mobility forces of the ball joint
    for (int i = 0; i<3; ++i){
        mobilityForces[i] = torqueInG[i];
        mobilityForces[i+3] = forceInG[i];
    }
    mobilityForces.dump("Mobility Forces after applying 6D spatial force to the block");


    model->getMultibodySystem().realize(state, Stage::Acceleration);
    Vector udotMobility = state.getUDot();
    udotMobility.dump("Accelerations due to mobility forces");

    // First make sure that accelerations are not zero accidentally
    ASSERT(udotMobility.norm() != 0.0 || udotBody.norm() != 0.0);
    // Then check if they are equal
    for (int i = 0; i<udotMobility.size(); ++i){
        ASSERT_EQUAL(udotMobility[i], udotBody[i], SimTK::Eps);
    }

    // clear the mobility forces
    mobilityForces = 0;

    // ---------------------------------------------------------------------------
    // Use a BodyActuator to Apply the same given Torques and Forces to the body
    // ---------------------------------------------------------------------------

    // Create and add the body actuator to the model
    BodyActuator* actuator = new BodyActuator(*block);
    actuator->setName("BodyAct");
    model->addForce(actuator);
    model->setUseVisualizer(false);

    // get a new system and state to reflect additions to the model
    State& state1 = model->initSystem();

    model->print("TestBodyActuatorModel.osim");

    // -------------- Provide control signals for bodyActuator ----------
    // Get the default control vector of the model
    Vector modelControls = model->getDefaultControls();
    
    // Specify a vector of control signals to include desired torques and forces
    Vector fixedControls(6);
    for (int i = 0; i < 3; i++){
        fixedControls(i) = torqueInG(i);
        fixedControls(i + 3) = forceInG(i);
    }
    fixedControls.dump("Spatial forces applied by body Actuator:");

    // Add control values and set their values
    actuator->addInControls(fixedControls, modelControls);
    model->setDefaultControls(modelControls);

    // ------------------- Compute Acc and Compare -------------
    // Compute the acc due to spatial forces applied by body actuator
    model->computeStateVariableDerivatives(state1);

    Vector udotBodyActuator = state1.getUDot();
    udotBodyActuator.dump("Accelerations due to body actuator");

    // First make sure that accelerations are not zero accidentally
    ASSERT(udotMobility.norm() != 0.0 || udotBodyActuator.norm() != 0.0);
    // Then verify that the BodyActuator also generates the same acceleration
    // as the equivalent applied mobility force
    for (int i = 0; i<udotBodyActuator.size(); ++i){
        ASSERT_EQUAL(udotMobility[i], udotBodyActuator[i], SimTK::Eps);
    }

    // -------------- Setup manager -------------------
    Manager manager(*model);
    manager.setIntegratorAccuracy(integ_accuracy);

    state1.setTime(0.0);
    manager.initialize(state1);
    double final_t = 1.00;
    state1 = manager.integrate(final_t);

    // ----------------- Test Copying the model -------------------
    // Before exiting lets see if copying the actuator works
    BodyActuator* copyOfActuator = actuator->clone();
    ASSERT(*copyOfActuator == *actuator);

    // Check that de/serialization works
    Model modelFromFile("TestBodyActuatorModel.osim");
    ASSERT(modelFromFile == *model, __FILE__, __LINE__,
        "Model from file FAILED to match model in memory.");

    std::cout << " ********** Test BodyActuator time = ********** " <<
        1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC << "ms\n" << endl;
}

//==================================================================================
//                         TEST ACTUATORS COMBINATION
//==================================================================================
/**
* This test verifies if using a BodyActuator generates equivalent result in the body
* acceleration compared to when using a combination of PointActuaor, TorqueActuator
* and BodyActuator. 
* It therefore also verifies model consistency when user defines and uses a 
* combination of these 3 actuators. 
* 
* @author Soha Pouya
*/
void testActuatorsCombination()
{
    using namespace SimTK;
    // start timing
    std::clock_t startTime = std::clock();

    // Setup OpenSim model
    Model *model = new Model;

    // turn off gravity
    model->setGravity(Vec3(0));

    // OpenSim bodies: 1) The ground
    const Ground& ground = model->getGround();
    //ground.addDisplayGeometry("block.vtp");

    // OpenSim bodies: 2) A Block
    // Geometrical/Inertial properties for the block
    double blockMass = 1.0, blockSideLength = 1.0;
    Vec3 blockMassCenter(0);
    // Brick Inertia: for the halves see doxygen  
    Inertia blockInertia = blockMass*Inertia::brick(blockSideLength/2, 
                                    blockSideLength/2, blockSideLength/2);
    std::cout << "blockInertia: " << blockInertia << std::endl;

    OpenSim::Body *block = new OpenSim::Body("block", blockMass, 
                                    blockMassCenter, blockInertia);

    // Add display geometry to the block to visualize in the GUI
    block->attachGeometry(new Brick(Vec3(blockSideLength/2, 
                                     blockSideLength/2, 
                                     blockSideLength/2)));

    // Make a FreeJoint from block to ground
    Vec3 locationInParent(0, blockSideLength/2, 0), orientationInParent(0), //locationInParent(0, blockSideLength, 0)
         locationInBody(0), orientationInBody(0);
    FreeJoint *blockToGroundFree = new FreeJoint("blockToGroundFreeJoint", 
                ground, locationInParent, orientationInParent, 
                *block, locationInBody, orientationInBody);

    // Add the body and joint to the model
    model->addBody(block);
    model->addJoint(blockToGroundFree);

    // specify magnitude and direction of desired force and torque vectors to apply
    double forceMag = 1.0;
    Vec3 forceAxis(1, 1, 1);
    SimTK::UnitVec3 forceUnitAxis(forceAxis); // to normalize
    Vec3 forceInG = forceMag * forceUnitAxis;

    double torqueMag = 1.0;
    Vec3 torqueAxis(1, 2, 1);
    SimTK::UnitVec3 torqueUnitAxis(torqueAxis); // to normalize
    Vec3 torqueInG = torqueMag*torqueUnitAxis;

    // needed to be called here once to build controller for body actuator
    /*State& state = */model->initSystem();
    
    // ---------------------------------------------------------------------------
    // Add a set of PointActuator, TorqueActuator and BodyActuator to the model
    // ---------------------------------------------------------------------------
    // Create and add a body actuator to the model
    BodyActuator* bodyActuator1 = new BodyActuator(*block);
    bodyActuator1->setName("BodyAct1");
    bodyActuator1->set_point(Vec3(0, blockSideLength/2, 0));
    model->addForce(bodyActuator1);
    
    // Create and add a torque actuator to the model
    TorqueActuator* torqueActuator =
        new TorqueActuator(*block, ground, torqueUnitAxis, true);
    torqueActuator->setName("torqueAct");
    model->addForce(torqueActuator);

    // Create and add a point actuator to the model
    PointActuator* pointActuator =
        new PointActuator("block");
    pointActuator->setName("pointAct");
    pointActuator->set_direction(forceUnitAxis);
    pointActuator->set_point(Vec3(0, blockSideLength/2,0));
    model->addForce(pointActuator);
    model->finalizeConnections(); // Needed so sockets have correct path
    // ------ build the model -----
    model->print("TestActuatorCombinationModel.osim");
    model->setUseVisualizer(false);

    // get a new system and state to reflect additions to the model
    State& state1 = model->initSystem();

    // ------------------- Provide control signals for bodyActuator ----------------
    // Get the default control vector of the model
    Vector modelControls = model->getDefaultControls();

    // Specify a vector of control signals for desired torques and forces
    Vector bodyActuator1Controls(6,0.0); 
    for (int i=0; i<3; i++) bodyActuator1Controls(i) = torqueInG(i); // torque in 3 axes
    for (int i=0; i<3; i++) bodyActuator1Controls(i+3) = forceInG(i); // force along 3 axes
    
    
    bodyActuator1Controls.dump("Spatial forces applied by first Body Actuator:");

    // Add control values and set their values
    bodyActuator1->addInControls(bodyActuator1Controls, modelControls);
    model->setDefaultControls(modelControls);

    // ---------------- Provide control signals for torqueActuator -----------------
    Vector torqueActuatorControls(1); // input to addInControl should be a Vector
    torqueActuatorControls(0) = torqueMag; // axis already defined when initializing 

    Vector torqueActuatorVector(3); // to print out the 3D vector of applied torque
    for (int i = 0; i < 3; i++){
        torqueActuatorVector(i) = torqueInG(i);
    }
    torqueActuatorVector.dump("Torques applied by the Torque Actuator:");

    // Add control values and set their values
    torqueActuator->addInControls(torqueActuatorControls, modelControls);
    model->setDefaultControls(modelControls);

    // ------------------ Provide control signals for pointActuator ----------------
    Vector pointActuatorControls(1); // input to addInControl should be a Vector
    pointActuatorControls(0) = forceMag; // axis already defined when initializing

    Vector pointActuatorVector(3); // to print out the whole force vector
    for (int i = 0; i < 3; i++){
        pointActuatorVector(i) = forceInG(i);
    }
    pointActuatorVector.dump("Forces applied by the point Actuator:");

    // Add control values and set their values
    pointActuator->addInControls(pointActuatorControls, modelControls);
    model->setDefaultControls(modelControls);

    
    // ----------------------- Compute the acc to Compare later --------------------
    // compare the acc due to forces/torques applied by all actuator
    model->computeStateVariableDerivatives(state1);

    Vector udotActuatorsCombination = state1.getUDot();
    udotActuatorsCombination.dump("Accelerations due to all 3 actuators");


    // -----------------------------------------------------------------------------
    // Add a BodyActuator to enclose all of the above spatial forces in one Actuator 
    // -----------------------------------------------------------------------------
    // Create and add a body actuator to the model
    BodyActuator* bodyActuator_sum = new BodyActuator(*block);
    bodyActuator_sum->setName("BodyAct_Sum");
    model->addForce(bodyActuator_sum);
    bodyActuator_sum->set_point(Vec3(0, blockSideLength / 2, 0));


    State& state2 = model->initSystem();
    model->setUseVisualizer(true);

    // Get the default control vector of the model
    Vector modelControls_2 = model->getDefaultControls();

    // Specify a vector of control signals for desired torques and forces
    Vector bodyActuatorSum_Controls(6,0.0);

    // make the torque component as the sum of body, torque and point actuators used 
    // in previous test case
    for (int i = 0; i < 3; i++){
        bodyActuatorSum_Controls(i)   = 2*torqueInG(i);
        bodyActuatorSum_Controls(i+3) = 2*forceInG(i);
    }   

    bodyActuatorSum_Controls.dump("Spatial forces applied by 2nd Body Actuator:");
    std::cout <<"(encloses sum of the above spatial forces in one BodyActuator)"<< std::endl;

    // Add control values and set their values
    bodyActuator_sum->addInControls(bodyActuatorSum_Controls, modelControls_2);
    model->setDefaultControls(modelControls_2);

    // --------------------------- Compute Acc and Compare -------------------------
    // now compare the acc due to forces/torques applied by this body actuator
    model->computeStateVariableDerivatives(state2);

    Vector udotOnlyBodyActuator = state2.getUDot();
    udotOnlyBodyActuator.dump("Accelerations due to only-one body actuator");

    // Verify that the bodyActuator_sum also generates the same acceleration
    // as the equivalent applied by 3 Actuators in previous test case
    // Also make sure that accelerations are not zero accidentally
    ASSERT(udotOnlyBodyActuator.norm() != 0.0 || udotActuatorsCombination.norm() != 0.0);
    for (int i = 0; i<udotActuatorsCombination.size(); ++i){
        ASSERT_EQUAL(udotOnlyBodyActuator[i], udotActuatorsCombination[i], 1.0e-12);
    }
    
    // ------------------------ Setup manager -----------------------
    Manager manager(*model);
    manager.setIntegratorAccuracy(integ_accuracy);

    state2.setTime(0.0);
    manager.initialize(state2);
    double final_t = 1.00;
    state2 = manager.integrate(final_t);


    std::cout << " ********** Test Actuator Combination time = ********** " <<
        1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC << "ms\n" << endl;
}

// Test de/serialization and numerical integration of
// ActivationCoordinateActautor.
void testActivationCoordinateActuator() {
    Model model;
    auto* body = new Body("body", 1, SimTK::Vec3(0), SimTK::Inertia(1.0));
    auto* joint = new PinJoint("joint", *body, model.getGround());
    joint->updCoordinate().setName("coord");
    model.addBody(body);
    model.addJoint(joint);
    auto* aca = new ActivationCoordinateActuator("coord");
    aca->setName("aca");
    const double a0 = 0.7;
    const double tau = 0.20;
    aca->set_default_activation(a0);
    aca->set_activation_time_constant(tau);
    model.addForce(aca);
    model.finalizeFromProperties();
    model.finalizeConnections();
    model.print("Model_ActivationCoordinateActuator.osim");

    Model modelDeserialized("Model_ActivationCoordinateActuator.osim");
    ASSERT(model == modelDeserialized);

    auto* controller = new PrescribedController();
    controller->addActuator(*aca);
    const double x = 0.15;
    controller->prescribeControlForActuator("aca", new Constant(x));
    model.addController(controller);

    auto state = model.initSystem();

    Manager manager(model);
    manager.initialize(state);
    const double tf = 0.10;
    state = manager.integrate(tf);

    const double expectedFinalActivation =
            (a0 - x) * exp(-tf / tau) + x;
    const double foundFinalActivation =
            aca->getStateVariableValue(state, "activation");
    ASSERT_EQUAL(expectedFinalActivation, foundFinalActivation, 1e-4);
}
