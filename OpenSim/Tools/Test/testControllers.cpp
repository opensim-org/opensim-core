/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testControllers.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
//  testControllers builds OpenSim models using the OpenSim API and verifies 
//  that controllers behave as described.
//
//  Tests Include:
//  1. Test a ControlSetController on a block with an ideal actuator
//  2. Test a PrescribedController on a block with an ideal actuator
//  3. Test a CorrectionController tracking a block with an ideal actuator
//  4. Test a PrescribedController on the arm26 model with reserves.
//     Add tests here as new controller types are added to OpenSim
//
//=============================================================================

#include <OpenSim/OpenSim.h>

#include <catch2/catch_all.hpp>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

TEST_CASE("Test Controller interface") {

    Model model = ModelFactory::createNLinkPendulum(2);
    auto* controller = new PrescribedController();
    controller->prescribeControlForActuator("/tau0", new Constant(1.0));
    controller->prescribeControlForActuator("/tau1", new Constant(2.0));

    SECTION("No actuators connecteed") {
        model.addController(controller);
        REQUIRE_THROWS(model.finalizeConnections());
    }

    SECTION("Adding actuators individually") {
        controller->addActuator(model.getComponent<Actuator>("/tau0"));
        controller->addActuator(model.getComponent<Actuator>("/tau1"));
        model.addController(controller);
        REQUIRE_NOTHROW(model.finalizeConnections());
    }

    SECTION("Adding multiple actuators from a ComponentList") {
        controller->setActuators(model.getComponentList<Actuator>());
        model.addController(controller);
        REQUIRE_NOTHROW(model.finalizeConnections());
    }

    SECTION("Adding multiple actuators from ReferencePtrs") {
        SimTK::Array_<SimTK::ReferencePtr<const Actuator>> actuators(2);
        actuators[0] = &model.getComponent<Actuator>("/tau0");
        actuators[1] = &model.getComponent<Actuator>("/tau1");
        controller->setActuators(actuators);
        model.addController(controller);
        REQUIRE_NOTHROW(model.finalizeConnections());
    }
}

TEST_CASE("testControlSetControllerOnBlock") {
    using namespace SimTK;

    // Create a new OpenSim model
    Model osimModel;
    osimModel.setName("osimModel");

    // Get the ground body
    const Ground& ground = osimModel.getGround();

    // Create a 20 kg, 0.1 m^3 block body
    double blockMass = 20.0, blockSideLength = 0.1;
    Vec3 blockMassCenter(0), groundOrigin(0), blockInGround(0, blockSideLength/2, 0);
    Inertia blockInertia = Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

    OpenSim::Body block("block", blockMass, blockMassCenter, blockMass*blockInertia);

    // Create a slider joint with 1 degree of freedom
    SimTK::Vec3 noRotation(0);
    SliderJoint blockToGround("slider",ground, blockInGround, noRotation, block, blockMassCenter, noRotation);
    
    // Create coordinate (degree of freedom) between the ground and block
    auto& sliderCoord = blockToGround.updCoordinate();
    double posRange[2] = {-1, 1};
    sliderCoord.setName("xTranslation");
    sliderCoord.setRange(posRange);

    // Add the block and joint to the model
    osimModel.addBody(&block);
    osimModel.addJoint(&blockToGround);

    // Define a single coordinate actuator.
    CoordinateActuator actuator(sliderCoord.getName());
    actuator.setName("actuator");

    // Add the actuator to the model
    osimModel.addForce(&actuator);

    double initialTime = 0;
    double finalTime = 1.0;

    // Define the initial and final control values
    double controlForce[1] = {100};
    // Create two control signals
    ControlLinear control;
    control.setName("actuator");
    // Create a control set and add the controls to the set
    ControlSet actuatorControls;
    actuatorControls.adoptAndAppend(&control);
    actuatorControls.setMemoryOwner(false);
    actuatorControls.setControlValues(initialTime, controlForce);
    actuatorControls.setControlValues(finalTime, controlForce);
    // Create a control set controller that simply applies controls from a ControlSet
    auto* actuatorController = new ControlSetController();
    actuatorController->addActuator(
        osimModel.getComponent<Actuator>("/forceset/actuator"));
    // Make a copy and set it on the ControlSetController as it takes ownership of the 
    // ControlSet passed in
    actuatorController->setControlSet(
        (ControlSet*)Object::SafeCopy(&actuatorControls));

    // add the controller to the model
    osimModel.addController(actuatorController);

    // Initialize the system and get the state representing the state system
    SimTK::State& si = osimModel.initSystem();

    // Specify zero slider joint kinematic states
    CoordinateSet &coordinates = osimModel.updCoordinateSet();
    coordinates[0].setValue(si, 0.0);    // x translation
    coordinates[0].setSpeedValue(si, 0.0);           // x speed

    // Create the integrator and manager for the simulation.
    double accuracy = 1.0e-3;
    Manager manager(osimModel);
    manager.setIntegratorAccuracy(accuracy);

    // Integrate from initial time to final time
    si.setTime(initialTime);
    manager.initialize(si);
    log_info("");
    log_info("Integrating from {} to {}", initialTime, finalTime);
    si = manager.integrate(finalTime);

    si.getQ().dump("Final position:");
    double x_err = fabs(coordinates[0].getValue(si) - 0.5*(controlForce[0]/blockMass)*finalTime*finalTime);
    ASSERT(x_err <= accuracy, __FILE__, __LINE__, "ControlSetControllerOnBlock failed to produce the expected motion.");

    // Save the simulation results
    Storage states(manager.getStateStorage());
    states.print("block_push.sto");

    osimModel.disownAllComponents();
}


TEST_CASE("testPrescribedControllerOnBlock") {
    using namespace SimTK;

    auto enabled = GENERATE(true, false);

    // Create a new OpenSim model
    Model osimModel;
    osimModel.setName("osimModel");

    // Get the ground body
    const Ground& ground = osimModel.getGround();

    // Create a 20 kg, 0.1 m^3 block body
    double blockMass = 20.0, blockSideLength = 0.1;
    Vec3 blockMassCenter(0), groundOrigin(0), blockInGround(0, blockSideLength/2, 0);
    Inertia blockInertia = Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

    OpenSim::Body block("block", blockMass, blockMassCenter, blockMass*blockInertia);

    // Create a slider joint with 1 degree of freedom
    SimTK::Vec3 noRotation(0);
    SliderJoint blockToGround("slider",ground, blockInGround, noRotation, block, blockMassCenter, noRotation);

    // Create 1 coordinate (degree of freedom) between the ground and block
    auto& sliderCoord = blockToGround.updCoordinate();
    double posRange[2] = {-1, 1};
    sliderCoord.setRange(posRange);

    // Add the block body to the model
    osimModel.addBody(&block);
    osimModel.addJoint(&blockToGround);

    // Define a single coordinate actuator.
    CoordinateActuator actuator(sliderCoord.getName());
    actuator.setName("actuator");

    // Add the actuator to the model
    osimModel.addForce(&actuator);

    double initialTime = 0;
    double finalTime = 1.0;

    // Define the initial and final control values
    double controlForce = 100;

    // Create a prescribed controller that simply applies a function of the force
    PrescribedController actuatorController;
    actuatorController.setName("testPrescribedController");
    actuatorController.setActuators(osimModel.updActuators());
    actuatorController.prescribeControlForActuator(
        osimModel.getActuators().get(0).getAbsolutePathString(),
        new Constant(controlForce));
    actuatorController.setEnabled(enabled);

    // add the controller to the model
    osimModel.addController(&actuatorController);
    osimModel.disownAllComponents();
    
    osimModel.print("blockWithPrescribedController.osim");
    Model modelFromFile("blockWithPrescribedController.osim");

    // Verify that serialization and then deserialization is correct
    ASSERT(osimModel == modelFromFile);

    // Initialize the system and get the state representing the state system
    SimTK::State& si = osimModel.initSystem();

    // Specify zero slider joint kinematic states
    CoordinateSet &coordinates = osimModel.updCoordinateSet();
    coordinates[0].setValue(si, 0.0);    // x translation
    coordinates[0].setSpeedValue(si, 0.0);           // x speed

    // Create the integrator and manager for the simulation.
    double accuracy = 1.0e-3;
    Manager manager(osimModel);
    manager.setIntegratorAccuracy(accuracy);

    // Integrate from initial time to final time
    si.setTime(initialTime);
    manager.initialize(si);
    log_info("");
    log_info("Integrating from {} to {}", initialTime, finalTime);
    si = manager.integrate(finalTime);

    si.getQ().dump("Final position:");

    double expected = enabled ? 0.5*(controlForce/blockMass)*finalTime*finalTime : 0;
    ASSERT_EQUAL(expected, coordinates[0].getValue(si), accuracy,
        __FILE__, __LINE__, 
        "PrescribedController failed to produce the expected motion of block.");

    // Save the simulation results
    Storage states(manager.getStateStorage());
    states.print("block_push.sto");

}


TEST_CASE("testCorrectionControllerOnBlock") {
    using namespace SimTK;

    // Create a new OpenSim model
    Model osimModel;
    osimModel.setName("osimModel");

    // Get the ground body
    const Ground& ground = osimModel.getGround();

    // Create a 20 kg, 0.1 m^3 block body
    double blockMass = 20.0, blockSideLength = 0.1;
    Vec3 blockMassCenter(0), groundOrigin(0), blockInGround(0, blockSideLength/2, 0);
    Inertia blockInertia = Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

    OpenSim::Body block("block", blockMass, blockMassCenter, blockMass*blockInertia);

    // Create a slider joint with 1 degree of freedom
    SimTK::Vec3 noRotation(0);
    SliderJoint blockToGround("slider",ground, blockInGround, noRotation, block, blockMassCenter, noRotation);

    // Create coordinate (degree of freedom) between the ground and block
    auto& sliderCoord = blockToGround.updCoordinate();
    double posRange[2] = {-1, 1};
    sliderCoord.setName("xTranslation");
    sliderCoord.setRange(posRange);

    // Add the block body to the model
    osimModel.addBody(&block);
    osimModel.addJoint(&blockToGround);

    // Generate tracking data
    // Storage *desiredXTranslation = new Storage();

    CorrectionController tracker;

    // add the controller to the model
    osimModel.addController(&tracker);

    // Initialize the system and get the state representing the state system
    /*SimTK::State& si = */osimModel.initSystem();

    // Create the manager for the simulation.
    Manager manager(osimModel);
    manager.setIntegratorAccuracy(1.0e-4);

    osimModel.disownAllComponents();
}


TEST_CASE("testPrescribedControllerFromFile") {
    using namespace SimTK;

    std::string modelFile = "arm26.osim";
    std::string actuatorsFile = "arm26_Reserve_Actuators.xml";
    std::string controlsFile = "arm26_controls.xml";

    double initialTime = 0.03;
    double finalTime = 1.0;

    // Create a new OpenSim model
    Model osimModel(modelFile);

    try{
        ForceSet *forceSet=new ForceSet(actuatorsFile, true);
        osimModel.updForceSet().append(*forceSet);
    }
    catch(const std::exception& e){
        log_error("Actuators not loaded: {}", e.what());
    }

    ControlSetController csc;
    ControlSet* cs = new ControlSet(controlsFile);
    csc.setControlSet(cs);
    csc.setActuators(osimModel.updActuators());
    // add the controller to the model
    osimModel.addController(&csc);

    // Initialize the system and get the state representing the state system
    SimTK::State& si = osimModel.initSystem();

    // Create the manager for the simulation.
    Manager manager(osimModel);
    manager.setIntegratorAccuracy(1.0e-5);

    // Integrate from initial time to final time
    si.setTime(initialTime);
    manager.initialize(si);
    log_info("");
    log_info("Integrating from {} to {}", initialTime, finalTime);
    si = manager.integrate(finalTime);

    string modelName = osimModel.getName();
    // Save the simulation results
    Storage std_states(manager.getStateStorage());
    string outfileName = "std_"+modelName + "_states.sto";
    std_states.print(outfileName);

    outfileName = "std_"+modelName +"_controls.sto";
    osimModel.printControlStorage(outfileName);
    Storage std_controls(outfileName);


    // don't double delete components allocated on the stack
    osimModel.disownAllComponents();

    // remove previous controllers
    osimModel.updControllerSet().remove(0);
    log_info("Number of Controllers should be 0 is {}", 
        osimModel.getControllerSet().getSize());
    
    //************* Rerun with a PrescribedController ***********************/

    PrescribedController prescribed(outfileName, 1);
    prescribed.setActuators(osimModel.updActuators());

    // add the controller to the model
    osimModel.addController(&prescribed);
    osimModel.finalizeConnections();
    osimModel.print("TestPrescribedController.osim");

    // Initialize the system and get the state representing the state system
    SimTK::State& s2 = osimModel.initSystem();

    // Create the manager for the simulation.
    Manager manager2(osimModel);
    manager2.setIntegratorAccuracy(1.0e-5);

    // Integrate from initial time to final time
    s2.setTime(initialTime);
    manager2.initialize(s2);
    log_info("");
    log_info("Integrating from {} to {}", initialTime, finalTime);
    s2 = manager2.integrate(finalTime);

    // Save the simulation results
    Storage states(manager2.getStateStorage());
    outfileName = modelName + "_states.sto";
    states.print(outfileName);

    outfileName = modelName +"_controls.sto";
    osimModel.printControlStorage(outfileName);
    Storage controls(outfileName);

    int nstates = osimModel.getNumStateVariables();
    /*int ncontrols = */osimModel.getNumControls();

    CHECK_STORAGE_AGAINST_STANDARD(states, std_states, 
        std::vector<double>(nstates, 0.005), __FILE__, __LINE__,
        "testPrescribedControllerFromFile '"+modelName+"'states failed");

    CHECK_STORAGE_AGAINST_STANDARD(controls, std_controls, 
        std::vector<double>(nstates, 0.015), __FILE__, __LINE__,
        "testPrescribedControllerFromFile '"+modelName+"'controls failed");
     
    osimModel.disownAllComponents();
}
