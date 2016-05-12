/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testControllers.cpp                        *
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

//==========================================================================================================
//  testControllers builds OpenSim models using the OpenSim API and verifies that controllers
//  behave as described.
//
//  Tests Include:
//      1. Test a control set controller on a block with an ideal actuator
//      2. Test a corrective controller on a block with an ideal actuator
//      
//     Add tests here as new controller types are added to OpenSim
//
//==========================================================================================================
#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testControlSetControllerOnBlock();
void testPrescribedControllerOnBlock(bool disabled);
void testCorrectionControllerOnBlock();
void testPrescribedControllerFromFile(const std::string& modelFile,
                                      const std::string& actuatorsFile,
                                      const std::string& controlsFile);

int main()
{
    try {
        cout << "Testing ControlSetController" << endl; 
        testControlSetControllerOnBlock();
        cout << "Testing PrescribedController" << endl; 
        testPrescribedControllerOnBlock(false);
        testPrescribedControllerOnBlock(true);
        cout << "Testing CorrectionController" << endl; 
        testCorrectionControllerOnBlock();
        cout << "Testing PrescribedController from File" << endl;
        testPrescribedControllerFromFile("arm26.osim", "arm26_Reserve_Actuators.xml",
                                         "arm26_controls.xml");
    }   
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

//==========================================================================================================
void testControlSetControllerOnBlock()
{
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

    //Create a free joint with 6 degrees-of-freedom
    SimTK::Vec3 noRotation(0);
    SliderJoint blockToGround("slider",ground, blockInGround, noRotation, block, blockMassCenter, noRotation);
    
    // Create coordinates (degrees-of-freedom) between the ground and block
    CoordinateSet& jointCoordinateSet = blockToGround.upd_CoordinateSet();
    double posRange[2] = {-1, 1};
    jointCoordinateSet[0].setName("xTranslation");
    jointCoordinateSet[0].setRange(posRange);

    // Add the block and joint to the model
    osimModel.addBody(&block);
    osimModel.addJoint(&blockToGround);

    // Define a single coordinate actuator.
    CoordinateActuator actuator(jointCoordinateSet[0].getName());
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
    ControlSetController actuatorController;
    // Make a copy and set it on the ControlSetController as it takes ownership of the 
    // ControlSet passed in
    actuatorController.setControlSet((ControlSet*)Object::SafeCopy(&actuatorControls));

    // add the controller to the model
    osimModel.addController(&actuatorController);

    // Initialize the system and get the state representing the state system
    SimTK::State& si = osimModel.initSystem();

    // Specify zero slider joint kinematic states
    CoordinateSet &coordinates = osimModel.updCoordinateSet();
    coordinates[0].setValue(si, 0.0);    // x translation
    coordinates[0].setSpeedValue(si, 0.0);           // x speed

    // Create the integrator and manager for the simulation.
    double accuracy = 1.0e-3;
    SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
    integrator.setAccuracy(accuracy);
    Manager manager(osimModel, integrator);

    // Integrate from initial time to final time
    manager.setInitialTime(initialTime);
    manager.setFinalTime(finalTime);
    std::cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
    manager.integrate(si);

    si.getQ().dump("Final position:");
    double x_err = fabs(coordinates[0].getValue(si) - 0.5*(controlForce[0]/blockMass)*finalTime*finalTime);
    ASSERT(x_err <= accuracy, __FILE__, __LINE__, "ControlSetControllerOnBlock failed to produce the expected motion.");

    // Save the simulation results
    Storage states(manager.getStateStorage());
    states.print("block_push.sto");

    osimModel.disownAllComponents();
}// end of testControlSetControllerOnBlock()


//==========================================================================================================
void testPrescribedControllerOnBlock(bool disabled)
{
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

    //Create a free joint with 6 degrees-of-freedom
    SimTK::Vec3 noRotation(0);
    SliderJoint blockToGround("slider",ground, blockInGround, noRotation, block, blockMassCenter, noRotation);
    // Create 6 coordinates (degrees-of-freedom) between the ground and block
    CoordinateSet& jointCoordinateSet = blockToGround.upd_CoordinateSet();
    double posRange[2] = {-1, 1};
    jointCoordinateSet[0].setRange(posRange);

    // Add the block body to the model
    osimModel.addBody(&block);
    osimModel.addJoint(&blockToGround);

    // Define a single coordinate actuator.
    CoordinateActuator actuator(jointCoordinateSet[0].getName());
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
    actuatorController.prescribeControlForActuator(0, new Constant(controlForce));
    actuatorController.setDisabled(disabled);

    // add the controller to the model
    osimModel.addController(&actuatorController);
    
    osimModel.print("blockWithPrescribedController.osim");
    Model modelfileFromFile("blockWithPrescribedController.osim");

    // Verify that serialization and then deserialization is correct
    ASSERT(osimModel == modelfileFromFile);

    // Initialize the system and get the state representing the state system
    SimTK::State& si = osimModel.initSystem();

    // Specify zero slider joint kinematic states
    CoordinateSet &coordinates = osimModel.updCoordinateSet();
    coordinates[0].setValue(si, 0.0);    // x translation
    coordinates[0].setSpeedValue(si, 0.0);           // x speed

    // Create the integrator and manager for the simulation.
    double accuracy = 1.0e-3;
    SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
    integrator.setAccuracy(accuracy);
    Manager manager(osimModel, integrator);

    // Integrate from initial time to final time
    manager.setInitialTime(initialTime);
    manager.setFinalTime(finalTime);
    std::cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
    manager.integrate(si);

    si.getQ().dump("Final position:");

    double expected = disabled ? 0 : 0.5*(controlForce/blockMass)*finalTime*finalTime;
    ASSERT_EQUAL(expected, coordinates[0].getValue(si), accuracy, __FILE__, __LINE__, "PrescribedController failed to produce the expected motion of block.");

    // Save the simulation results
    Storage states(manager.getStateStorage());
    states.print("block_push.sto");

    osimModel.disownAllComponents();
}// end of testPrescribedControllerOnBlock()


//==========================================================================================================
void testCorrectionControllerOnBlock()
{
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

    //Create a free joint with 6 degrees-of-freedom
    SimTK::Vec3 noRotation(0);
    SliderJoint blockToGround("slider",ground, blockInGround, noRotation, block, blockMassCenter, noRotation);
    // Create coordinates (degrees-of-freedom) between the ground and block
    CoordinateSet& jointCoordinateSet = blockToGround.upd_CoordinateSet();
    double posRange[2] = {-1, 1};
    jointCoordinateSet[0].setName("xTranslation");
    jointCoordinateSet[0].setRange(posRange);

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

    // Create the integrator and manager for the simulation.
    SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
    integrator.setAccuracy(1.0e-4);
    Manager manager(osimModel, integrator);

    osimModel.disownAllComponents();
}// end of testCorrectionControllerOnBlock()


void testPrescribedControllerFromFile(const std::string& modelFile,
                                      const std::string& actuatorsFile,
                                      const std::string& controlsFile)
{
    using namespace SimTK;

    double initialTime = 0.03;
    double finalTime = 1.0;

    // Create a new OpenSim model
    Model osimModel(modelFile);

    try{
        ForceSet *forceSet=new ForceSet(osimModel, actuatorsFile);
        osimModel.updForceSet().append(*forceSet);
    }
    catch(const std::exception& e){
        cout << "Actuators not loaded: " << e.what() << endl;
    }

    ControlSetController csc;
    ControlSet* cs = new ControlSet(controlsFile);
    csc.setControlSet(cs);

    // add the controller to the model
    osimModel.addController(&csc);

    // Initialize the system and get the state representing the state system
    SimTK::State& si = osimModel.initSystem();

    // Create the integrator and manager for the simulation.
    SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
    integrator.setAccuracy(1.0e-5);
    Manager manager(osimModel, integrator);

    // Integrate from initial time to final time
    manager.setInitialTime(initialTime);
    manager.setFinalTime(finalTime);
    cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
    manager.integrate(si);

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
    cout << "Number of Controllers should be 0 is ";
        cout << osimModel.getControllerSet().getSize() << endl;
    
    
    //************* Rerun with a PrescribedController ***********************/

    PrescribedController prescribed();
    // TODO
    // Convert Storage std_controls to set of Functions and map to actuators

    // add the controller to the model
    osimModel.addController(&prescribed);
    osimModel.print("TestPrescribedController.osim");

    // Initialize the system and get the state representing the state system
    SimTK::State& s2 = osimModel.initSystem();

    // Create the integrator and manager for the simulation.
    SimTK::RungeKuttaMersonIntegrator integrator2(osimModel.getMultibodySystem());
    integrator2.setAccuracy(1.0e-5);
    Manager manager2(osimModel, integrator2);

    // Integrate from initial time to final time
    manager2.setInitialTime(initialTime);
    manager2.setFinalTime(finalTime);
    cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
    manager2.integrate(s2);

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
        Array<double>(0.005, nstates), __FILE__, __LINE__,
        "testPrescribedControllerFromFile '"+modelName+"'states failed");

    CHECK_STORAGE_AGAINST_STANDARD(controls, std_controls, 
        Array<double>(0.01, nstates), __FILE__, __LINE__,
        "testPrescribedControllerFromFile '"+modelName+"'controls failed");
     
    osimModel.disownAllComponents();
}
