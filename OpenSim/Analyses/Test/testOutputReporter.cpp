/* --------------------------------------------------------------------------*
*                      OpenSim:  testOutputReporter.cpp                      *
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

#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Analyses/OutputReporter.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
using namespace std;

//==============================================================================
static const double IntegrationAccuracy         = 1e-8;
static const double SimulationTestTolerance     = 1e-6;
static const double InitializationTestTolerance = 1e-8;
static const double CorrectnessTestTolerance    = 1e-8;

static const int SimulationTest         = 0;
static const int InitializationTest     = 1;
static const int CorrectnessTest        = 2;

// MUSCLE CONSTANTS
static const double MaxIsometricForce0 = 100.0,
    OptimalFiberLength0 = 0.1,
    TendonSlackLength0 = 0.2,
    PennationAngle0 = 0.0;

static const double Activation0 = 0.01,
Deactivation0 = 0.4;

/*
Main simulation driver used to generate Outputs and report them

@param muscle   a muscle model that satisfies the Muscle interface
@param act0     the initial i of the muscle
@param accuracy the desired accuracy of the integrated solution
@param printResults print the osim model associated with this test.
*/
void simulateMuscle(
        const Muscle &muscModel,
        double integrationAccuracy,
        bool printResults)
{
    using SimTK::Vec3;

    //==========================================================================
    // 0. SIMULATION SETUP: Create the block and ground
    //==========================================================================
    // Define the initial and final simulation times
    double initialTime = 0.0;
    double finalTime = 0.5;

    //Physical properties of the model
    double ballMass = 10;
    double ballRadius = 0.05;
    double anchorWidth = 0.1;

    // Create an OpenSim model
    Model model;

    double optimalFiberLength = muscModel.getOptimalFiberLength();
    double pennationAngle     = muscModel.getPennationAngleAtOptimalFiberLength();
    double tendonSlackLength  = muscModel.getTendonSlackLength();

    // Use a copy of the muscle model passed in to add path points later
    Muscle *muscle = muscModel.clone();

    // Get a reference to the model's ground body
    Ground& ground = model.updGround();

    OpenSim::Body * ball = new OpenSim::Body("ball",
        ballMass,
        Vec3(0),
                        ballMass*SimTK::Inertia::sphere(ballRadius));

    ball->attachGeometry(new Sphere(ballRadius));
    // ball connected  to ground via a slider along X
    double xSinG = optimalFiberLength*cos(pennationAngle) + tendonSlackLength;

    SliderJoint* slider = new SliderJoint("slider",
        ground,
        Vec3(anchorWidth / 2 + xSinG, 0, 0),
        Vec3(0),
        *ball,
        Vec3(0),
                        Vec3(0));

    auto& sliderCoord = slider->updCoordinate();
    sliderCoord.setName("tx");
    sliderCoord.setDefaultValue(1.0);
    sliderCoord.setRangeMin(0);
    sliderCoord.setRangeMax(1.0);

    // add ball to model
    model.addBody(ball);
    model.addJoint(slider);

    //==========================================================================
    // 1. SIMULATION SETUP: Add the muscle
    //==========================================================================
    //Attach the muscle
    /*const string &actuatorType = */muscle->getConcreteClassName();
    muscle->setName("muscle");
    muscle->addNewPathPoint("muscle-box", ground, Vec3(anchorWidth / 2, 0, 0));
    muscle->addNewPathPoint("muscle-ball", *ball, Vec3(-ballRadius, 0, 0));

    model.addForce(muscle);

    // Create a prescribed controller that simply 
    //applies controls as function of time
    Constant control(0.5);
    PrescribedController* muscleController = new PrescribedController();

    muscleController->setActuators(model.updActuators());
    // Set the individual muscle control functions 
    //for the prescribed muscle controller
    muscleController->prescribeControlForActuator("muscle", control);

    // Add the control set controller to the model
    model.addController(muscleController);

    //==========================================================================
    // 2. OUTPUTREPORTER SETUP: create and add the OutputReporter
    //==========================================================================
    OutputReporter* outputReporter = new OutputReporter(&model);
    outputReporter->append_output_paths("/|kinetic_energy");
    outputReporter->append_output_paths("/jointset/slider/tx|value");
    outputReporter->append_output_paths("/bodyset/ball|linear_velocity");
    outputReporter->append_output_paths("/bodyset/ball|angular_acceleration");
    // Paths can also be relative to the model.
    outputReporter->append_output_paths("bodyset/ball|acceleration");
    outputReporter->append_output_paths("jointset/slider|reaction_on_child");
    outputReporter->append_output_paths("|com_position");

    // should print a warning
    outputReporter->append_output_paths("/bodyset/ball|transform");

    model.addAnalysis(outputReporter);

    model.finalizeFromProperties();
    model.printBasicInfo();

    //==========================================================================
    // 3. SIMULATION Initialization
    //==========================================================================

    // Initialize the system and get the default state    
    SimTK::State& state = model.initSystem();

    model.getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
    model.equilibrateMuscles(state);

    CoordinateSet& modelCoordinateSet = model.updCoordinateSet();

    // Define non-zero (defaults are 0) states for the free joint
    // set x-translation value
    modelCoordinateSet[0].setValue(state, 1.0, true);

    // Get Initial reported values
    model.realizeReport(state);
    double t0 = state.getTime();
    auto ke0 = model.getMultibodySystem().calcKineticEnergy(state);
    auto ang_acc0 = ball->getAngularAccelerationInGround(state);
    auto reaction0 = slider->calcReactionOnChildExpressedInGround(state);

    //==========================================================================
    // 4. SIMULATION Integration
    //==========================================================================
    // Create the manager
    Manager manager(model);
    manager.setIntegratorAccuracy(integrationAccuracy);

    // Integrate from initial time to final time
    state.setTime(initialTime);
    manager.initialize(state);

    // simulate
    state = manager.integrate(finalTime);

    //==========================================================================
    // 4. Print the results
    //==========================================================================
    model.updAnalysisSet().printResults("testOutputReporter");

    //==========================================================================
    // 5. Verify files were written with correct values
    //==========================================================================
    TimeSeriesTable tableD("testOutputReporter_Outputs.sto");
    TimeSeriesTable_<SimTK::Vec3> tableV3("testOutputReporter_OutputsVec3.sto");
    TimeSeriesTable_<SimTK::SpatialVec> tableSV("testOutputReporter_OutputsSpatialVec.sto");

    double val_t0 = tableD.getIndependentColumn()[0];
    const SimTK::Real val_ke0 = tableD.getRowAtIndex(0)[0];
    const Vec3 val_omega0 = tableV3.getRowAtIndex(0)[1];
    const SimTK::SpatialVec val_jrf0 = tableSV.getRowAtIndex(0)[1];

    CHECK_THAT(t0, Catch::Matchers::WithinAbs(val_t0, SimTK::Eps));
    CHECK_THAT(ke0, Catch::Matchers::WithinAbs(val_ke0, SimTK::Eps));
    for (int i = 0; i < 3; ++i) {
        CHECK_THAT(ang_acc0[i], 
                Catch::Matchers::WithinAbs(val_omega0[i], SimTK::Eps));
        for (int j = 0; j < 2; ++j) {
            CHECK_THAT(reaction0[j][i], 
                    Catch::Matchers::WithinAbs(val_jrf0[j][i], SimTK::Eps));
        }
    }

    double val_tf = tableD.getIndependentColumn()[tableD.getNumRows() - 1];
    const SimTK::Real val_ke = tableD.getRowAtIndex(tableD.getNumRows() - 1)[0];
    const Vec3 val_omega = tableV3.getRowAtIndex(tableV3.getNumRows() - 1)[1];
    const SimTK::SpatialVec val_jrf = 
        tableSV.getRowAtIndex(tableSV.getNumRows() - 1)[1];

    model.realizeReport(state);
    auto ke = model.getMultibodySystem().calcKineticEnergy(state);
    auto ang_acc = ball->getAngularAccelerationInGround(state);
    auto reaction = slider->calcReactionOnChildExpressedInGround(state);

    CHECK_THAT(state.getTime(), Catch::Matchers::WithinAbs(val_tf, SimTK::Eps));
    CHECK_THAT(ke, Catch::Matchers::WithinAbs(val_ke, 1e-10));
    for (int i = 0; i < 3; ++i) {
        CHECK_THAT(ang_acc[i], 
                Catch::Matchers::WithinAbs(val_omega[i], SimTK::Eps));
        for (int j = 0; j < 2; ++j) {
            CHECK_THAT(reaction[j][i], 
                    Catch::Matchers::WithinAbs(val_jrf[j][i], SimTK::Eps));
        }
    }
}

TEST_CASE("Output Reporter")
{
    Millard2012EquilibriumMuscle muscle("muscle",
        MaxIsometricForce0,
        OptimalFiberLength0,
        TendonSlackLength0,
        PennationAngle0);

    muscle.setActivationTimeConstant(Activation0);
    muscle.setDeactivationTimeConstant(Deactivation0);

    simulateMuscle( muscle,
        IntegrationAccuracy,
        true);
}
