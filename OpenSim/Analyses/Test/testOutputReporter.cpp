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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Analyses/OutputReporter.h>

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
This function completes a controlled activation, controlled stretch simulation
of a muscle. After the simulation has completed, the results can be
tested in a number of different ways to ensure that the muscle model is
functioning

@param muscle   a muscle model that satisfies the Muscle interface
@param startX   the starting position of the muscle anchor. I have no idea
why this value is included.
@param act0     the initial i of the muscle
@param motion   the forced stretch of the simulation
@param control  the activation control signal that is applied to the muscle
@param accuracy the desired accuracy of the integrated solution
@param printResults print the osim model associated with this test.
*/
void simulateMuscle(const Muscle &muscle,
                    double startX,
                    double act0,
                    const Function *motion,
                    const Function *control,
                    double integrationAccuracy,
                    bool printResults);


int main()
{
    SimTK::Array_<std::string> failures;

    try {
    Millard2012EquilibriumMuscle muscle("muscle",
                    MaxIsometricForce0,
                    OptimalFiberLength0,
                    TendonSlackLength0,
                    PennationAngle0);

    muscle.setActivationTimeConstant(Activation0);
    muscle.setDeactivationTimeConstant(Deactivation0);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle( muscle,
                    x0,
                    act0,
                    &motion,
                    &control,
                    IntegrationAccuracy,
                    true);
    }

    catch (const Exception& e) {
        e.print(cout);
        failures.push_back("testOutputReporter");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }


    cout << "testOutputReporter Done" << endl;
    return 0;
}

/*==============================================================================
Main test driver to be used on any muscle model 
================================================================================
*/
void simulateMuscle(
    const Muscle &aMuscModel,
    double startX,
    double act0,
        const Function *motion,  // prescribe motion of free end of muscle
        const Function *control, // prescribed excitation signal to the muscle
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

    double optimalFiberLength = aMuscModel.getOptimalFiberLength();
    double pennationAngle     = aMuscModel.getPennationAngleAtOptimalFiberLength();
    double tendonSlackLength  = aMuscModel.getTendonSlackLength();

    // Use a copy of the muscle model passed in to add path points later
    PathActuator *aMuscle = aMuscModel.clone();

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

    if (motion != NULL){
        sliderCoord.setPrescribedFunction(*motion);
        sliderCoord.setDefaultIsPrescribed(true);
    }
    // add ball to model
    model.addBody(ball);
    model.addJoint(slider);


    //==========================================================================
    // 1. SIMULATION SETUP: Add the muscle
    //==========================================================================
    //Attach the muscle
    /*const string &actuatorType = */aMuscle->getConcreteClassName();
    aMuscle->setName("muscle");
    aMuscle->addNewPathPoint("muscle-box", ground, Vec3(anchorWidth / 2, 0, 0));
    aMuscle->addNewPathPoint("muscle-ball", *ball, Vec3(-ballRadius, 0, 0));

    ActivationFiberLengthMuscle_Deprecated *aflMuscle
        = dynamic_cast<ActivationFiberLengthMuscle_Deprecated *>(aMuscle);
    if (aflMuscle){
        // Define the default states for the muscle that has 
        //activation and fiber-length states
        aflMuscle->setDefaultActivation(act0);
        aflMuscle->setDefaultFiberLength(aflMuscle->getOptimalFiberLength());
    }
    else{
        ActivationFiberLengthMuscle *aflMuscle2
            = dynamic_cast<ActivationFiberLengthMuscle *>(aMuscle);
        if (aflMuscle2){
            // Define the default states for the muscle 
            //that has activation and fiber-length states
            aflMuscle2->setDefaultActivation(act0);
            aflMuscle2->setDefaultFiberLength(aflMuscle2
                ->getOptimalFiberLength());
        }
    }

    model.addForce(aMuscle);

    // Create a prescribed controller that simply 
    //applies controls as function of time
    PrescribedController * muscleController = new PrescribedController();
    if (control != NULL){
        muscleController->setActuators(model.updActuators());
        // Set the individual muscle control functions 
        //for the prescribed muscle controller
        muscleController->prescribeControlForActuator("muscle", control->clone());

        // Add the control set controller to the model
        model.addController(muscleController);
    }

    //==========================================================================
    // 2. OUTPUTREPORTER SETUP: create and add the OutputReporter
    //==========================================================================
    OutputReporter* outputReporter = new OutputReporter(&model);
    outputReporter->append_output_paths("kinetic_energy");
    outputReporter->append_output_paths("slider/tx/value");
    outputReporter->append_output_paths("ball/linear_velocity");
    outputReporter->append_output_paths("ball/angular_acceleration");
    outputReporter->append_output_paths("ball/acceleration");
    outputReporter->append_output_paths("slider/reaction_on_child");

    // should print a warning
    outputReporter->append_output_paths("ball/transform");

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
    modelCoordinateSet[0].setValue(state, startX, true);

    //==========================================================================
    // 4. SIMULATION Integration
    //==========================================================================
    // Create the integrator
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    integrator.setAccuracy(integrationAccuracy);

    // Create the manager
    Manager manager(model, integrator);

    // Integrate from initial time to final time
    state.setTime(initialTime);
    cout << "\nIntegrating from " << initialTime << " to " << finalTime << endl;

    // Start timing the simulation
    const clock_t start = clock();
    // simulate
    manager.integrate(state, finalTime);

    // how long did it take?
    double comp_time = (double)(clock() - start) / CLOCKS_PER_SEC;

    //==========================================================================
    // 4. Print the results
    //==========================================================================
    model.updAnalysisSet().printResults("testOutputReporter");
}
