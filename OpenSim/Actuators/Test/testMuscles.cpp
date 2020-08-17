/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testMuscles.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Matthew Millard                                      *
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
//  testMuscles simulates OpenSim Muscles using the OpenSim API and 
//  tests that muscles can compute equilibrium under varying conditions, 
//  including the full range of activations. Simulations are also tested
//  for energy conservation by comparing changes in system power to muscle
//  power.
//
//  Models tested include:
//      1. RigidTendonMuscle (Stateless, with user-defined f-l, f-v splines)
//      2. Thelen2003Muscle (Uses the Muscle interface)
//      3. Millard2012EquilibriumMuscle
//      4. Millard2012AccelerationMuscle
//      5. DeGrooteFregly2016Muscle
//      
//     Add more test cases to address specific problems with muscle models
//
//==============================================================================
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Analyses/osimAnalyses.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Auxiliary/auxiliaryTestMuscleFunctions.h>

using namespace OpenSim;
using namespace std;

//==============================================================================
static const double IntegrationAccuracy         = 1e-5;
static const double SimulationTestTolerance     = 1e-4;
static const double InitializationTestTolerance = 1e-6;
static const double CorrectnessTestTolerance    = 1e-6;

static const int SimulationTest         = 0;
static const int InitializationTest     = 1;
static const int CorrectnessTest        = 2;

// MUSCLE CONSTANTS
static const double MaxIsometricForce0  = 100.0, 
                    OptimalFiberLength0 = 0.1, 
                    TendonSlackLength0  = 0.2, 
                    PennationAngle0     = 0.0,
                    PennationAngle1     = SimTK::Pi/4;

static const double Activation0     = 0.01, 
                    Deactivation0   = 0.4,  
                    ShutteDelpActivation1 = 7.6,    
                    ShutteDelpActivation2 = 2.5;

/*
This function completes a controlled activation, controlled stretch simulation 
of a muscle. After the simulation has completed, the results can be 
tested in a number of different ways to ensure that the muscle model is 
functioning

@param aMuscle  a path actuator
@param startX   the starting position of the muscle anchor. I have no idea
                why this value is included.
@param act0     the initial activation of the muscle
@param motion   the forced stretch of the simulation
@param control  the activation control signal that is applied to the muscle
@param printResults print the osim model associated with this test.
*/
void simulateMuscle(const Muscle &aMuscle, 
                    double startX, 
                    double act0, 
                    const Function *motion, 
                    const Function *control, 
                    bool printResults);

//void testPathActuator();
void testRigidTendonMuscle();
void testThelen2003Muscle_Deprecated();
void testThelen2003Muscle();
void testMillard2012EquilibriumMuscle();
void testMillard2012AccelerationMuscle();
void testDeGrooteFregly2016Muscle();
void testSchutte1993Muscle();
void testDelp1990Muscle();

void testMuscleEquilibriumSolve(const Model& model, const Storage& statesStore);

int main()
{
    SimTK::Array_<std::string> failures;
    
    /*
    try { testThelen2003Muscle_Deprecated();
        cout << "Thelen2003Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
        { e.print(cerr); failures.push_back("testThelen2003Muscle_Deprecated");}
        
    try { testSchutte1993Muscle();
        cout << "Schutte1993Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
        { e.print(cerr); failures.push_back("testSchutte1993Muscle"); }

    try { testDelp1990Muscle();
        cout << "Delp1990Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
        { e.print(cerr); failures.push_back("testDelp1990Muscle"); }
*/
    try { testRigidTendonMuscle();
        cout << "RigidTendonMuscle Test passed" << endl; }
    catch (const Exception& e)
        { e.print(cout); failures.push_back("testRigidTendonMuscle"); }

    try { testThelen2003Muscle();
        cout << "Thelen2003Muscle Test passed" << endl; }
    catch (const Exception& e)
        { e.print(cout); failures.push_back("testThelen2003Muscle"); }

    try { testMillard2012EquilibriumMuscle();
        cout << "Millard2012EquilibriumMuscle Test passed" << endl; 
    }catch (const Exception& e){ 
        e.print(cout);
        failures.push_back("testMillard2012EquilibriumMuscle");
    }
    try { testMillard2012AccelerationMuscle();
        cout << "Millard2012AccelerationMuscle Test passed" << endl; 
    }catch (const Exception& e){ 
        e.print(cout);
        failures.push_back("testMillard2012AccelerationMuscle");
    }
    try { testDeGrooteFregly2016Muscle();
        cout << "DeGrooteFregly2016Muscle Test passed" << endl;
    } catch (const Exception& e) {
        e.print(cout);
        failures.push_back("testDeGrooteFregly2016Muscle");
    }

    printf("\n\n");
    cout <<"************************************************************"<<endl;
    cout <<"************************************************************"<<endl;

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    
    cout << "testMuscles Done" << endl;
    return 0;
}

/*==============================================================================  
    Main test driver to be used on any muscle model (derived from Muscle) so new 
    cases should be easy to add currently, the test only verifies that the work 
    done by the muscle corresponds to the change in system energy.
================================================================================
*/
void simulateMuscle(
        const Muscle &aMuscModel, 
        double startX, 
        double act0, 
        const Function *motion,  // prescribe motion of free end of muscle
        const Function *control, // prescribed excitation signal to the muscle
        bool printResults)
{
    string prescribed = (motion == NULL) ? "." : " with Prescribed Motion.";

    cout << "\n******************************************************" << endl;
    cout << "Test " << aMuscModel.getConcreteClassName() 
         << " Model" << prescribed << endl;
    cout << "******************************************************" << endl;
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
    *aMuscle = aMuscModel;

    // Get a reference to the model's ground body
    Ground& ground = model.updGround();
    //ground.updDisplayer()->setScaleFactors(Vec3(anchorWidth, anchorWidth, 2*anchorWidth));

    OpenSim::Body * ball = new OpenSim::Body("ball", 
                        ballMass , 
                        Vec3(0),  
                        ballMass*SimTK::Inertia::sphere(ballRadius));
    
    ball->attachGeometry(new Sphere(ballRadius));
    //ball->updDisplayer()->setScaleFactors(Vec3(2*ballRadius));
    // ball connected  to ground via a slider along X
    double xSinG = optimalFiberLength*cos(pennationAngle)+tendonSlackLength;

    SliderJoint* slider = new SliderJoint( "slider", 
                        ground, 
                        Vec3(anchorWidth/2+xSinG, 0, 0), 
                        Vec3(0), 
                        *ball, 
                        Vec3(0), 
                        Vec3(0));

    auto& sliderCoord = slider->updCoordinate();
    sliderCoord.setName("tx");
    sliderCoord.setDefaultValue(1.0);
    sliderCoord.setRangeMin(0); 
    sliderCoord.setRangeMax(1.0);

    if(motion != NULL){
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
    const string &actuatorType = aMuscle->getConcreteClassName();
    aMuscle->setName("muscle");
    aMuscle->addNewPathPoint("muscle-box", ground, Vec3(anchorWidth/2,0,0));
    aMuscle->addNewPathPoint("muscle-ball", *ball, Vec3(-ballRadius,0,0));
    
    ActivationFiberLengthMuscle_Deprecated *aflMuscle 
        = dynamic_cast<ActivationFiberLengthMuscle_Deprecated *>(aMuscle);
    if(aflMuscle){
        // Define the default states for the muscle that has 
        //activation and fiber-length states
        aflMuscle->setDefaultActivation(act0);
        aflMuscle->setDefaultFiberLength(aflMuscle->getOptimalFiberLength());
    }else{
        ActivationFiberLengthMuscle *aflMuscle2 
            = dynamic_cast<ActivationFiberLengthMuscle *>(aMuscle);
        if(aflMuscle2){
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
    std::unique_ptr<PrescribedController> 
        muscleController{new PrescribedController{}};
    if(control != NULL){
        muscleController->setActuators(model.updActuators());
        // Set the individual muscle control functions 
        //for the prescribed muscle controller
        muscleController->prescribeControlForActuator("muscle",control->clone());

        // Add the control set controller to the model
        model.addController(muscleController.release());
    }

    // Set names for muscles / joints.
    Array<string> muscNames;
    muscNames.append(aMuscle->getName());
    Array<string> jointNames;
    jointNames.append("slider");

//==========================================================================
// 2. SIMULATION SETUP: Instrument the test with probes
//==========================================================================

    // Add an ActuatorPowerProbe to measure the work done by the muscle actuator 
    ActuatorPowerProbe * muscWorkProbe = 
        new ActuatorPowerProbe(muscNames, true, 1);
    muscWorkProbe->setOperation("integrate");
    model.addProbe(muscWorkProbe);

    // Add a JointInternalPowerProbe to measure the work done by the joint
    // will be 0 unless joint has prescribed motion
    JointInternalPowerProbe * jointWorkProbe = 
        new JointInternalPowerProbe(jointNames, true, 1);
    jointWorkProbe->setOperation("integrate");
    model.addProbe(jointWorkProbe);

    model.finalizeConnections(); // Needed so sockets have correct absolute path on print
    /* Since all components are allocated on the stack don't have model 
       own them (and try to free)*/
//  model.disownAllComponents();
    model.setName(actuatorType+"ModelTest");
    model.print(actuatorType+"ModelTest.osim");

    /* Setup a Muscle Analysis to report all internal values of the 
       muscle during the simulation. If you uncomment, remember to 
       uncomment the corresponding calls to write the results to 
       file after the simulation.*/
    MuscleAnalysis * muscleAnalysis = new MuscleAnalysis();
    Array<string> tmp; 
    tmp.append("muscle");
    muscleAnalysis->setMuscles(tmp);
    model.addAnalysis(muscleAnalysis);
   
    /*
        Muscle *musclePtr = dynamic_cast<Muscle *>(aMuscle);
        MuscleActiveFiberPowerProbe fiberPowerProbe(&musclePtr); 
        fiberPowerProbe.setName("ActiveFiberPower");
        fiberPowerProbe.setOperation("integrate");
        fiberPowerProbe.setOperationParameter(0);
    
        if(musclePtr){
            model.addProbe(&fiberPowerProbe);
        }
    */

    // Define visualizer for debugging
    //model.setUseVisualizer(true);

//==========================================================================
// 3. SIMULATION Initialization
//==========================================================================

    // Initialize the system and get the default state    
    SimTK::State& si = model.initSystem();
    model.getMultibodySystem().realize(si,SimTK::Stage::Dynamics);
    model.equilibrateMuscles(si);

    CoordinateSet& modelCoordinateSet = model.updCoordinateSet();

    // Define non-zero (defaults are 0) states for the free joint
    // set x-translation value
    modelCoordinateSet[0].setValue(si, startX, true); 

    //Copy the initial state
    SimTK::State initialState(si);

    // Check muscle is setup correctly 
    const PathActuator &actu 
        = dynamic_cast<const PathActuator&>(model.updActuators().get("muscle"));
    double length = actu.getLength(si);
    double trueLength = startX + xSinG - anchorWidth/2;
    
    ASSERT_EQUAL(length/trueLength, 1.0, InitializationTestTolerance,
                __FILE__, __LINE__,
                "testMuscles: path failed to initialize to correct length." );

    model.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);

    double Emuscle0 = muscWorkProbe->getProbeOutputs(si)(0);
    //cout << "Muscle initial energy = " << Emuscle0 << endl;
    double Esys0 = model.getMultibodySystem().calcEnergy(si);
    Esys0 += (Emuscle0 + jointWorkProbe->getProbeOutputs(si)(0));
    /*double PEsys0 = */model.getMultibodySystem().calcPotentialEnergy(si);
    //cout << "Total initial system energy = " << Esys0 << endl; 

//==========================================================================
// 4. SIMULATION Integration
//==========================================================================

    // Create the manager
    Manager manager(model);
    manager.setIntegratorAccuracy(IntegrationAccuracy);

    // Integrate from initial time to final time
    si.setTime(initialTime);
    manager.initialize(si);
    cout<<"\nIntegrating from " << initialTime<< " to " << finalTime << endl;

    // Start timing the simulation
    const clock_t start = clock();
    // simulate
    manager.integrate(finalTime);

    // how long did it take?
    double comp_time = (double)(clock()-start)/CLOCKS_PER_SEC;

    // Save the simulation results
    Storage states(manager.getStateStorage());
    states.print(actuatorType+"_states.sto");

    StatesTrajectory statesTraj =
        StatesTrajectory::createFromStatesStorage(model, states);

//==========================================================================
// 5. SIMULATION Reporting
//==========================================================================

    double realTimeMultiplier = ((finalTime-initialTime)/comp_time);
    printf("testMuscles: Realtime Multiplier: %f\n"
           "           :  simulation duration / clock duration\n"
           "              > 1 : faster than real time\n"
           "              = 1 : real time\n"
           "              < 1 : slower than real time\n",
            realTimeMultiplier );
    
    /*
    ASSERT(comp_time <= (finalTime-initialTime));
    printf("testMuscles: PASSED Realtime test\n"
           "             %s simulation time: %f with accuracy %f\n\n",
                         actuatorType.c_str(), comp_time , accuracy);
    */

    //An analysis only writes to a dir that exists, so create here.
    if(printResults){
        IO::makeDir("testMuscleResults");
        muscleAnalysis->printResults(actuatorType, "testMuscleResults");
    }

    double muscleWork = muscWorkProbe->getProbeOutputs(si)(0);
    cout << "Muscle work = " << muscleWork << endl;

//==========================================================================
// 6. SIMULATION Tests
//==========================================================================
    auto& muscle = model.updMuscles()[0];
    testMuscleEquilibriumSolve(model, states);

/*==========================================================================
    7. Correctness test:  d/dt(KE+PE-W) = 0 ?
    
    Check that the derivative of system energy less work is conserved
============================================================================
*/      
    if(true){
        Storage* fiberActivePwrSto  
            = muscleAnalysis->getFiberActivePowerStorage(); 
        Storage* fiberPassivePwrSto 
            = muscleAnalysis->getFiberPassivePowerStorage();
        Storage* tendonPwrSto       
            = muscleAnalysis->getTendonPowerStorage();
        Storage* musclePwrSto       
            = muscleAnalysis->getMusclePowerStorage();


        double *fiberActivePwrDat  = NULL;
        double *fiberPassivePwrDat = NULL;
        double *tendonPwrDat       = NULL;
        double *musclePwrDat       = NULL;

        fiberActivePwrSto->getDataColumn(   "#1",fiberActivePwrDat);
        fiberPassivePwrSto->getDataColumn(  "#1",fiberPassivePwrDat);
        tendonPwrSto->getDataColumn("#1", tendonPwrDat);
        musclePwrSto->getDataColumn("#1",musclePwrDat);
        std::unique_ptr<double[]>  fiberActivePwrDat_ptr{fiberActivePwrDat};
        std::unique_ptr<double[]> fiberPassivePwrDat_ptr{fiberPassivePwrDat};
        std::unique_ptr<double[]>       tendonPwrDat_ptr{tendonPwrDat};
        std::unique_ptr<double[]>       musclePwrDat_ptr{musclePwrDat};

        double dKEPEW_dt = 0;
        
        double dtendonPE    = 0;
        double dfiberPE     = 0;
        double dfiberW      = 0;
        double dboundaryW   = 0;

        int numSteps = fiberActivePwrSto->getSize();

        for(int i=0; i<numSteps; i++){
            dtendonPE   = -tendonPwrDat[i];
            dfiberPE    = -fiberPassivePwrDat[i];
            dfiberW     =  fiberActivePwrDat[i];
            dboundaryW  =  -musclePwrDat[i];

            dKEPEW_dt = dtendonPE + dfiberPE - dfiberW - dboundaryW;

            ASSERT_EQUAL( dKEPEW_dt, 0.0, CorrectnessTestTolerance,
                          __FILE__, __LINE__,
                        "testMuscles: d/dt(system energy-work) non-zero.");
        }
    }
}


//==============================================================================
// Individual muscle model (derived from Muscle) test cases can be added here
//==============================================================================

void testRigidTendonMuscle()
{
    RigidTendonMuscle   muscle( "muscle",
                                MaxIsometricForce0,
                                OptimalFiberLength0,
                                TendonSlackLength0,
                                PennationAngle0);

    double x0 = 0;
    double act0 = 0.5;
    Constant control(act0);

    Sine motion(OptimalFiberLength0, 2.0*SimTK::Pi, 0);

    // concentric
    simulateMuscle(muscle, 
        x0, 
        act0, 
        NULL, 
        &control, 
        false);

    // eccentric 
    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion,
        &control, 
        false);
}



void testThelen2003Muscle_Deprecated()
{
    Thelen2003Muscle_Deprecated muscle("muscle",
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
                    NULL, 
                    &control, 
                    false);
}

void testThelen2003Muscle()
{

    Thelen2003Muscle muscle("muscle",
                            MaxIsometricForce0,
                            OptimalFiberLength0,
                            TendonSlackLength0,
                            PennationAngle0);

    Thelen2003Muscle muscle1("muscle",
                            MaxIsometricForce0,
                            OptimalFiberLength0,
                            TendonSlackLength0,
                            PennationAngle1);

    muscle.setActivationTimeConstant(Activation0);
    muscle.setDeactivationTimeConstant(Deactivation0);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(OptimalFiberLength0, 2.0*SimTK::Pi, 0);


    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control, 
        false);

    simulateMuscle(muscle1, 
        x0, 
        act0, 
        &motion, 
        &control, 
        false);

    Model m;
    muscle.addNewPathPoint("p1", m.getGround(), SimTK::Vec3(0.0));
    muscle.addNewPathPoint("p2", m.getGround(), SimTK::Vec3(1.0));
    // Test property bounds.
    {
        Thelen2003Muscle musc = muscle;
        musc.set_FmaxTendonStrain(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);

        musc.set_FmaxTendonStrain(0.1);
        musc.finalizeFromProperties();
    }
    {
        Thelen2003Muscle musc = muscle;
        musc.set_FmaxMuscleStrain(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc = muscle;
        musc.set_KshapeActive(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc = muscle;
        musc.set_KshapePassive(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc = muscle;
        musc.set_Af(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc = muscle;
        musc.set_Flen(1.001);
        musc.set_fv_linear_extrap_threshold(5.0);
        musc.finalizeFromProperties();

        musc.set_Flen(1.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc = muscle;
        musc.set_Flen(1.3);
        musc.finalizeFromProperties();
    }
    {
        Thelen2003Muscle musc = muscle;
        musc.set_fv_linear_extrap_threshold(1.0 / 1.4);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);

        musc.set_fv_linear_extrap_threshold(1.007);
        musc.finalizeFromProperties();

        musc.set_Flen(3.0);

        musc.set_fv_linear_extrap_threshold(1.0 / 3.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);

        musc.set_fv_linear_extrap_threshold(1.001 / 3.0);
        musc.finalizeFromProperties();
    }

    // Ensure the properties of MuscleFixedWidthPennationModel and
    // MuscleFirstOrderActivationDynamicModel are being set by Thelen2003Muscle
    // when they are subcomponents of the Muscle. Here, we set the properties
    // of Thelen2003Muscle and then check the properties of its subcomponents.
    // Also ensures properties survive serialization/deserialization.
    {
        const string& filename = "testSettingMuscleSubcomponentProperties.osim";
        const double optimalFiberLength = 1234.56;
        const double pennAngAtOptimal   = 0.123456;
        const double maximumPennation   = 1.23456;
        const double actTimeConstant    = 0.0123;
        const double deactTimeConstant  = 0.0246;
        const double minimumActivation  = 0.0357;

        // Create muscle and add to model.
        Model myModel;
        cout << "new Thelen2003Muscle myMuscle" << endl;
        Thelen2003Muscle* myMcl = new Thelen2003Muscle("myMuscle",
            MaxIsometricForce0, OptimalFiberLength0, TendonSlackLength0,
            PennationAngle0);
        
        myMcl->addNewPathPoint("p1", myModel.getGround(), SimTK::Vec3(0.0));
        myMcl->addNewPathPoint("p2", myModel.getGround(), SimTK::Vec3(1.0));
        myModel.addForce(myMcl);

        // Set properties of Thelen2003Muscle.
        myMcl->setOptimalFiberLength(optimalFiberLength);
        myMcl->setPennationAngleAtOptimalFiberLength(pennAngAtOptimal);
        myMcl->setMaximumPennationAngle(maximumPennation);
        myMcl->setActivationTimeConstant(actTimeConstant);
        myMcl->setDeactivationTimeConstant(deactTimeConstant);
        myMcl->setMinimumActivation(minimumActivation);
        myMcl->setMinControl(minimumActivation);

        cout << "myMuscle->finalizeFromProperties" << endl;
        myMcl->finalizeFromProperties();

        // Check properties of MuscleFixedWidthPennationModel.
        const MuscleFixedWidthPennationModel& pennMdl =
            myMcl->getPennationModel();
        ASSERT_EQUAL(optimalFiberLength, pennMdl.get_optimal_fiber_length(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "optimal_fiber_length was not set in pennation model");
        ASSERT_EQUAL(pennAngAtOptimal, pennMdl.get_pennation_angle_at_optimal(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "pennation_angle_at_optimal was not set in pennation model");
        ASSERT_EQUAL(maximumPennation, pennMdl.get_maximum_pennation_angle(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "maximum_pennation_angle was not set in pennation model");

        // Check properties of MuscleFirstOrderActivationDynamicModel.
        const MuscleFirstOrderActivationDynamicModel& actMdl =
            myMcl->getActivationModel();
        ASSERT_EQUAL(actTimeConstant, actMdl.get_activation_time_constant(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "activation_time_constant was not set in activation model");
        ASSERT_EQUAL(deactTimeConstant, actMdl.get_deactivation_time_constant(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "deactivation_time_constant was not set in activation model");
        ASSERT_EQUAL(minimumActivation, actMdl.get_minimum_activation(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "minimum_activation was not set in activation model");

        myModel.finalizeConnections();  // Needed so sockets have correct absolute path on print
        // Print model and read back in.
        myModel.print(filename);
        Model myModel2(filename);

        const Thelen2003Muscle& myMcl2 = dynamic_cast<const Thelen2003Muscle&>(
            myModel2.getMuscles().get("myMuscle") );

        // Check properties of MuscleFixedWidthPennationModel.
        const MuscleFixedWidthPennationModel& pennMdl2 =
            myMcl2.getPennationModel();
        ASSERT_EQUAL(optimalFiberLength, pennMdl2.get_optimal_fiber_length(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "optimal_fiber_length was not set in pennation model");
        ASSERT_EQUAL(pennAngAtOptimal, pennMdl2.get_pennation_angle_at_optimal(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "pennation_angle_at_optimal was not set in pennation model");
        ASSERT_EQUAL(maximumPennation, pennMdl2.get_maximum_pennation_angle(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "maximum_pennation_angle was not set in pennation model");

        // Check properties of MuscleFirstOrderActivationDynamicModel.
        const MuscleFirstOrderActivationDynamicModel& actMdl2 =
            myMcl2.getActivationModel();
        ASSERT_EQUAL(actTimeConstant, actMdl2.get_activation_time_constant(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "activation_time_constant was not set in activation model");
        ASSERT_EQUAL(deactTimeConstant, actMdl2.get_deactivation_time_constant(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "deactivation_time_constant was not set in activation model");
        ASSERT_EQUAL(minimumActivation, actMdl2.get_minimum_activation(),
            SimTK::SignificantReal, __FILE__, __LINE__,
            "minimum_activation was not set in activation model");
    }

    // Test exception when muscle cannot be initialized.
    {
        Model model;

        const double optimalFiberLength = 0.001; //short fiber and tendon
        const double tendonSlackLength  = 0.001;
        auto muscle = new Thelen2003Muscle("muscle", 1., optimalFiberLength,
                                           tendonSlackLength, 0.);
        muscle->addNewPathPoint("p1", model.updGround(), SimTK::Vec3(0));
        muscle->addNewPathPoint("p2", model.updGround(), SimTK::Vec3(0,0,1));
        model.addForce(muscle);

        SimTK::State& state = model.initSystem();
        ASSERT_THROW( MuscleCannotEquilibrate,
                      muscle->computeInitialFiberEquilibrium(state) );
    }

    // Test exception handling when invalid properties are propagated to
    // MuscleFixedWidthPennationModel and MuscleFirstOrderActivationDynamicModel
    // subcomponents.
    {
        Model model;
        auto muscle = new Thelen2003Muscle("muscle", 1., 0.5, 0.5, 0.);
        muscle->addNewPathPoint("p1", model.updGround(), SimTK::Vec3(0));
        muscle->addNewPathPoint("p2", model.updGround(), SimTK::Vec3(0,0,1));
        model.addForce(muscle);
        model.finalizeFromProperties();

        // Set each property that is propagated to the pennation model outside
        // its valid range.
        muscle->setOptimalFiberLength(0.);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setOptimalFiberLength(0.5);
        model.finalizeFromProperties();

        muscle->setPennationAngleAtOptimalFiberLength(SimTK::Pi/2.0 + 0.1);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setPennationAngleAtOptimalFiberLength(0.);
        model.finalizeFromProperties();

        muscle->setMaximumPennationAngle(SimTK::Pi/2.0 + 0.1);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setMaximumPennationAngle(0.);
        model.finalizeFromProperties();

        // Set each property that is propagated to the activation dynamics model
        // outside its valid range.
        muscle->setActivationTimeConstant(0.);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setActivationTimeConstant(0.1);
        model.finalizeFromProperties();

        muscle->setDeactivationTimeConstant(0.);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setDeactivationTimeConstant(0.1);
        model.finalizeFromProperties();

        muscle->setMinimumActivation(-0.1);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setMinimumActivation(0.01);
        model.finalizeFromProperties();
    }
}


void testMillard2012EquilibriumMuscle()
{
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

    Sine motion(2.0*OptimalFiberLength0, 2*SimTK::Pi, 0);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control,
        false);

    // Test extremely short fiber where muscle should still initialize.
    {
        Model model;

        const double optimalFiberLength = 0.01; //short fiber and
        const double tendonSlackLength  = 100.0; //long tendon
        auto muscle = new Millard2012EquilibriumMuscle("muscle", 1.,
                          optimalFiberLength, tendonSlackLength, 0.);
        muscle->addNewPathPoint("p1", model.updGround(), SimTK::Vec3(0));
        muscle->addNewPathPoint("p2", model.updGround(), SimTK::Vec3(0,0,1));
        model.addForce(muscle);

        SimTK::State& state = model.initSystem();
        muscle->setActivation(state, 1.);
        model.realizeVelocity(state);
        muscle->computeInitialFiberEquilibrium(state);
    }

    // Test extremely short (but stretched) fiber, which should still initialize.
    {
        Model model;

        const double optimalFiberLength = 0.01; //short fiber and
        const double tendonSlackLength =  0.1; //short tendon
        auto muscle = new Millard2012EquilibriumMuscle("muscle", 1.,
            optimalFiberLength, tendonSlackLength, 0.);
        muscle->addNewPathPoint("p1", model.updGround(), SimTK::Vec3(0));
        muscle->addNewPathPoint("p2", model.updGround(), SimTK::Vec3(0, 0, 1));
        model.addForce(muscle);

        SimTK::State& state = model.initSystem();
        muscle->setActivation(state, 0.01);
        model.realizeVelocity(state);
        muscle->computeInitialFiberEquilibrium(state);
    }

    // Test exception handling when invalid properties are propagated to
    // MuscleFixedWidthPennationModel and MuscleFirstOrderActivationDynamicModel
    // subcomponents.
    {
        Model model;
        auto muscle = new Millard2012EquilibriumMuscle("mcl", 1., 0.5, 0.5, 0.);
        muscle->addNewPathPoint("p1", model.updGround(), SimTK::Vec3(0));
        muscle->addNewPathPoint("p2", model.updGround(), SimTK::Vec3(0,0,1));
        model.addForce(muscle);
        model.finalizeFromProperties();

        // Set each property that is propagated to the pennation model outside
        // its valid range.
        muscle->setOptimalFiberLength(0.);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setOptimalFiberLength(0.5);
        model.finalizeFromProperties();

        muscle->setPennationAngleAtOptimalFiberLength(SimTK::Pi/2.0 + 0.1);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setPennationAngleAtOptimalFiberLength(0.);
        model.finalizeFromProperties();

        muscle->set_maximum_pennation_angle(SimTK::Pi/2.0 + 0.1);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->set_maximum_pennation_angle(0.);
        model.finalizeFromProperties();

        // Set each property that is propagated to the activation dynamics model
        // outside its valid range.
        muscle->setActivationTimeConstant(0.);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setActivationTimeConstant(0.1);
        model.finalizeFromProperties();

        muscle->setDeactivationTimeConstant(0.);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setDeactivationTimeConstant(0.1);
        model.finalizeFromProperties();

        muscle->setMinimumActivation(-0.1);
        ASSERT_THROW(InvalidPropertyValue, model.finalizeFromProperties());
        muscle->setMinimumActivation(0.01);
        model.finalizeFromProperties();
    }
}

void testMillard2012AccelerationMuscle()
{
    Millard2012AccelerationMuscle muscle("muscle",
                            MaxIsometricForce0,
                            OptimalFiberLength0,
                            TendonSlackLength0,
                            PennationAngle0);

    //otherwise the simulations are a bit slow ...
    muscle.setMass(muscle.getMass()*10);

    MuscleFirstOrderActivationDynamicModel actMdl = muscle.getActivationModel();
    actMdl.set_activation_time_constant(Activation0);
    actMdl.set_deactivation_time_constant(Deactivation0);
    muscle.setActivationModel(actMdl);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    // Note that simulateMuscle will call testMuscleEquilibriumSolve() but it
    // will ignore the Millard2012AccelerationMuscle because it is not an
    // equilibrium-based muscle model.
    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control,
        false);
}

void testDeGrooteFregly2016Muscle() {

    DeGrooteFregly2016Muscle muscle;
    muscle.setName("muscle");
    muscle.set_max_isometric_force(MaxIsometricForce0);
    muscle.set_optimal_fiber_length(OptimalFiberLength0);
    muscle.set_tendon_slack_length(TendonSlackLength0);
    muscle.set_pennation_angle_at_optimal(PennationAngle0);
    muscle.set_tendon_compliance_dynamics_mode("explicit");
    muscle.set_activation_time_constant(Activation0);
    muscle.set_deactivation_time_constant(Deactivation0);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(OptimalFiberLength0, SimTK::Pi, 0);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion,  
        &control, 
        false);
}

void testSchutte1993Muscle()
{
    Schutte1993Muscle_Deprecated muscle("muscle",
                                        MaxIsometricForce0,
                                        OptimalFiberLength0,
                                        TendonSlackLength0,
                                        PennationAngle0);

    muscle.setActivation1(ShutteDelpActivation1);
    muscle.setActivation2(ShutteDelpActivation2);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control,
        false);

}


void testDelp1990Muscle()
{
    Delp1990Muscle_Deprecated muscle("muscle",
                                    MaxIsometricForce0,
                                    OptimalFiberLength0,
                                    TendonSlackLength0,
                                    PennationAngle0);

    muscle.setActivation1(ShutteDelpActivation1);
    muscle.setActivation2(ShutteDelpActivation2);
    muscle.setMass(0.1);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control,
        false);
}



void testMuscleEquilibriumSolve(const Model& model, const Storage& statesStore)
{
    // Get the muscle to test
    const Muscle& muscle = model.getMuscles()[0];

    if (dynamic_cast<const Millard2012AccelerationMuscle*>(&muscle)) {
        // Millard2012AccelerationMuscle is not an Equilibrium-based Muscle
        return;
    }

    // Load input data as StatesTrajectory used to perform the Analysis
    auto statesTraj = StatesTrajectory::createFromStatesStorage(
        model, statesStore, true, false);
    size_t nstates = statesTraj.getSize();

    // muscle active, passive, total muscle and tendon force quantities
    double af, pf, mf, tf, nfl, fv;
    af = pf = mf = tf = nfl = fv = SimTK::NaN;

    // Tolerance for muscle equilibrium solution 
    const double equilTol = muscle.getMaxIsometricForce()*SimTK::SqrtEps;

    // The maximum acceptable change in force between two contiguous states
    const double maxDelta = muscle.getMaxIsometricForce() / 10;

    const int N = 20;
    const double minActivation = muscle.getMinControl();
    const double dAct = (1.0 - minActivation) / N;
    double activation = 0;

    SimTK::State s = model.getWorkingState();
    // Independently compute the active fiber force at every state
    for (size_t i = 0; i < nstates; ++i) {
        s = statesTraj[i];

        // test a full sweep of default activations at each state
        for (int j = 0; j <= N; ++j) {
            activation = minActivation + j*dAct;
            muscle.setActivation(s, activation);

            try {
                muscle.computeEquilibrium(s);
            }
            catch (const MuscleCannotEquilibrate&) {
                // Write out the muscle equilibrium error as a function of
                // fiber lengths.
                if (const auto* thelen =
                    dynamic_cast<const Thelen2003Muscle*>(&muscle)) {
                    thelen->printCurveToCSVFile(
                        Thelen2003Muscle::CurveType::FiberForceVelocity, "");
                    reportTendonAndFiberForcesAcrossFiberLengths(*thelen, s);
                }
                else if (const auto* millard =
                    dynamic_cast<const Millard2012EquilibriumMuscle*>(&muscle)) {
                    reportTendonAndFiberForcesAcrossFiberLengths(*millard, s);
                }

                throw;
            }

            model.realizeDynamics(s);

            // Get the fiber-length
            nfl = muscle.getNormalizedFiberLength(s);

            SimTK_ASSERT_ALWAYS(nfl >= 0.0, 
                "Equilibrium failed to compute valid fiber length.");

            // get active and passive forces given the default activation
            af = muscle.getActiveFiberForceAlongTendon(s);
            pf = muscle.getPassiveFiberForceAlongTendon(s);
            // now the total muscle force is the active + passive
            mf = af + pf;
            tf = muscle.getTendonForce(s);

            // equilibrium demands tendon and muscle fiber forces are equivalent
            ASSERT_EQUAL<double>(tf, mf, equilTol,
                __FILE__, __LINE__, "testMuscleEquilibriumSolve(): " +
                muscle.getConcreteClassName() + 
                " failed to solve for muscle (fiber) and tendon equilibrium. ");
        }
    }
}
