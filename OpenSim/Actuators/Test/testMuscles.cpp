/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testMuscles.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
//  testMuscles simulates various OpenSim models using the OpenSim API and 
//  compares muscle behavior for varying physical parameters (fiber-to-tendon 
//  ratio, tendon stiffness, etc...)
//
//  Models tested include:
//      1. PathActuator (Base of Muscle, is controlled tension along a GeometryPath)
//      2. RigidTendonMuscle (Stateless muscle with user-defined fiber f-l, f-v splines)
//      3. Thelen2003Muscle_Deprecated (Simm implementation)
//      4. Thelen2003Muscle (Uses the Muscle interface)
//      5. Schutte1993Muscle(_Deprecated)
//      6. Delp1990Muscle(_Deprecated)
//      
//     Add more test cases to address specific problems with muscle models
//
//==============================================================================
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Analyses/osimAnalyses.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

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
@param accuracy the desired accuracy of the integrated solution
@param testType 0: No test, just simulate the muscle 
                1: Initialization test
                2: Correctness test: ensure that d/dt(KE+PE-W) = 0 
@param testTolerance    the desired tolerance associated with the test
@param printResults print the osim model associated with this test.
*/
void simulateMuscle(const Muscle &aMuscle, 
                    double startX, 
                    double act0, 
                    const Function *motion, 
                    const Function *control, 
                    double integrationAccuracy,
                    int testType,
                    double testTolerance,
                    bool printResults);

//void testPathActuator();
void testRigidTendonMuscle();
void testThelen2003Muscle_Deprecated();
void testThelen2003Muscle();
void testMillard2012EquilibriumMuscle();
void testMillard2012AccelerationMuscle();
void testSchutte1993Muscle();
void testDelp1990Muscle();

int main()
{
    SimTK::Array_<std::string> failures;
    
    printf(
         "The following tests have been removed:\n" 
         "    1) testPathActuator\n" 
         "        This is not a muscle, and so its internal power and energy \n"
         "        transfers cannot be measured and its correctness cannot be\n"
         "        determined\n"
         "    2) testThelen2003Muscle_Deprecated\n"
         "    3) testShutte1993Muscle\n"
         "    4)  testDelp1990Muscle \n"
         "        These muscle models (items 2,3,4) do not implement\n "
         "        calcMuscleDynamicsInfo and so its impossible to measure the\n"
         "        internal power and energy transfer within these internal\n "
         "        power transfers cannot be measured and its correctness\n "
         "        cannot be determined\n\n"
        );

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
        { e.print(cerr); failures.push_back("testRigidTendonMuscle"); }
    
    try { testThelen2003Muscle();
        cout << "Thelen2003Muscle Test passed" << endl; }
    catch (const Exception& e)
        { e.print(cerr); failures.push_back("testThelen2003Muscle"); }
    
    try { testMillard2012EquilibriumMuscle();
        cout << "Millard2012EquilibriumMuscle Test passed" << endl; 
    }catch (const Exception& e){ 
        e.print(cerr);
        failures.push_back("testMillard2012EquilibriumMuscle");
    }
    try { testMillard2012AccelerationMuscle();
        cout << "Millard2012AccelerationMuscle Test passed" << endl; 
    }catch (const Exception& e){ 
        e.print(cerr);
        failures.push_back("testMillard2012AccelerationMuscle");
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

    TODO: Test will fail with prescribed motion until the work done by this 
    constraint is accounted for.
================================================================================
*/
void simulateMuscle(
        const Muscle &aMuscModel, 
        double startX, 
        double act0, 
        const Function *motion,  // prescribe motion of free end of muscle
        const Function *control, // prescribed excitation signal to the muscle
        double integrationAccuracy,
        int testType,
        double testTolerance,
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
    double finalTime = 0.25;
    
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
    
    ball->attachGeometry(Sphere{ballRadius});
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

    CoordinateSet& jointCoordinateSet = slider->upd_CoordinateSet();
        jointCoordinateSet[0].setName("tx");
        jointCoordinateSet[0].setDefaultValue(1.0);
        jointCoordinateSet[0].setRangeMin(0); 
        jointCoordinateSet[0].setRangeMax(1.0);
    
    if(motion != NULL){
        jointCoordinateSet[0].setPrescribedFunction(*motion);
        jointCoordinateSet[0].setDefaultIsPrescribed(true);
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
    ActuatorPowerProbe * muscWorkProbe = new ActuatorPowerProbe(muscNames, true, 1);
    muscWorkProbe->setOperation("integrate");
    model.addProbe(muscWorkProbe);

    // Add a JointInternalPowerProbe to measure the work done by the joint
    // will be 0 unless joint has prescribed motion
    JointInternalPowerProbe * jointWorkProbe = new JointInternalPowerProbe(jointNames, true, 1);
    jointWorkProbe->setOperation("integrate");
    model.addProbe(jointWorkProbe);

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
    const PathActuator &muscle 
        = dynamic_cast<const PathActuator&>(model.updActuators().get("muscle"));
    double length = muscle.getLength(si);
    double trueLength = startX + xSinG - anchorWidth/2;
    
    ASSERT_EQUAL(length/trueLength, 1.0, testTolerance, __FILE__, __LINE__, 
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

    // Create the integrator
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    integrator.setAccuracy(integrationAccuracy);

    // Create the manager
    Manager manager(model, integrator);

    // Integrate from initial time to final time
    manager.setInitialTime(initialTime);
    manager.setFinalTime(finalTime);
    cout<<"\nIntegrating from " << initialTime<< " to " << finalTime << endl;

    // Start timing the simulation
    const clock_t start = clock();
    // simulate
    manager.integrate(si);

    // how long did it take?
    double comp_time = (double)(clock()-start)/CLOCKS_PER_SEC;

    // Save the simulation results
    Storage states(manager.getStateStorage());
    states.print(actuatorType+"_states.sto");


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
    if(printResults == true){
        IO::makeDir("testMuscleResults");
        muscleAnalysis->printResults(actuatorType, "testMuscleResults");
    }

    double muscleWork = muscWorkProbe->getProbeOutputs(si)(0);
    cout << "Muscle work = " << muscleWork << endl;

//==========================================================================
// 6. SIMULATION Tests
//==========================================================================
   


/*==========================================================================
    7. Initialization test: dF/dt ?= K*dl/dt for t = 0

    dF/dt : the rate change of muscle force
    K     : the stiffness of the whole muscle
    dl/dt : the length change of the whole muscle

    Here we compute dF/dt numerically, while at time zero we have the analytical
    values for K and dl/dt
============================================================================
*/    
    if(testType == 1){
      cout << "CREATE THE INITIALIZATION TEST" << endl;    
    }


/*==========================================================================
    8a. Correctness test:  KE+PE-W = const ?
    
    Check that system energy less work is conserved
    *This test of correctness is not being used because I've been unable
    to successfully wire the MuscleFiberActivePowerProbe into the model, 
    nor have I generalized the MuscleDynamicInfo interface enough to permit
    an explicit KE+PE-W test.
============================================================================
*/    
    

    if(false){
        model.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);
        double Esys = model.getMultibodySystem().calcEnergy(si);
        /*double KEsys =  */model.getMultibodySystem().calcKineticEnergy(si);
        /*double xSpeed = */modelCoordinateSet[0].getSpeedValue(si);
        // double KEsysCheck = 0.5*ballMass*xSpeed*xSpeed;
        /*double PEsys =  */model.getMultibodySystem().calcPotentialEnergy(si);
        double jointWork = jointWorkProbe->computeProbeInputs(si)(0);
        double ESysMinusWork = Esys 
                                - muscWorkProbe->computeProbeInputs(si)(0)
                                - jointWork; 


        double muscleWork = 0;//fiberWorkMeter.get
        muscWorkProbe->computeProbeInputs(si);
        cout << "Muscle work = " << muscleWork << endl;  
        cout << "Esys - Work = " << ESysMinusWork 
             << " :: Esys0 = " << Esys0 << endl; 
        ASSERT_EQUAL(ESysMinusWork, Esys0, testTolerance, __FILE__, __LINE__, 
            "testMuscles: System energy-work -not conserved.");
                
        //Minimum requirement to pass is simulation of single 
        //muscle on slider is real-time

    }

/*==========================================================================
    8b. Correctness test:  d/dt(KE+PE-W) = 0 ?
    
    Check that the derivative of system energy less work is conserved
============================================================================
*/      
    if(testType == 2){
        Storage *fiberActivePwrSto  
            = muscleAnalysis->getFiberActivePowerStorage(); 
        Storage *fiberPassivePwrSto 
            = muscleAnalysis->getFiberPassivePowerStorage();
        Storage *tendonPwrSto       
            = muscleAnalysis->getTendonPowerStorage();
        Storage *musclePwrSto       
            = muscleAnalysis->getMusclePowerStorage();

        double *fiberActivePwrDat  = NULL;
        double *fiberPassivePwrDat = NULL;
        double *tendonPwrDat       = NULL;
        double *musclePwrDat       = NULL;

        fiberActivePwrSto->getDataColumn(   "#1",fiberActivePwrDat);
        fiberPassivePwrSto->getDataColumn(  "#1",fiberPassivePwrDat);
        tendonPwrSto->getDataColumn("#1", tendonPwrDat);
        musclePwrSto->getDataColumn("#1",musclePwrDat);

        double dKEPEW_dt = 0;
        
        double dtendonPE    = 0;
        double dfiberPE     = 0;
        double dfiberW      = 0;
        double dboundaryW   = 0;

        int numSteps = fiberActivePwrSto->getSize();
        bool flag_notTested = false;

        for(int i=0; i<numSteps; i++){
            dtendonPE   = -tendonPwrDat[i];
            dfiberPE    = -fiberPassivePwrDat[i];
            dfiberW     =  fiberActivePwrDat[i];
            dboundaryW  =  -musclePwrDat[i]; 

            dKEPEW_dt = dtendonPE + dfiberPE - dfiberW - dboundaryW;

            if(SimTK::isNaN(dKEPEW_dt) == false && flag_notTested == false){
                ASSERT_EQUAL(   dKEPEW_dt, 
                                0.0, 
                                testTolerance,  
                                __FILE__, 
                                __LINE__,
                            "testMuscles: d/dt(system energy-work) non-zero.");
            }else{
                flag_notTested = true;
            }

        }

        if(flag_notTested == false){
            printf("testMuscles: PASSED Correctness test\n"
                  "            : d/dt(system energy-work) = 0\n"
                  "            : with a numerical accuracy of %fe-6\n\n",
                                testTolerance*1e6);
        }else{
            ASSERT_EQUAL(0.0,1.0,0.1,
                   "testMuscles: INCOMPLETE Correctness test\n"
                   "           : Required power fields in MuscleDynamicsInfo\n"
                   "           : struct not populated by this muscle model\n\n");
        }

    }

    /*if(!musclePtr){

        ASSERT_EQUAL(0.0,1.0,0.1,
                   "testMuscles: INCOMPLETE Correctness test\n"
               "             Actuator not of type Muscle tested\n\n");
    }*/


}


//==============================================================================
// Individual muscle model (derived from Muscle) test cases can be added here
//==============================================================================

/*void testPathActuator()
{
    double x0 = 0;
    double act0 = 0.2;
    double loadX = 50;

    PathActuator muscle;
    muscle.setOptimalForce(maxIsometricForce0);

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    // concentric
    //simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
    // eccentric 
    simulateMuscle(muscle, x0, act0, &motion, &control, accuracy,false);
}*/


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

    Sine sineWave(0.1, SimTK::Pi, 0);

    /*
    void simulateMuscle(const Muscle &aMuscle, 
                    double startX, 
                    double act0, 
                    const Function *motion, 
                    const Function *control, 
                    double integrationAccuracy,
                    int testType,
                    double testTolerance,
                    bool printResults);
    */

    // concentric
    simulateMuscle(muscle, 
        x0, 
        act0, 
        NULL, 
        &control, 
        IntegrationAccuracy,
        SimulationTest,
        SimulationTestTolerance,
        false);
    // eccentric 
    simulateMuscle(muscle, 
        x0, 
        act0, 
        &sineWave, 
        &control, 
        IntegrationAccuracy,
        CorrectnessTest,
        CorrectnessTestTolerance,
        false);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &sineWave, 
        &control, 
        IntegrationAccuracy,
        InitializationTest,
        InitializationTestTolerance,
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
                    IntegrationAccuracy,
                    SimulationTest,
                    SimulationTestTolerance,
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

    Sine motion(OptimalFiberLength0, SimTK::Pi*2, 0);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control, 
        IntegrationAccuracy,
        InitializationTest,
        InitializationTestTolerance,
        false);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control, 
        IntegrationAccuracy,
        CorrectnessTest,
        CorrectnessTestTolerance,
        false);

    simulateMuscle(muscle1, 
        x0, 
        act0, 
        &motion, 
        &control, 
        IntegrationAccuracy,
        CorrectnessTest,
        CorrectnessTestTolerance,
        false);

    // Test property bounds.
    {
        Thelen2003Muscle musc;
        musc.set_FmaxTendonStrain(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);

        musc.set_FmaxTendonStrain(0.1);
        musc.finalizeFromProperties();
    }
    {
        Thelen2003Muscle musc;
        musc.set_FmaxMuscleStrain(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc;
        musc.set_KshapeActive(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc;
        musc.set_KshapePassive(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc;
        musc.set_Af(0.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc;
        musc.set_Flen(1.001);
        musc.set_fv_linear_extrap_threshold(5.0);
        musc.finalizeFromProperties();

        musc.set_Flen(1.0);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                SimTK::Exception::ErrorCheck);
    }
    {
        Thelen2003Muscle musc;
        musc.set_Flen(1.3);
        musc.finalizeFromProperties();
    }
    {
        Thelen2003Muscle musc;
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
        Thelen2003Muscle* myMcl = new Thelen2003Muscle("myMuscle",
            MaxIsometricForce0, OptimalFiberLength0, TendonSlackLength0,
            PennationAngle0);
        myModel.addForce(myMcl);

        // Set properties of Thelen2003Muscle.
        myMcl->setOptimalFiberLength(optimalFiberLength);
        myMcl->setPennationAngleAtOptimalFiberLength(pennAngAtOptimal);
        myMcl->setMaximumPennationAngle(maximumPennation);
        myMcl->setActivationTimeConstant(actTimeConstant);
        myMcl->setDeactivationTimeConstant(deactTimeConstant);
        myMcl->setMinimumActivation(minimumActivation);

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

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control, 
        IntegrationAccuracy,
        InitializationTest,
        InitializationTestTolerance,
        false);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control, 
        IntegrationAccuracy,
        CorrectnessTest,
        CorrectnessTestTolerance,
        false);
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

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control, 
        IntegrationAccuracy,
        InitializationTest,
        InitializationTestTolerance,
        false);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control, 
        IntegrationAccuracy,
        CorrectnessTest,
        CorrectnessTestTolerance,
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
        IntegrationAccuracy,
        SimulationTest,
        SimulationTestTolerance,
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
        IntegrationAccuracy,
        SimulationTest,
        SimulationTestTolerance,
        false);

}
