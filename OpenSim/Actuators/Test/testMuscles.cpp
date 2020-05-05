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
static const double EquilibriumTestTolerance = 1e-6;
static const double CorrectnessTestTolerance    = 1e-6;

static const int SimulationTest         = 0;
static const int EquilibriumTest     = 1;
static const int CorrectnessTest        = 2;


// MUSCLE CONSTANTS
static const double MaxIsometricForce         = 100.0,
                    OptimalFiberLength        = 0.1,
                    TendonSlackLengthMedium   = 0.2,
                    TendonSlackLengthShort    = 0.01,
                    TendonSlackLengthLong     = 2.0,
                    PennationAngleZero        = 0.0,
                    PennationAngleBig         = SimTK::Pi/4,
                    PennationAngleSmall       = SimTK::Pi/16;


static const double Activation0     = 0.01, 
                    Deactivation0   = 0.4,  
                    ShutteDelpActivation1 = 7.6,    
                    ShutteDelpActivation2 = 2.5;


//Equilibrium tests added for the actively supported muscle models
static const bool testEquilibriumThelen2003Muscle              = true;
static const bool testEquilibriumMillard2012EquilibrumMuscle   = true;
static const bool testEquilibriumMillard2012AccelerationMuscle = true;

/*
@author M.Millard
@date April 2020

    To-Do: Once the muscle models have been updated and are passing the test
    increase the number of sample points, particularly in the path
    lengths tested.

    The parameters in this struct are used to configure the functions
    runMuscleEquilibriumArchitectureAndStateSweep and
    runMuscleEquilibriumStateSweep which equilibrates a muscle across a range of
    activations, path lengths, and path speeds. The default values recommended
    for use in this struct will allow a large sampling of equilibrium tests to
    be conducted across the full range of states, activations, and muscle
    architectures that might be used so that problems related to equilibration
    are caught here rather than elsewhere. Particular attention is paid to
    sweeping across the full domain of the muscle curves length and velocity so
    that any numerical problems during equilibrium-solving related to the
    curves (discontinuities or perhaps areas of high curvature) are identified
    here rather than by a user.

    In addition, there is a crude test of the
    quality of the equilibrium to see if the equilibrate method is trying
    to distribute the velocity of the path between the tendon and the fiber.
    This test is important because using a simple equilibrate method where
    the fiber or tendon velocity is static results in large muscle state
    transients at the beginning of of the simulation.

    @param minNormFiberLengthAlongTendon (recommend default : 0.)
      The desired normalized length of the fiber along the tendon at the
      shortest path tested.

    @param maxNormFiberLengthAlongTendon (recommend default : 2.)
        (Identical meaning as minNormFiberLengthAlongTendon but at the maximum
        path length tested.)

    @param minNormTendonStrain (recommend default : -0.5)
        The desired minimum normalized tendon strain at the shortest path length
        tested. For example if a value of -2.3 is used here it means that the
        shortest tendon length will be close to ltSlk*(1 + etIso*(-2.3)) where
        ltSlk is the tendon slack length and etIso is the tendon strain at one
        norm force.

    @param maxNormTendonStrain (recommend default : 2.0)
        (Identical meaning as minNormTendonStrain but at the maximum path length
        tested)

    @param minTendonSlackLengthToFiberLengthRatio (recommended default 0.1)
        The shortest tendon slack length tested will adopt this
        minTendonSlackLengthToFiberLengthRatio

    @param maxTendonSlackLengthToFiberLengthRatio (recommended default 20)
        The longest tendon slack length tested will adopt this
        maxTendonSlackLengthToFiberLengthRatio. A very long length is
        recommended (like 20) since some initialization failures (e.g. pairing
        a muscle model that does not have C2 continuous curves with a
        Newton method) are unlikely to be caught with shorter tendons.

    @param numberOfTendonSlackLengthsToTest (recommended default 3)
        The number of tendon slack lengths to test between
        minTendonSlackLengthToFiberLengthRatio and
        maxTendonSlackLengthToFiberLengthRatio.

        Note: Tendon slack lengths between
        minTendonSlackLengthToFiberLengthRatio and
        maxTendonSlackLengthToFiberLengthRatio are spaced exponentially
        (using base 10). For example if

        - the minimum value is 0.1
        - the maximum is 20
        - and 3 samples are taken

        then the middle value will be 1.4, not 10.05. This
        spacing is intended to meet two aims:

        1. To cover the range of tendon-slack-length-to-fiber-length ratios
           that typically occur in most living creatures: 0.1-10.

        2. To also cover some extreme lengths that are useful to trigger certain
           initialization problems. For example if the muscle model has curves
           that are not C2 continuous (and this has not been handled by the
           initialization routine) it is easiest to observe an initialization
           failure with a very long tendon. Briefly, a very long tendon will
           lead to a larger Newton step. If a discontinuity exists, this large
           step is more likely to lead to cycling over the disconinuity without
           convergence. Without a long tendon, this type of failure is unlikely
           to be observed as it will depend on having a very specific initial
           condition.

    @param minPennationAngle (recommended default 0.)
        The minimum pennation-angle-at-the-optimal-fiber-length that will be
        tested which should be zero.

    @param maxPennationAngle (recommended default M_PI/16)
        The maximum pennation-angle-angle-at-the-optimal-fiber-length that
        will tested. This should be a non-zero value that s small enough such
        that
        optimalFiberLength*sin(maxPennationAngle) < 0.4 * optimalFiberLength
        : this will ensure that the shortest parts of the active-force-length
        curve are all touched in the equilibration parameter sweep.

    @param numberOfPennationAnglesToTest (recommended default 2)
        The number of pennation angles to test between minPennationAngle
        and maxPennationAngle. In principle only a pennation angle of zero
        and some non-zero pennation angle need to be tested: this will ensure
        that both branches of the pennation code (typically there is a branch
        for a pennation angle of 0, and a branch for a non-zero pennation angle)
        get touched during the sampling

    @param normSpeedScaling (recommended default : 2.0)
        This scales the path speed tested which covers the range from
        -maxPathSpeed to maxPathSpeed where

        maxPathSpeed = normSpeedScaling
                        * optimalFiberLength
                        * maximumContactionVelocity
                        * tendonSpeedScaling

        For the interested reader:

        The parameter tendonSpeedScaling is an internally calculated scale factor
        that is set to max( tendonSlackLength/optimalFiberLength, 1). This extra
        scale factor is in place so that, no matter the muscle architecture, the
        fiber velocity will be sampled across its full range. If this still
        isn't clear, here is a bit more detail:

        Since the path speed distribution between fiber and tendon should vary
        as a function of muscle architecture (where longer tendons yield slower
        fiber velocities) the scale factor tendonSpeedScaling is added with the
        intent of ensuring the test samples across the minimum and maximum range
        of fiber velocities. Note that we cannot calculate the precise path
        speed required for a given target fiber speed because this calculation
        is an equilibrium problem.

    @param activationSamplePoints (recommended default : 10)
        The number of activation samples to make between the minimum and maximum
        activations. Samples are made linearly. In the very least the minimum
        and maximum values should be tested.

    @param pathLengthSamplePoints (recommended default : 100)
        The number of path length samples to make between the minimum and
        maximum values. Since the equilibrium routine is most
        dependent on the force-length properties of the muscle model it is
        critical to have a dense sampling here: if possible increase this value
        as much as possible.

    @param pathSpeedSamplePoints (recommended default : 11)
        The number of path speed samples to make between the minimum and maximum
        path speeds. Here I recommend using an odd number so that the path speed
        of '0' is included in the values tested. Since good equilibrium methods
        should distribute the path speed between the fiber and tendon in some
        way it is a good idea to test a range of speeds here. At the time of
        this documents writing none of the equilibrium methods depend on the
        continuity of the force-velocity curve: increasing the number of
        sampling points here is probably not of great value.

    @param tolerance (recommended default : 1e-6)
        The numerical tolerance applied to the tests within the function
        runMuscleEquilibriumStateSweep.

    @param rejectStaticSolution (recommended default: true)

        This is a crude test of the quality of the equilbrium solution: an
        equilibrium of poor quality is one that results in a large muscle
        state or force transient at the beginning of the simulation. This
        typically happens when no effort is made to distribute the path
        velocity between the fiber and the tendon.

        If this flag is set to true, and every single sweep results in a
        fiber, or tendon, velocity that is less than
        staticSolutionNormVelocityTolerance an exception will be raised. If
        at least one of the equilibrium attempts produces a fiber, or tendon,
        velocity that is greater than staticSolutionNormVelocityTolerance then
        this test will pass. This is why it is crude: now that you know how
        this works do not cheat and set the fiber velocity by default to some
        small value slightly larger than staticSolutionNormVelocityTolerance.

    @param staticSolutionNormVelocityTolerance (recommended default 0.001)
        If the fiber velocity on a single test is equal to or less than this
        tolerance than it is considered to be static.
*/
struct EquilibriumTestSettings {

    double minNormFiberLengthAlongTendon;
    double maxNormFiberLengthAlongTendon;
    double minNormTendonStrain          ;
    double maxNormTendonStrain          ;
    double minTendonSlackLengthToFiberLengthRatio;
    double maxTendonSlackLengthToFiberLengthRatio;
    int numberOfTendonSlackLengthsToTest;
    double minPennationAngle            ;
    double maxPennationAngle            ;
    int numberOfPennationAnglesToTest   ;
    double normSpeedScaling             ;
    int activationSamplePoints          ;
    int pathLengthSamplePoints          ;
    int pathSpeedSamplePoints           ;
    double tolerance                    ;
    bool rejectStaticSolution           ;
    double staticSolutionNormVelocityTolerance;

    EquilibriumTestSettings():
        minNormFiberLengthAlongTendon(0),
        maxNormFiberLengthAlongTendon(2.0),
        minNormTendonStrain(-0.5),
        maxNormTendonStrain(2.0),
        minTendonSlackLengthToFiberLengthRatio(0.1),
        maxTendonSlackLengthToFiberLengthRatio(20),
        numberOfTendonSlackLengthsToTest(3),
        minPennationAngle(0.),
        maxPennationAngle(M_PI/16),
        numberOfPennationAnglesToTest(2),
        normSpeedScaling(2.0),
        activationSamplePoints(10),
        pathLengthSamplePoints(10),
        pathSpeedSamplePoints(11),
        tolerance(1e-6),
        rejectStaticSolution(true),
        staticSolutionNormVelocityTolerance(0.001){}

};

/*
@author M.Millard
@date April 2020

Similar to simulateMuscle this function creates a test model that consists of a
ball on a slider joint with a single muscle spanning between ground and the
ball. In contrast to simulateMuscle this function does not simulate the muscle
but instead equilibrates the muscle with a dense sampling across the states a
muscle of a muscle: activation,length, and velocity. This dense sweep is
configured by default to sample the full domain of the active-force-length,
passive-force-length, tendon-force-length, and force-velocity curves.

The intent of this dense sweep is to ensure that any
equilibrium problem areas get exposed and that the equilibrium
solutions are of good quality. Equilibrium problem areas refer to problems in
the equilibrium methods of each muscle that either return a solution that does
not satisfy the desired tolerance or fails all together. In terms of quality,
equilibrium methods should find a solution that does not cause a transient
at the beginning of the simulation. At the present time this test it is checking
that the equilibrium method is making an effort to distribute the path velocity
between the tendon and the fiber. A more detailed test (left for future work)
would simulate the muscle for a short period of time to see if the fiber force
or velocity is rapidly changing.

This test has been written after equilibrium problems were noted with the
Thelen2003Muscle model at a normalized fiber length of 1.0: at this length the
passive-force-length curve has a C1 discontinuity which makes the Newton method
(used in the initial implementation) fail to converge. It is worth noting that
the Newton method fails only when two criteria are met:

- The true solution lies very close, a distance d0, to a discontinuity
- The Newton method takes steps, of distance d1, that are relatively large
  (d1 >> d0) such that the method ends pathologically jumping from one
  side of the discontiuity to another.

Thus far it has only been possible to reach these conditions with a muscle that
has a very long tendon: other wise the gradient is too big, the Newton steps
are too small, and the routine converges.

@param aMuscle  a path actuator
@param minActivation the minimum activation to test
@param maxActivation the maximum activation to test
@param tendonStrainAtOneNormForce: the Cauchy strain of the tendon at one norm
       force (usually around 0.04).
*/
void runMuscleEquilibriumStateSweep(const Muscle &aMuscleModel,
                                  double minActivation,
                                  double maxActivation,
                                  double tendonStrainAtOneNormForce,
                                  const EquilibriumTestSettings& settings={});

/**
This function serves as an outer loop to runMuscleEquilibriumStateSweep varying
parameters of muscle architecture prior to running
runMuscleEquilibriumStateSweep. At the present time the parameters tendon slack
length and pennation angle are varied: tendon slack is known to have an effect
on the convergence properties of muscle-equilibrium methods; pennation is varied
simply for completeness.

@param aMuscle  a path actuator
@param minActivation the minimum activation to test
@param maxActivation the maximum activation to test
@param tendonStrainAtOneNormForce: the Cauchy strain of the tendon at one norm
       force (usually around 0.04).

*/
void runMuscleEquilibriumArchitectureAndStateSweep(const Muscle &aMuscleModel,
                                  double minActivation,
                                  double maxActivation,
                                  double tendonStrainAtOneNormForce,
                                  const EquilibriumTestSettings& settings={});


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
        const Muscle &aMuscleModel,
        double startX, 
        double act0, 
        const Function *motion,  // prescribe motion of free end of muscle
        const Function *control, // prescribed excitation signal to the muscle
        bool printResults)
{
    string prescribed = (motion == NULL) ? "." : " with Prescribed Motion.";

    cout << "\n******************************************************" << endl;
    cout << "Test " << aMuscleModel.getConcreteClassName()
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

    double optimalFiberLength = aMuscleModel.getOptimalFiberLength();
    double pennationAngle     = aMuscleModel.getPennationAngleAtOptimalFiberLength();
    double tendonSlackLength  = aMuscleModel.getTendonSlackLength();

    // Use a copy of the muscle model passed in to add path points later
    Muscle *aMuscle(aMuscleModel.clone());

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
    
    std::unique_ptr<ActivationFiberLengthMuscle_Deprecated> aflMuscle(
        dynamic_cast<ActivationFiberLengthMuscle_Deprecated *>(
          aMuscle));

    if(aflMuscle){
        // Define the default states for the muscle that has 
        //activation and fiber-length states
        aflMuscle->setDefaultActivation(act0);
        aflMuscle->setDefaultFiberLength(aflMuscle->getOptimalFiberLength());
    }else{
        std::unique_ptr< ActivationFiberLengthMuscle> aflMuscle2(
            dynamic_cast<ActivationFiberLengthMuscle *>(aMuscle));
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

    // Equilibrate the system and get the default state
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
    
    ASSERT_EQUAL(length/trueLength, 1.0, EquilibriumTestTolerance,
                __FILE__, __LINE__,
                "testMuscles: path failed to equilibrate to correct length." );

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
                                MaxIsometricForce,
                                OptimalFiberLength,
                                TendonSlackLengthMedium,
                                PennationAngleZero);

    double x0 = 0;
    double act0 = 0.5;
    Constant control(act0);

    Sine motion(OptimalFiberLength, 2.0*SimTK::Pi, 0);

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
                                        MaxIsometricForce,
                                        OptimalFiberLength,
                                        TendonSlackLengthMedium,
                                        PennationAngleZero);

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
    if(testEquilibriumThelen2003Muscle){

        Thelen2003Muscle muscleInit("muscle",
                                MaxIsometricForce,
                                OptimalFiberLength,
                                TendonSlackLengthShort,
                                PennationAngleSmall);

        double etIso  = muscleInit.get_FmaxTendonStrain();
        double minAct = muscleInit.get_minimum_activation();
        double maxAct = muscleInit.get_max_control();

        runMuscleEquilibriumArchitectureAndStateSweep(muscleInit,minAct,maxAct,
                                                      etIso);
    }else{
      cout << "\n******************************************************"<< endl;
      cout << "Skipping: Equilibrium test Thelen2003Muscle"           << endl;
      cout << "******************************************************" << endl;
    }

    Thelen2003Muscle muscle("muscle",
                            MaxIsometricForce,
                            OptimalFiberLength,
                            TendonSlackLengthMedium,
                            PennationAngleZero);

    Thelen2003Muscle muscle1("muscle",
                            MaxIsometricForce,
                            OptimalFiberLength,
                            TendonSlackLengthMedium,
                            PennationAngleBig);


    muscle.setActivationTimeConstant(Activation0);
    muscle.setDeactivationTimeConstant(Deactivation0);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(OptimalFiberLength, 2.0*SimTK::Pi, 0);


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

    //Check set/get
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
            MaxIsometricForce, OptimalFiberLength, TendonSlackLengthMedium,
            PennationAngleZero);
        
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

    // Test exception when muscle cannot be equilibrated.
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
    if(testEquilibriumMillard2012EquilibrumMuscle){

        Millard2012EquilibriumMuscle muscleInit("muscle",
                                MaxIsometricForce,
                                OptimalFiberLength,
                                TendonSlackLengthShort,
                                PennationAngleSmall);

        muscleInit.setMuscleConfiguration(false,false,0.01);
        muscleInit.setMinimumActivation(0.);


        double etIso=muscleInit.getTendonForceLengthCurve()
                               .getStrainAtOneNormForce();
        double minAct = 0.;
        double maxAct = 1.;

        runMuscleEquilibriumArchitectureAndStateSweep(muscleInit,minAct,maxAct,
                                                      etIso);

    }else{
      cout << "\n******************************************************"<<endl;
      cout << "Skipping: Equilibrium test Millard2012EquilibriumMuscle" <<endl;
      cout << "******************************************************"  <<endl;
    }

    Millard2012EquilibriumMuscle muscle("muscle",
                            MaxIsometricForce,
                            OptimalFiberLength,
                            TendonSlackLengthMedium,
                            PennationAngleZero);

    muscle.setActivationTimeConstant(Activation0);
    muscle.setDeactivationTimeConstant(Deactivation0);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(2.0*OptimalFiberLength, 2*SimTK::Pi, 0);

    simulateMuscle(muscle, 
        x0, 
        act0, 
        &motion, 
        &control,
        false);




    // Test extremely short fiber where muscle should still equilibrate.
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

    // Test extremely short (but stretched) fiber, which should still
    // equilibrate.
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
    if(testEquilibriumMillard2012AccelerationMuscle){

        Millard2012AccelerationMuscle muscleInit("muscle",
                                MaxIsometricForce,
                                OptimalFiberLength,
                                TendonSlackLengthShort,
                                PennationAngleSmall);

        double etIso=muscleInit.getTendonForceLengthCurve()
                               .getStrainAtOneNormForce();
        double minAct = 0.;
        double maxAct = 1.;

        runMuscleEquilibriumArchitectureAndStateSweep(muscleInit,minAct,maxAct,
                                                     etIso);


    }else{
        cout <<"\n******************************************************"<<endl;
        cout <<"Skipping: Equilibrium test Thelen2003Muscle"           << endl;
        cout <<"******************************************************" <<endl;
    }
    Millard2012AccelerationMuscle muscle("muscle",
                            MaxIsometricForce,
                            OptimalFiberLength,
                            TendonSlackLengthMedium,
                            PennationAngleZero);

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

void testSchutte1993Muscle()
{
    Schutte1993Muscle_Deprecated muscle("muscle",
                                        MaxIsometricForce,
                                        OptimalFiberLength,
                                        TendonSlackLengthMedium,
                                        PennationAngleZero);

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
                                    MaxIsometricForce,
                                    OptimalFiberLength,
                                    TendonSlackLengthMedium,
                                    PennationAngleZero);

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

void runMuscleEquilibriumArchitectureAndStateSweep(const Muscle &aMuscleModel,
                                  double minActivation,
                                  double maxActivation,
                                  double tendonStrainAtOneNormForce,
                                  const EquilibriumTestSettings& settings)
{
    // Use a copy of the muscle model passed in to add path points later
    std::unique_ptr<Muscle> aMuscle(aMuscleModel.clone());

    double optimalFiberLength = aMuscle->getOptimalFiberLength();

    double ltSlk, ltSlkNormExp, ltSlkNormMinExp, ltSlkNormMaxExp,
           ltSlkNormDeltaExp;

    ltSlkNormDeltaExp = 0.;

    //An exponential spacing is used for tendon slack lengths so that
    //very short, medium, and very long tendons can be tested without
    //an enormous number of samples. It is necessary to test such a huge
    //range of tendon slack lengths to both cover a range of tendon lengths
    //typical of living creatures ( with tendon-slack-length-to-fiber-length
    //ratios ranging from 0.1 to 10) and also to trigger certain numerical
    //problems. For example if C2 discontinuity is broken in one of the muscle
    //curves you are unlikely to see the initialization routine fail unless
    //you are using a very long tendon or are very lucky.
    ltSlkNormMinExp=std::log10(settings.minTendonSlackLengthToFiberLengthRatio);
    ltSlkNormMaxExp=std::log10(settings.maxTendonSlackLengthToFiberLengthRatio);

    assert(settings.numberOfTendonSlackLengthsToTest > 0);
    if(settings.numberOfTendonSlackLengthsToTest > 1){

        ltSlkNormDeltaExp = (ltSlkNormMaxExp-ltSlkNormMinExp
                             )/(settings.numberOfTendonSlackLengthsToTest-1);
    }

    double pennationAngle, pennationAngleDelta;
    pennationAngleDelta = 0.;

    assert(settings.numberOfPennationAnglesToTest > 0);
    if(settings.numberOfPennationAnglesToTest > 1){
        pennationAngleDelta =( settings.maxPennationAngle
                              -settings.minPennationAngle
                              )/(settings.numberOfPennationAnglesToTest-1);
    }

    for(int i=0; i<settings.numberOfTendonSlackLengthsToTest;++i){

        ltSlkNormExp = std::pow( 10., ltSlkNormMinExp + i*ltSlkNormDeltaExp);

        ltSlk = ltSlkNormExp*optimalFiberLength;
        aMuscle->setTendonSlackLength(ltSlk);

        for(int j=0; j<settings.numberOfPennationAnglesToTest;++j){
            pennationAngle = settings.minPennationAngle +j*pennationAngleDelta;
            aMuscle->setPennationAngleAtOptimalFiberLength(pennationAngle);

            runMuscleEquilibriumStateSweep(*aMuscle.get(),minActivation,
                maxActivation,tendonStrainAtOneNormForce,settings);
        }
    }

}

void runMuscleEquilibriumStateSweep(const Muscle &aMuscleModel,
                                  double minActivation,
                                  double maxActivation,
                                  double etIso,
                                  const EquilibriumTestSettings& settings)
{

    cout << "\n******************************************************" << endl;
    cout << "Test Equilibrium " << aMuscleModel.getConcreteClassName() << endl;
    cout << "******************************************************" << endl;
    using SimTK::Vec3;

    //==========================================================================
    // 0. Define the sweep range
    //==========================================================================

    double optimalFiberLength =aMuscleModel.getOptimalFiberLength();
    double pennationAngle     =aMuscleModel.getPennationAngleAtOptimalFiberLength();
    double tendonSlackLength  =aMuscleModel.getTendonSlackLength();
    double maxNormContractionVelocity = aMuscleModel.getMaxContractionVelocity();
    //==========================================================================
    // 0a. Activation sweep settings
    //==========================================================================

    //==========================================================================
    // 0b. Path length sweep settings
    //==========================================================================
    //This assumes a particular pennation model: I have to live with this for
    //now because, in general, there is no base class for pennation models.
    //As of April 2020 all pennation models in OpenSim are identical to
    //the class MuscleFixedWidthPennationModel which assumes that the
    //pennation height h is constant
    //
    //    h = lce*sin(alpha) = lceOpt*sin(alphaOpt).
    //
    //Note lce and alpha are the fiber length and pennation angle (which
    //allowed to vary). The variables lceOpt and alphaOpt are the optimal fiber
    //length and the pennation-angle-at-the-optimal-fiber-length and these
    //are fixed.

    double pennatedHeight = optimalFiberLength*sin(pennationAngle);

    double minFiberLengthAT   =
        settings.minNormFiberLengthAlongTendon*optimalFiberLength;
    double maxFiberLengthAT =
        settings.maxNormFiberLengthAlongTendon*optimalFiberLength;

    double minFiberLength   = std::sqrt(pennatedHeight*pennatedHeight
                                        +minFiberLengthAT*minFiberLengthAT);
    double maxFiberLength   = std::sqrt(pennatedHeight*pennatedHeight
                                        +maxFiberLengthAT*maxFiberLengthAT);

    double minTendonLength =
        tendonSlackLength*(1+settings.minNormTendonStrain*etIso);

    double maxTendonLength =
        tendonSlackLength*(1+settings.maxNormTendonStrain*etIso);

    double minPathLength = minFiberLengthAT + minTendonLength;
    double maxPathLength = maxFiberLengthAT + maxTendonLength;
    double pathLengthTestInput = 0.5*(minPathLength+maxPathLength);

    //==========================================================================
    // 0c. Path velocity sweep settings
    //==========================================================================
    //We want the fiber to experience -vmax to vmax. Because the tendon will
    //end up taking some of this velocity we will extend the range for the
    //entire path
    double vmax = optimalFiberLength*maxNormContractionVelocity;
    double tendonToFiberLengthRatio = tendonSlackLength/optimalFiberLength;

    double tendonSpeedScaling = vmax*std::max(1.0,tendonToFiberLengthRatio);

    double minPathSpeed    = -tendonSpeedScaling*settings.normSpeedScaling;
    double maxPathSpeed    =  tendonSpeedScaling*settings.normSpeedScaling;
    double pathSpeedTestInput =  maxPathSpeed*(4.0/11.);

    //==========================================================================
    // 1. Create the multibody model
    //==========================================================================

    // Create an OpenSim model
    Model model;

    //Physical properties of the model
    double ballMass = 10;
    double ballRadius = 0.05;

    // Use a copy of the muscle model passed in to add path points later
    Muscle *aMuscle(aMuscleModel.clone());
    //std::unique_ptr<Muscle> aMuscle(aMuscleModel.clone());

    // Get a reference to the model's ground body
    Ground& ground = model.updGround();

    OpenSim::Body * ball = new OpenSim::Body("ball",
                        ballMass,
                        Vec3(0),
                        ballMass*SimTK::Inertia::sphere(ballRadius));

    ball->attachGeometry(new Sphere(ballRadius));

    SliderJoint* slider = new SliderJoint( "slider",
                        ground,
                        Vec3(0),
                        Vec3(0),
                        *ball,
                        Vec3(0),
                        Vec3(0));

    auto& sliderCoord = slider->updCoordinate();
    sliderCoord.setName("tx");
    sliderCoord.setDefaultValue(pathLengthTestInput);
    sliderCoord.setDefaultSpeedValue(pathSpeedTestInput);

    model.addBody(ball);
    model.addJoint(slider);

//  ==========================================================================
//   1. Add the muscle
//  ==========================================================================

    //Attach the muscle
    const string &actuatorType = aMuscle->getConcreteClassName();
    aMuscle->setName("muscle");
    aMuscle->addNewPathPoint("muscle-box", ground, Vec3(0));
    aMuscle->addNewPathPoint("muscle-ball", *ball, Vec3(0));

    model.addForce(aMuscle);
    //model.addForce(aMuscle.get());
    model.finalizeConnections();

//  ==========================================================================
//   3. Check to see that the system is set up correctly
//  ==========================================================================

    //Set the path length, velocity, and activation states
    sliderCoord.setDefaultValue(pathLengthTestInput);
    sliderCoord.setDefaultSpeedValue(pathSpeedTestInput);
    SimTK::State& si = model.initSystem();
    model.getMultibodySystem().realize(si,SimTK::Stage::Dynamics);

    //Set activationTestInput to some value that's different from the
    //current setting
    double activationTestInput = aMuscle->getActivation(si);
    if(activationTestInput > 0.5 && activationTestInput <=1){
        activationTestInput -= 0.1;
    }else{
        activationTestInput += 0.1;
    }
    aMuscle->setActivation(si,activationTestInput);

    //Solve for a equilibrium
    model.getMultibodySystem().realize(si,SimTK::Stage::Dynamics);
    model.equilibrateMuscles(si);


    //Now check that the settings of the test (path length, speed, and
    //activation were correctly copied over and applied to the test model
    double pathLength = aMuscle->getLength(si);
    double pathSpeed  = aMuscle->getSpeed(si);
    double activation = aMuscle->getActivation(si);

    ASSERT_EQUAL(pathLength, pathLengthTestInput, settings.tolerance,
      __FILE__, __LINE__,
      "runMuscleEquilibriumStateSweep: path length failed to equilibrate." );
    ASSERT_EQUAL(pathSpeed, pathSpeedTestInput, settings.tolerance,
      __FILE__, __LINE__,
      "runMuscleEquilibriumStateSweep: path velocity failed to equilibrate." );
    ASSERT_EQUAL(activation, activationTestInput, settings.tolerance,
      __FILE__, __LINE__,
      "runMuscleEquilibriumStateSweep: activation failed to equilibrate." );

    //==========================================================================
    // 4. Perform Equilibrium Sweep
    //==========================================================================

    double activationSetting = 0.;
    double pathLengthSetting = 0.;
    double pathSpeedSetting  = 0.;

    double activationDelta  = 0.;
    if(settings.activationSamplePoints > 1){
        activationDelta = (maxActivation-minActivation)
                          /(settings.activationSamplePoints-1);
    }

    double lengthDelta = 0.;
    if(settings.pathLengthSamplePoints > 1){
        lengthDelta      = (maxPathLength-minPathLength)
                           /(settings.pathLengthSamplePoints-1);
    }

    double velocityDelta    =0;
    if(settings.pathSpeedSamplePoints > 1){
        velocityDelta = (maxPathSpeed-minPathSpeed)
                        /(settings.pathSpeedSamplePoints-1);
    }

    double tendonForce = 0.;
    double fiberForce = 0.;

    //The range of the fiber length and speed tested is relevant, varies with
    //the tendon length, and is not under the direct control of a generic test
    //routine like this.
    //
    //If the test passes this range information is printed to the screen
    //so that who ever is working with this code can make a judgement call
    //if enough of the fiber length and speed range has been tested.
    double minFiberLengthTested =  SimTK::Infinity;
    double maxFiberLengthTested = -SimTK::Infinity;
    double minFiberSpeedTested  =  SimTK::Infinity;
    double maxFiberSpeedTested  = -SimTK::Infinity;


    //Set to false at the first occurance of a solution with a non-zero
    //fiber/tendon velocity.
    bool isTendonSolutionStatic = true;
    bool isFiberSolutionStatic = true;


    for(int i=0; i<settings.activationSamplePoints; ++i){
        activationSetting = minActivation + i*activationDelta;

        double fiberLength              = 0.;
        double fiberLengthAlongTendon   = 0.;
        double fiberSpeed               = 0.;
        double normFiberSpeed           = 0.;
        double fiberSpeedAlongTendon    = 0.;
        double tendonLength             = 0.;
        double tendonSpeed              = 0.;
        double normTendonSpeed          = 0.;

        double mtLength = 0;
        double mtSpeed  = 0;


        for(int j=0; j<settings.pathLengthSamplePoints; ++j){
            pathLengthSetting = minPathLength + j*lengthDelta;

            for(int k=0; k<settings.pathSpeedSamplePoints; ++k){
                pathSpeedSetting = minPathSpeed + k*velocityDelta;

                //Set the path length, path speed, and activation
                sliderCoord.setDefaultValue(pathLengthSetting);
                sliderCoord.setDefaultSpeedValue(pathSpeedSetting);
                si = model.initSystem();
                aMuscle->setActivation(si,activationSetting);
                model.getMultibodySystem().realize(si,SimTK::Stage::Dynamics);

                //Solve the for equilibrium
                //  This is the function that this entire test is focused on.
                model.equilibrateMuscles(si);

                //Check to make sure the test conditions were met
                pathLength     = aMuscle->getLength(si);
                pathSpeed      = aMuscle->getSpeed(si);
                activation     = aMuscle->getActivation(si);

                ASSERT_EQUAL(pathLength, pathLengthSetting, SimTK::Eps,
                  __FILE__, __LINE__,"Desired path length failed to be set." );
                ASSERT_EQUAL(pathSpeed,pathSpeedSetting,    SimTK::Eps,
                  __FILE__, __LINE__,"Desired path speed failed to be set." );
                ASSERT_EQUAL(activation,activationSetting, SimTK::Eps,
                  __FILE__, __LINE__,"Desired activation failed to be set." );

                fiberLength           =aMuscle->getFiberLength(si);
                fiberLengthAlongTendon=aMuscle->getFiberLengthAlongTendon(si);
                fiberSpeed            =aMuscle->getFiberVelocity(si);
                fiberSpeedAlongTendon =aMuscle->getFiberVelocityAlongTendon(si);
                normFiberSpeed        =aMuscle->getNormalizedFiberVelocity(si);
                tendonLength          =aMuscle->getTendonLength(si);
                tendonSpeed           =aMuscle->getTendonVelocity(si);
                normTendonSpeed       =tendonSpeed/tendonSlackLength;

                mtLength = fiberLengthAlongTendon + tendonLength;
                mtSpeed  = fiberSpeedAlongTendon + tendonSpeed;

                ASSERT_EQUAL(mtLength, pathLengthSetting, settings.tolerance,
                  __FILE__, __LINE__,
                  "musculotendon length does not match path length." );
                ASSERT_EQUAL(mtSpeed,pathSpeedSetting, settings.tolerance,
                  __FILE__, __LINE__,
                  "musculotendon speed does not match path speed." );

                if(fiberLength < minFiberLengthTested){
                  minFiberLengthTested = fiberLength;
                }
                if(fiberSpeed < minFiberSpeedTested){
                  minFiberSpeedTested = fiberSpeed;
                }
                if(fiberLength > maxFiberLengthTested){
                  maxFiberLengthTested = fiberLength;
                }
                if(fiberSpeed > maxFiberSpeedTested){
                  maxFiberSpeedTested = fiberSpeed;
                }
                //Check to make sure that the force equilibrium has been
                //satisfied
                //Note: this test will still work for the
                //      Millard2012AccelerationMucle since it is equilibrated
                //      s.t. the acceleration on the mass is very very small.

                tendonForce = aMuscle->getTendonForce(si);
                fiberForce  = aMuscle->getFiberForceAlongTendon(si);


                ASSERT_EQUAL(fiberForce, tendonForce, settings.tolerance,
                             __FILE__, __LINE__, "Tendon and fiber-force-along-tendon "
                            "do not match to desired tolerance." );

                //Check to see if the fiber velocity/tendon velocity is non-zero
                //to tolerance
                if(std::fabs(normFiberSpeed)
                   >settings.staticSolutionNormVelocityTolerance){
                    isFiberSolutionStatic = false;
                }
                if(std::fabs(normTendonSpeed)
                   >settings.staticSolutionNormVelocityTolerance){
                    isTendonSolutionStatic = false;
                }

            }
        }
    }

    cout << "[" << minActivation << ", " << maxActivation << "]"
         << " : Activation Range Tested" << endl;

    cout  << "[" << (minFiberLengthTested/optimalFiberLength) << ", "
          << (maxFiberLengthTested/optimalFiberLength) << "]"
          << " : Norm. Fiber Length Range Tested" << endl;

    cout  << "[" << (minFiberSpeedTested/optimalFiberLength)
                    /maxNormContractionVelocity
          << ", "<< (maxFiberSpeedTested/optimalFiberLength)
                    /maxNormContractionVelocity
          << "] : Norm. Fiber Speed Range Tested (n.b. 1 = vmax)" << endl;

    if(settings.rejectStaticSolution){
        ASSERT_EQUAL(isFiberSolutionStatic,false, __FILE__, __LINE__,
                     "Static-fiber equilibrium is used." );
        ASSERT_EQUAL(isTendonSolutionStatic,false, __FILE__, __LINE__,
                     "Static-tendon equilibrium is used." );
    }

}


