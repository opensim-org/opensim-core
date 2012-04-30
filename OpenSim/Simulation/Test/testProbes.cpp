// testProbes.cpp
// Author:  Tim Dorn
/*
* Copyright (c) 2005-2011, Stanford University. All rights reserved. 
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
//	testProbes builds various OpenSim models using the OpenSim API and compares muscle behavior
//  for varying physical parameters (fiber-to-tendon ratio, tendon stiffness, etc...)
//
//	Models tested include:
//      1. PathActuator (Base of Muscle, is controlled tension along a GeometryPath)
//		2. RigidTendonMuscle (Stateless muscle with user-defined fiber f-l, f-v splines)
//      3. Thelen2003Muscle_Deprecated (Simm implementation)
//		4. Thelen2003MuscleV1 (Updated to correspond to the Thelen paper.)
//		4. Thelen2003Muscle (Uses the Muscle interface)
//		5. Schutte1993Muscle(_Deprecated)
//		6. Delp1990Muscle(_Deprecated)
//		
//     Add more test cases to address specific problems with probes
//
//==========================================================================================================
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Simulation/Model/PathActuator.h>
#include <OpenSim/Actuators/RigidTendonMuscle.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Simulation/Model/ActuatorPowerProbe.h>
#include <OpenSim/Simulation/Model/ForceProbe.h>
#include <OpenSim/Simulation/Model/JointPowerProbe.h>
//#include <OpenSim/Simulation/Model/MuscleMetabolicRateProbeBhargava2004.h>
//#include <OpenSim/Simulation/Model/MuscleMetabolicRateProbeUmberger2002.h>
#include <OpenSim/Simulation/Model/SystemEnergyProbe.h>
#include <OpenSim/Analyses/ProbeReporter.h>
#include <OpenSim/Analyses/ForceReporter.h>

using namespace OpenSim;
using namespace std;

//==========================================================================================================
static const double accuracy = 1e-4;

// MUSCLE CONSTANTS
static const double maxIsometricForce = 100.0, optimalFiberLength = 0.1, tendonSlackLength = 0.2, pennationAngle = 0.0;
static const double activation = 0.01, deactivation = 0.4,	activation1 = 7.6,	activation2 = 2.5;

void simulateMuscle(PathActuator &aMuscle, const double &startX, const double &act0, 
                    Function *motion, 
                    Function *control, 
                    const double &accuracy);

void testPathActuator();
void testRigidTendonMuscle();
void testThelen2003Muscle_Deprecated();
void testThelen2003Muscle();
void testSchutte1993Muscle();
void testDelp1990Muscle();
void testThelen2003MuscleV1();

int main()
{
    SimTK::Array_<std::string> failures;
    //try { testPathActuator();
        //cout << "PathActuator Test passed" << endl; }
    //catch (const Exception& e)
        //{ e.print(cerr); failures.push_back("testPathActuator"); }

    try { testRigidTendonMuscle();
        cout << "RigidTendonMuscle Test passed" << endl; }
    catch (const Exception& e)
        { e.print(cerr); failures.push_back("testRigidTendonMuscle"); }

    //try { testThelen2003Muscle_Deprecated();
        //cout << "Thelen2003Muscle_Deprecated Test passed" << endl; }
    //catch (const Exception& e)
        //{ e.print(cerr); failures.push_back("testThelen2003Muscle_Deprecated"); }

    //try { testThelen2003Muscle();
        //cout << "Thelen2003Muscle Test passed" << endl; }
    //catch (const Exception& e)
        //{ e.print(cerr); failures.push_back("testThelen2003Muscle"); }
        
    //try { testThelen2003MuscleV1();
    //	cout << "Thelen2003MuscleV1 Test passed" << endl; }
 //   catch (const Exception& e)
    //	{ e.print(cerr); failures.push_back("testThelen2003MuscleV1"); }
    
    /*
    try { testSchutte1993Muscle();
        cout << "Schutte1993Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
        { e.print(cerr); failures.push_back("testSchutte1993Muscle"); }

    try { testDelp1990Muscle();
        cout << "Delp1990Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
        { e.print(cerr); failures.push_back("testDelp1990Muscle"); }
    */

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "testProbes Done" << endl;
    return 0;
}

//==========================================================================================================
// Main test driver to be used on any muscle model (derived from Muscle) so new cases should be easy to add
// Currently, the test only verifies that the work done by the muscle corresponds to the change in system
// energy.
//
// TODO: Test will fail wih prescribe motion until the work done by this constraint is accounted for.
//==========================================================================================================
void simulateMuscle(PathActuator &aMuscle, const double &startX, const double &act0, 
                    Function *motion,  // prescribe motion of free end of muscle
                    Function *control, // prescribed excitation signal to the muscle
                    const double &accuracy)
{
    cout << "\n******************************************************" << endl;
    cout << " Test " << aMuscle.getConcreteClassName() << " Muscle Actuator Type." <<endl;
    cout << "******************************************************" << endl;
    using SimTK::Vec3;

    // Define the initial and final simulation times
    double initialTime = 0.0;
    double finalTime = 1.0;
    
    //Physical properties of the model
    double ballMass = 10;
    double ballRadius = 0.05;
    double anchorWidth = 0.1;

    // Create an OpenSim model and set its name
    Model model;

    // Get a reference to the model's ground body
    Body& ground = model.getGroundBody();
    ground.addDisplayGeometry("box.vtp");
    ground.updDisplayer()->setScaleFactors(Vec3(anchorWidth, anchorWidth, 2*anchorWidth));

    OpenSim::Body ball("ball", ballMass , Vec3(0),  ballMass*SimTK::Inertia::sphere(ballRadius));
    ball.addDisplayGeometry("sphere.vtp");
    ball.updDisplayer()->setScaleFactors(Vec3(2*ballRadius));
    // ball connected  to ground via a slider along X
    double xSinG = optimalFiberLength*cos(pennationAngle)+tendonSlackLength;
    SliderJoint slider("slider", ground, Vec3(anchorWidth/2+xSinG, 0, 0), Vec3(0), ball, Vec3(0), Vec3(0));
    CoordinateSet& jointCoordinateSet = slider.getCoordinateSet();
    jointCoordinateSet[0].setName("tx");
    jointCoordinateSet[0].setDefaultValue(1.0);
    jointCoordinateSet[0].setRangeMin(0); jointCoordinateSet[0].setRangeMax(1.0);
    if(motion != NULL)
        jointCoordinateSet[0].setPrescribedFunction(*motion);
    // add ball to model
    model.addBody(&ball);

    //Attach the muscle
    const string &actuatorType = aMuscle.getConcreteClassName();
    aMuscle.setName("muscle");
    aMuscle.addNewPathPoint("muscle-box", ground, Vec3(anchorWidth/2,0,0));
    aMuscle.addNewPathPoint("muscle-ball", ball, Vec3(-ballRadius,0,0));
    
    ActivationFiberLengthMuscle_Deprecated *aflMuscle = dynamic_cast<ActivationFiberLengthMuscle_Deprecated *>(&aMuscle);
    if(aflMuscle){
        // Define the default states for the muscle that has activation and fiber-length states
        aflMuscle->setDefaultActivation(act0);
        aflMuscle->setDefaultFiberLength(aflMuscle->getOptimalFiberLength());
    }else{
        ActivationFiberLengthMuscle *aflMuscle2 = dynamic_cast<ActivationFiberLengthMuscle *>(&aMuscle);
        if(aflMuscle2){
            // Define the default states for the muscle that has activation and fiber-length states
            aflMuscle2->setDefaultActivation(act0);
            aflMuscle2->setDefaultFiberLength(aflMuscle2->getOptimalFiberLength());
        }
    }


    model.addForce(&aMuscle);

    // Create a prescribed controller that simply applies controls as function of time
    PrescribedController muscleController;
    if(control != NULL){
        muscleController.setActuators(model.updActuators());
        // Set the indiviudal muscle control functions for the prescribed muscle controller
        muscleController.prescribeControlForActuator("muscle", control->clone());

        // Add the control set controller to the model
        model.addController(&muscleController);
    }


    // --------------------------------------------------------------------
    // ADD PROBES TO THE MODEL TO GET INTERESTING VALUES
    // --------------------------------------------------------------------
    Array<string> muscNames;
    muscNames.append(aMuscle.getName());

    Array<string> jointNames;
    jointNames.append("slider");

    // Add ActuatorPowerProbe to measure work done by the muscle 
    ActuatorPowerProbe muscWorkProbe(muscNames);
    muscWorkProbe.setName("ActuatorPower");
    muscWorkProbe.setOperation("integrate");
    model.addProbe(&muscWorkProbe);

    // Add ActuatorPowerProbe to measure power generated by the muscle 
    ActuatorPowerProbe muscPowerProbe(muscWorkProbe);	// use copy constructor
    muscPowerProbe.setOperation("");
    model.addProbe(&muscPowerProbe);

    // Add JointPowerProbe to measure work done by the joint 
    JointPowerProbe jointWorkProbe(jointNames);
    jointWorkProbe.setName("JointPower");
    jointWorkProbe.setOperation("integrate");
    model.addProbe(&jointWorkProbe);

    // Add JointPowerProbe to measure power generated by the joint 
    JointPowerProbe jointPowerProbe(jointWorkProbe);	// use copy constructor
    jointPowerProbe.setOperation("");
    model.addProbe(&jointPowerProbe);

    // Add ForceProbe to measure the impulse of the muscle force 
    ForceProbe impulseProbe(muscNames);
    impulseProbe.setName("Force");
    impulseProbe.setOperation("integrate");
    model.addProbe(&impulseProbe);

    // Add ForceProbe to report the muscle force 
    ForceProbe forceProbe(impulseProbe);			// use copy constructor
    forceProbe.setOperation("");
    model.addProbe(&forceProbe);

    // Add ForceProbe to report -3.5X the muscle force 
    double scaleFactor = -3.50;
    ForceProbe forceProbeScale(impulseProbe);		// use copy constructor
    forceProbeScale.setOperation("scale");
    forceProbeScale.setOperationParameter(scaleFactor);
    model.addProbe(&forceProbeScale);

    // Add SystemEnergyProbe to measure the system KE+PE
    SystemEnergyProbe sysEnergyProbe(true, true);
    sysEnergyProbe.setName("SystemEnergy");
    sysEnergyProbe.setOperation("");
    model.addProbe(&sysEnergyProbe);

    // Add SystemEnergyProbe to measure system power (d/dt system KE+PE)
    SystemEnergyProbe sysPowerProbe(sysEnergyProbe);	// use copy constructor
    sysPowerProbe.setOperation("differentiate");
    model.addProbe(&sysPowerProbe);

    // Add muscle metabolic probes as well
    // NEED TO WAIT UNTIL MUSCLE MODELS ARE FINISHED
    // --------------------------------------------------
    //bool addMuscleMetabolicProbes = false;
    //if(addMuscleMetabolicProbes) {
    //    
    //    MetabolicMuscle m(1.0, 0.5, 40, 133, 74, 111);
    //    m.setName(aMuscle.getName());
    //    MetabolicMuscleSet mms;
    //    mms.append(m);
    //
    //    // MuscleMetabolicRateProbeBhargava2004 Energy Rate Probe
    //    MuscleMetabolicRateProbeBhargava2004 metabolicRateProbeBhargava(true, true, true, false, true);
    //    metabolicRateProbeBhargava.setMetabolicMuscleSet(mms);
    //    metabolicRateProbeBhargava.setOperation("");
    //    
    //    // MuscleMetabolicRateProbeBhargava2004 Energy Probe
    //    MuscleMetabolicRateProbeBhargava2004 metabolicEnergyProbeBhargava(metabolicRateProbeBhargava);   // use copy constructor
    //    metabolicEnergyProbeBhargava.setOperation("integrate");
    //    
    //    // MuscleMetabolicRateProbeUmberger2002 Energy Rate Probe
    //    MuscleMetabolicRateProbeUmberger2002 metabolicRateProbeUmberger(true, true, false, true);
    //    
    //    metabolicRateProbeUmberger.setMetabolicMuscleSet(mms);
    //    metabolicRateProbeUmberger.setOperation("");
    //    
    //    // MuscleMetabolicRateProbeUmberger2002 Energy Probe
    //    MuscleMetabolicRateProbeUmberger2002 metabolicEnergyProbeUmberger(metabolicRateProbeUmberger);   // use copy constructor
    //    metabolicEnergyProbeUmberger.setOperation("integrate");
    //    
    //    cout << "ADDING MUSCLE METABOLIC PROBES" << endl;
    //    model.addProbe(&metabolicRateProbeBhargava);
    //    model.addProbe(&metabolicEnergyProbeBhargava);
    //    model.addProbe(&metabolicRateProbeUmberger);
    //    model.addProbe(&metabolicEnergyProbeUmberger);
    //    
    //}



    // --------------------------------------------------------------------
    // ADD PROBEREPORTER AND FORCEREPORTER TO RECORD VALUES
    // --------------------------------------------------------------------
    ProbeReporter* probeReporter = new ProbeReporter(&model);
    model.addAnalysis(probeReporter);
    ForceReporter* forceReporter = new ForceReporter(&model);
    model.addAnalysis(forceReporter);
    // --------------------------------------------------------------------

    // Since all components are allocated on the stack don't have model own them (and try to free)
    model.disownAllComponents();
    model.setName(actuatorType+"ModelTest");
    model.print(actuatorType+"ModelTest.osim");
    model.printBasicInfo(cout);

    // Initialize the system and get the default state
    SimTK::State& si = model.initSystem();

    // Define non-zero (defaults are 0) states for the free joint
    CoordinateSet& modelCoordinateSet = model.updCoordinateSet();
    modelCoordinateSet[0].setValue(si, startX); // set x-translation value

    // Check muscle is setup correctly 
    const PathActuator &muscle = dynamic_cast<const PathActuator&>(model.updActuators().get("muscle"));
    double length = muscle.getLength(si);
    double trueLength = startX + xSinG - anchorWidth/2;
    ASSERT_EQUAL(trueLength, length, 0.01*accuracy);
    model.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);


    // Get initial muscle and system energy
    // ---------------------------------------
    double Emuscle0 = muscWorkProbe.getRecordValues(si).get(0);
    double Esys0 = sysEnergyProbe.getRecordValues(si).get(0);
    Esys0 += (Emuscle0 + jointWorkProbe.getRecordValues(si).get(0));
    //cout << "Total initial system energy = " << Esys0 << endl; 


    // Create the integrator and manager
    // -----------------------------------
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    integrator.setAccuracy(accuracy);
    Manager manager(model, integrator);


    // Integrate from initial time to final time
    // ------------------------------------------
    manager.setInitialTime(initialTime);
    manager.setFinalTime(finalTime);
    cout<<"\nIntegrating from " << initialTime<< " to " << finalTime << endl;
    const clock_t start = clock();		// Start timing the simulation
    manager.integrate(si);				// Simulate
    double comp_time = (double)(clock()-start)/CLOCKS_PER_SEC;		// how long did it take?


    // Test system energy - work using
    // "integrate" operation on muscWorkProbe, jointWorkProbe
    // "" operation on sysEnergyProbe
    // ---------------------------------------------------------
    model.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);
    double muscleWorkAtEnd = muscWorkProbe.getRecordValues(si).get(0);
    double jointWorkAtEnd = jointWorkProbe.getRecordValues(si).get(0);
    double systemEnergyAtEnd = sysEnergyProbe.getRecordValues(si).get(0);
    double ESysMinusWork = systemEnergyAtEnd - muscleWorkAtEnd - jointWorkAtEnd; 

    cout << "Muscle work at end of simulation = " << muscleWorkAtEnd << endl;
    cout << "Joint work at end of simulation = " << jointWorkAtEnd << endl;
    cout << "System energy at end of simulation = " << systemEnergyAtEnd << endl;
    cout << "Total system energy - work = " << ESysMinusWork << endl; 
    ASSERT_EQUAL(Esys0, ESysMinusWork, 0.5*accuracy, __FILE__, __LINE__, "System energy with muscle not conserved.");


    // Test 'Scale' operator is working using
    // "" operation on forceProbe
    // "scale" operation on forceProbeScale
    ASSERT_EQUAL(forceProbeScale.getRecordValues(si).get(0), scaleFactor*forceProbe.getRecordValues(si).get(0), 0.5*accuracy, __FILE__, __LINE__, "Error with 'scale' operation.");



    // Save the simulation results (states, forces, probes)
    // -----------------------------------------------------
    Storage states(manager.getStateStorage());
    states.print(actuatorType+"_states.sto");
    probeReporter->getProbeStorage().print(actuatorType+"_probes.sto");
    forceReporter->getForceStorage().print(actuatorType+"_forces.sto");

    cout << "Done with printing results..." << endl;
    //system("pause");
    
    // Minimum requirement to pass is simulation of single muscle on slider is real-time
    //ASSERT(comp_time <= (finalTime-initialTime));
    cout << actuatorType << " simulation in " << comp_time << "s, for " << accuracy << " accuracy." << endl;
}


//==========================================================================================================
// Individudal muscle model (derived from Muscle) test cases can be added here
//==========================================================================================================
void testThelen2003Muscle_Deprecated()
{
    Thelen2003Muscle_Deprecated muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
    muscle.setActivationTimeConstant(activation);
    muscle.setDeactivationTimeConstant(deactivation);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
    // Uncomment when work done by prescribed motion constraint is accounted for.
    simulateMuscle(muscle, x0, act0, &motion, &control, accuracy);
}

void testThelen2003Muscle()
{
    Thelen2003Muscle muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
    muscle.setActivationTimeConstant(activation);
    muscle.setDeactivationTimeConstant(deactivation);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
    // Uncomment when work done by prescribed motion constraint is accounted for.
    //simulateMuscle(muscle, x0, act0, &motion, &control, accuracy);
}


void testSchutte1993Muscle()
{
    Schutte1993Muscle_Deprecated muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
    muscle.setActivation1(activation1);
    muscle.setActivation2(activation2);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}


void testDelp1990Muscle()
{
    Delp1990Muscle_Deprecated muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
    muscle.setActivation1(activation1);
    muscle.setActivation2(activation2);
    muscle.setMass(0.1);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}

void testPathActuator()
{
    double x0 = 0;
    double act0 = 0.2;
    double loadX = 50;

    PathActuator muscle;
    muscle.setOptimalForce(maxIsometricForce);

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}


void testRigidTendonMuscle()
{
    RigidTendonMuscle muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}

void testThelen2003MuscleV1()
{
    double activation = 0.01, deactivation = 0.4;

    Thelen2003MuscleV1 muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
    muscle.setActivationTimeConstant(activation);
    muscle.setDeactivationTimeConstant(deactivation);

    double x0 = 0;
    double act0 = 0.2;

    Constant control(0.5);

    Sine motion(0.1, SimTK::Pi, 0);

    simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}
