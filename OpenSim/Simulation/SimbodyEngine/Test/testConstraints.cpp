/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testConstraints.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Samuel R. Hamner                                     *
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
// testConstraints builds OpenSim models using the OpenSim API and builds an 
// equivalent Simbody system using the Simbody API for each test case. A test 
// fails if the OpenSim and Simbody final states of the simulation are not 
// equivalent (norm-err less than 10x integration error tolerance)
//
//  Tests Include:
//      1. Test locking (constraint) mechanism on coordinates
//      2. Test WelConstraint against Simbody Constraint::Weld
//      3. PointOnLineConstraint against Simbody built-in PointOnLine constraint
//      4. CoordinateCouplerConstraint as a custom knee 
//      5. RollingOnSurfaceConstraint as foot on floor against SimTK NoSlip1D
//     Add tests here as new constraint types are added to OpenSim
//
//==============================================================================
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/osimCommon.h>

#include <OpenSim/Common/FunctionAdapter.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ConstraintSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>

#include <OpenSim/Simulation/SimbodyEngine/PointConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/ConstantDistanceConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/PointOnLineConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/RollingOnSurfaceConstraint.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "SimTKsimbody.h"

using namespace OpenSim;
using namespace std;

//===============================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-4;
const static double duration = 1.00;
const static SimTK::Vec3 gravity_vec(0, -9.8065, 0);
//Thigh
const static double femurMass = 8.806;
const static SimTK::Vec3 femurCOM(0, 0.5, 0);
const static SimTK::Inertia femurInertiaAboutCOM(SimTK::Vec3(0.1268, 0.0332, 0.1337));
//Shank
const static SimTK::MassProperties tibiaMass(3.510, SimTK::Vec3(0), SimTK::Inertia(SimTK::Vec3(0.0477, 0.0048, 0.0484)));
//Foot
const static SimTK::MassProperties footMass(1.20, SimTK::Vec3(0), SimTK::Inertia(SimTK::Vec3(0.001361, 0.003709, 0.003916)));
//Toes
const static SimTK::MassProperties toesMass(0.205126, SimTK::Vec3(0), SimTK::Inertia(SimTK::Vec3(0.000117, 0.000179, 0.000119)));
// Joint locations
const static SimTK::Vec3 hipInGround(0, 0.7, 0);
const static SimTK::Vec3 hipInFemur(0.0020, 0.1715, 0);
const static SimTK::Vec3 kneeInFemur(0.0033, -0.2294, 0);
const static SimTK::Vec3 kneeInTibia(0.0, 0.1862, 0.0);
const SimTK::Vec3 ankleInTibia(0.0, -0.243800, 0);
const SimTK::Vec3 ankleInFoot(-0.035902, 0.051347, 0);
const SimTK::Vec3 mtpInFoot(0.098032, -0.038000, 0);
const SimTK::Vec3 mtpInToes(-0.035902, 0.051347, 0);
//================================================================================

class MultidimensionalFunction : public OpenSim::Function {
OpenSim_DECLARE_CONCRETE_OBJECT(MultidimensionalFunction, OpenSim::Function);

public:
    MultidimensionalFunction() {};
    virtual ~MultidimensionalFunction() {};

    double calcValue(const SimTK::Vector& x) const override
    {
        return 2*x[0]*x[0] + x[1];
    }
    double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const override
    {
       int nd = (int)derivComponents.size();
       if (nd < 1)
           return SimTK::NaN;
       if (derivComponents[0] == 0){
            if (nd == 1)
                return 4*x[0];
            else if(derivComponents[0] == 0)
                return 4;
       }
       else if (derivComponents[0] == 1){
           if (nd == 1)
               return 1;
       }
       return 0;
    }
    int getArgumentSize() const override {return 2;}
    int getMaxDerivativeOrder() const override { return 2;}
    SimTK::Function* createSimTKFunction() const override
    {
        return new FunctionAdapter(*this);
    }
}; // End of MultidimensionalFunction

void testRollingOnSurfaceConstraint();
void testCoordinateLocking();
void testWeldConstraint();
void testPointOnLineConstraint();
void testCoordinateCouplerConstraint();
void testPointConstraint();
void testConstantDistanceConstraint();
void testSerializeDeserialize();

int main()
{
    try {
        testPointConstraint();
        testConstantDistanceConstraint();
        testCoordinateLocking();
        testWeldConstraint();
        // Compare behavior of PointOnLineConstraint between the foot and ground
        testPointOnLineConstraint();
        // Compare behavior of CoordinateCouplerConstraint as a custom knee
        testCoordinateCouplerConstraint();
        // test OpenSim roll constraint against a composite of Simbody constraints
        testRollingOnSurfaceConstraint();
        testSerializeDeserialize();
    }
    catch(const OpenSim::Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

//==============================================================================
// Common Functions 
//==============================================================================
int initTestStates(SimTK::Vector &qi, SimTK::Vector &ui)
{
    using namespace SimTK;

    Random::Uniform randomAngle(-Pi/4, Pi/4);
    Random::Uniform randomSpeed(-1.0, 1.0);

    // Provide initial states as random angles and speeds for OpenSim and 
    // Simbody models
    for(int i = 0; i<qi.size(); i++)
        qi[i] = randomAngle.getValue();

    for(int i = 0; i<ui.size(); i++)
        ui[i] = randomSpeed.getValue();
    
    return qi.size();
}

void integrateSimbodySystem(SimTK::MultibodySystem &system, SimTK::State &state)
{
    using namespace SimTK;

    // realize simbody system to velocity stage
    system.realize(state, Stage::Velocity);

    RungeKuttaMersonIntegrator integ(system);
    integ.setAccuracy(integ_accuracy);
    
    TimeStepper ts(system, integ);
    ts.initialize(state);
    ts.stepTo(duration);
    state = ts.getState();
}

void integrateOpenSimModel(Model *osimModel, SimTK::State &osim_state)
{
    using namespace SimTK;

    // SETUP OpenSim SIMULATION Manager
    osimModel->getMultibodySystem().realize(osim_state, Stage::Velocity);

    Manager manager(*osimModel);
    manager.setIntegratorAccuracy(integ_accuracy);

    manager.initialize(osim_state);
    osim_state = manager.integrate(duration);
}

void compareSimulationStates(SimTK::Vector q_sb, SimTK::Vector u_sb,
    SimTK::Vector q_osim, SimTK::Vector u_osim,
    string errorMessagePrefix = "")
{
    using namespace SimTK;
    
    Vector q_err = q_sb;
    Vector u_err = u_sb;

    int nq = q_osim.size();
    if(q_sb.size() > nq){ // we have an unused quaternion slot in Simbody

        q_sb.dump("Simbody q's:");
        q_osim.dump("OpenSim q's:");
        //This is a hack knowing the free and ball joint tests have the
        // quaternion q's first and that the q's are packed as qqqq or aaa*
        // for a ball and qqqqxyz or aaaxyz* for a free joint
        int quat_ind = ((nq > 6) ? 6 : 3);
        int j = 0;
        for(int i=0; i< q_sb.size(); i++){
            if(i != quat_ind){
                q_err[j] = q_sb[i] - q_osim[j];
                j++;
            }
        }
    }
    else{
        if(nq > q_sb.size()){ // OpenSim using constrains
            q_err[0] = q_sb[0] - q_osim[0];
            q_err[1] = q_sb[1] - q_osim[1];
            u_err[0] = u_sb[0] - u_osim[0];
            u_err[1] = u_sb[1] - u_osim[1];
        }
        else{
            q_err = q_sb - q_osim;
            u_err = u_sb - u_osim;
        }
    }

    //cout<<"\nSimbody - OpenSim:"<<endl;
    //q_err.dump("Diff q's:");
    //u_err.dump("Diff u's:");

    stringstream errorMessage1, errorMessage2;
    errorMessage1 << "testConstraints compareSimulationStates failed q_err.norm = " << q_err.norm();
    errorMessage2 << "testConstraints compareSimulationStates failed u_err.norm = " << u_err.norm();
    ASSERT(q_err.norm() <= 10*integ_accuracy, __FILE__, __LINE__, errorMessagePrefix + errorMessage1.str());
    ASSERT(u_err.norm() <= 20*integ_accuracy, __FILE__, __LINE__, errorMessagePrefix + errorMessage2.str());
}

void compareSimulations(SimTK::MultibodySystem &system, SimTK::State &state, Model *osimModel, SimTK::State &osim_state, string errorMessagePrefix = "")
{
    using namespace SimTK;

    // Set the initial states for both Simbody system and OpenSim model
    Vector& qi = state.updQ();
    Vector& ui = state.updU();
    int nq_sb = initTestStates(qi, ui);
    int nq = osim_state.getNQ();

    // Push down to OpenSim "state"
        if(nq == 2*nq_sb){ //more coordinates because OpenSim model is constrained
            osim_state.updY()[0] = state.getY()[0];
            osim_state.updY()[1] = state.getY()[1];
            osim_state.updY()[nq] = state.getY()[nq_sb];
            osim_state.updY()[nq+1] = state.getY()[nq_sb+1];
        }
        else    
            osim_state.updY() = state.getY();
    

    //==========================================================================================================
    // Integrate Simbody system
    integrateSimbodySystem(system, state);

    // Simbody model final states
    qi = state.updQ();
    ui = state.updU();

    qi.dump("\nSimbody Final q's:");
    ui.dump("\nSimbody Final u's:");

    //==========================================================================================================
    // Integrate OpenSim model
    integrateOpenSimModel(osimModel, osim_state);

    // Get the state at the end of the integration from OpenSim.
    Vector& qf = osim_state.updQ();
    Vector& uf = osim_state.updU();
    qf.dump("\nOpenSim Final q's:");
    uf.dump("\nOpenSim Final u's:");

    //==========================================================================================================
    // Compare Simulation Results
    compareSimulationStates(qi, ui, qf, uf, errorMessagePrefix);
}
//==========================================================================================================

//==========================================================================================================
// Test Cases
//==========================================================================================================

void testPointConstraint()
{
    using namespace SimTK;

    cout << endl;
    cout << "==================================================================" << endl;
    cout << " OpenSim PointConstraint vs. Simbody Constraint::Ball " << endl;
    cout << "==================================================================" << endl;

    Random::Uniform randomLocation(-1, 1);
    Vec3 pointOnFoot(randomLocation.getValue(), randomLocation.getValue(), randomLocation.getValue());
    Vec3 pointOnGround(0,0,0);

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Create a free joint between the foot and ground
    MobilizedBody::Free foot(matter.Ground(), Transform(Vec3(0)), 
        SimTK::Body::Rigid(footMass), Transform(Vec3(0)));
    
    // Constrain foot to point on ground
    SimTK::Constraint::Ball simtkBall(matter.Ground(), pointOnGround, foot, pointOnFoot);

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================================================
    // Setup OpenSim model
    Model *osimModel = new Model;
    //OpenSim bodies
    const Ground& ground = osimModel->getGround();

    //OpenSim foot
    OpenSim::Body osim_foot("foot", footMass.getMass(), footMass.getMassCenter(), footMass.getInertia());

    // create foot as a free joint
    FreeJoint footJoint("footToGround", ground, Vec3(0), Vec3(0), osim_foot, Vec3(0), Vec3(0));
    
    // Add the thigh body which now also contains the hip joint to the model
    osimModel->addBody(&osim_foot);
    osimModel->addJoint(&footJoint);

    // add a point constraint
    PointConstraint ballConstraint(ground, pointOnGround, osim_foot, pointOnFoot);
    osimModel->addConstraint(&ballConstraint);

    // BAD: have to set memoryOwner to false or program will crash when this test is complete.
    osimModel->disownAllComponents();

    osimModel->setGravity(gravity_vec);

    //Add analyses before setting up the model for simulation
    Kinematics *kinAnalysis = new Kinematics(osimModel);
    kinAnalysis->setInDegrees(false);
    osimModel->addAnalysis(kinAnalysis);

    // Need to setup model before adding an analysis since it creates the AnalysisSet
    // for the model if it does not exist.
    State& osim_state = osimModel->initSystem();

    //==========================================================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, osimModel, osim_state, "testPointConstraint FAILED\n");
}

void testConstantDistanceConstraint()
{
    using namespace SimTK;

    cout << endl;
    cout << "==================================================================" << endl;
    cout << " OpenSim ConstantDistanceConstraint vs. Simbody Constraint::Rod " << endl;
    cout << "==================================================================" << endl;

    Random::Uniform randomLocation(-1, 1);
    Vec3 pointOnFoot(randomLocation.getValue(), randomLocation.getValue(), randomLocation.getValue());
    Vec3 pointOnGround(0,0,0);
    /** for some reason, adding another Random::Uniform causes testWeldConstraint to fail.  
    Why doesn't it cause this test to fail???? */
    //Random::Uniform randomLength(0.01, 0.2);
    //randomLength.setSeed(1024);
    //double rodLength = randomLength.getValue(); 
    double rodLength = 0.05;

    //std::cout << "Random Length = " << rodLength2 << ", used length = " << rodLength << std::endl;

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Create a free joint between the foot and ground
    MobilizedBody::Free foot(matter.Ground(), Transform(Vec3(0)), 
        SimTK::Body::Rigid(footMass), Transform(Vec3(0)));
    
    // Constrain foot to point on ground
    SimTK::Constraint::Rod simtkRod(matter.Ground(), pointOnGround, foot, pointOnFoot, rodLength);


    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================================================
    // Setup OpenSim model
    Model osimModel;
    //OpenSim bodies
    const Ground& ground = osimModel.getGround();

    //OpenSim foot
    auto* osim_foot = new OpenSim::Body("foot", footMass.getMass(),
                                        footMass.getMassCenter(),
                                        footMass.getInertia());

    // create foot as a free joint
    auto* footJoint = new FreeJoint("footToGround", ground, Vec3(0), Vec3(0),
                                    *osim_foot, Vec3(0), Vec3(0));
    
    // Add the thigh body which now also contains the hip joint to the model
    osimModel.addBody(osim_foot);
    osimModel.addJoint(footJoint);

    // add a constant distance constraint
    auto* rodConstraint = new ConstantDistanceConstraint(
                ground, pointOnGround, *osim_foot, pointOnFoot, rodLength);
    osimModel.addConstraint(rodConstraint);

    osimModel.setGravity(gravity_vec);

    //Add analyses before setting up the model for simulation
    Kinematics* kinAnalysis = new Kinematics(&osimModel);
    kinAnalysis->setInDegrees(false);
    osimModel.addAnalysis(kinAnalysis);

    // Need to setup model before adding an analysis since it creates the
    // AnalysisSet for the model if it does not exist.
    State& osim_state = osimModel.initSystem();

    //=========================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, &osimModel, osim_state,
                       "testConstantDistanceConstraint FAILED\n");
}
void testCoordinateLocking()
{
    using namespace SimTK;

    cout << endl;
    cout << "==================================================================" << endl;
    cout << " OpenSim testCoordinateLocking " << endl;
    cout << "==================================================================" << endl;


    double fixedKneeAngle = Pi/2;

    // Setup OpenSim model
    Model *osimModel = new Model;
    //OpenSim bodies
    const Ground& ground = osimModel->getGround();;

    //OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

    // create hip as a pin joint
    PinJoint hip("hip",ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));
    // Rename hip coordinates for a pin joint
    hip.updCoordinate().setName("hip_flex");

    // Add the thigh body 
    osimModel->addBody(&osim_thigh);
    osimModel->addJoint(&hip);

    // Add OpenSim shank via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
        tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // create pin knee joint
    PinJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0),
                          osim_shank, Vec3(0), Vec3(0));

    // Add the shank body and knee joint
    osimModel->addBody(&osim_shank);
    osimModel->addJoint(&knee);

    // BAD: have to set memoryOwner to false or program will crash when this test is complete.
    osimModel->disownAllComponents();

    osimModel->setGravity(gravity_vec);

    // Initialize the state of the model based on the defaults of ModelComponents.
    State& si = osimModel->initSystem();

    // Model joint states
    CoordinateSet &coordinates = osimModel->updCoordinateSet();
    coordinates[0].setValue(si, 0);
    coordinates[0].setSpeedValue(si, 1.0);
    coordinates[1].setValue(si, fixedKneeAngle);
    coordinates[1].setLocked(si, true);  // lock the knee
    osimModel->setPropertiesFromState(si);

    // Test that serialization and deserialization of locking works
    osimModel->print("testLockingModel.osim");
    // Model and all its contents are wiped out
    delete osimModel;
    // Reload the model
    osimModel = new Model("testLockingModel.osim");

    // Re-initialize the state of the model based on now saved defaults of the model.
    State& si2 = osimModel->initSystem();

    osimModel->getMultibodySystem().realize(si2, Stage::Velocity );
 
    // Create the integrator and manager for the simulation.
    Manager manager(*osimModel);
    manager.setIntegratorMaximumStepSize(1.0e-3);
    manager.setIntegratorMinimumStepSize(1.0e-6);
    manager.setIntegratorAccuracy(integ_accuracy);

    // Print out the initial position and velocity states
    si2.getQ().dump("Initial q's"); // pendulum positions
    si2.getU().dump("Initial u's"); // pendulum velocities
    Vector qi = si2.getQ();

    // Integrate from initial time to final time
    si2.setTime(0.0);
    manager.initialize(si2);
    cout<<"\n\nIntegrating from "<<si2.getTime()<<" to "<<duration<<std::endl;
    si2 = manager.integrate(duration);

    // Print out the final position and velocity states
    Vector qf = si2.getQ();
    qf.dump("Final q's"); // pendulum positions
    si2.getU().dump("Final u's"); // pendulum velocities

    stringstream errorMessage;
    errorMessage << "testCoordinateLocking FAILED\ntestCoordinateLocking: q_err = " << qf[1]-qi[1];
    ASSERT(fabs(qf[1]-fixedKneeAngle) <= integ_accuracy, __FILE__, __LINE__, errorMessage.str());
}

void testWeldConstraint()
{
    using namespace SimTK;

    cout << endl;
    cout << "==================================================================" << endl;
    cout << " OpenSim WeldConstraint vs. Simbody Constraint::Weld " << endl;
    cout << "==================================================================" << endl;

    Random::Uniform randomValue(-0.05, 0.1);
    Vec3 weldInGround(randomValue.getValue(), randomValue.getValue(), 0);
    Vec3 weldInFoot(0.1*randomValue.getValue(), 0.1*randomValue.getValue(), 0);

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Thigh connected by hip
    MobilizedBody::Pin thigh(matter.Ground(), SimTK::Transform(hipInGround), 
        SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), Transform(hipInFemur));
    // Pin knee connects shank
    MobilizedBody::Pin shank(thigh, Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), Transform(kneeInTibia));
    // Pin ankle connects foot
    MobilizedBody::Pin foot(shank, Transform(ankleInTibia), SimTK::Body::Rigid(footMass), Transform(ankleInFoot));

    SimTK::Constraint::Weld weld(matter.Ground(), Transform(weldInGround), foot, Transform(weldInFoot));

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================================================
    // Setup OpenSim model
    Model *osimModel = new Model;
    //OpenSim bodies
    const Ground& ground = osimModel->getGround();;

    //OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

    // create Pin hip joint
    PinJoint hip("hip", ground, hipInGround, Vec3(0), 
                        osim_thigh, hipInFemur, Vec3(0));

    // Add the thigh body which now also contains the hip joint to the model
    osimModel->addBody(&osim_thigh);
    osimModel->addJoint(&hip);

    //OpenSim shank
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
        tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // create Pin knee joint
    PinJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), 
                          osim_shank, kneeInTibia, Vec3(0));

    // Add the thigh body which now also contains the hip joint to the model
    osimModel->addBody(&osim_shank);
    osimModel->addJoint(&knee);

    //OpenSim foot
    OpenSim::Body osim_foot("foot", footMass.getMass(), 
        footMass.getMassCenter(), footMass.getInertia());

    // create Pin ankle joint
    PinJoint ankle("ankle", osim_shank, ankleInTibia, Vec3(0),
                             osim_foot, ankleInFoot, Vec3(0));

    // Add the foot body which now also contains the hip joint to the model
    osimModel->addBody(&osim_foot);
    osimModel->addJoint(&ankle);

    // add a point on line constraint
    WeldConstraint footConstraint( "footConstraint", 
                                   ground, SimTK::Transform(weldInGround), 
                                   osim_foot, SimTK::Transform(weldInFoot) );
    osimModel->addConstraint(&footConstraint);

    // BAD: but if model maintains ownership, it will attempt to delete stack allocated objects
    osimModel->disownAllComponents();

    osimModel->setGravity(gravity_vec);

    //Add analyses before setting up the model for simulation
    Kinematics *kinAnalysis = new Kinematics(osimModel);
    kinAnalysis->setInDegrees(false);
    osimModel->addAnalysis(kinAnalysis);

    osimModel->setup();
    osimModel->print("testWeldConstraint.osim");

    // Need to setup model before adding an analysis since it creates the AnalysisSet
    // for the model if it does not exist.
    State& osim_state = osimModel->initSystem();

    //=========================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, osimModel, osim_state, 
        "testWeldConstraint FAILED\n");
}

void testPointOnLineConstraint()
{
    using namespace SimTK;

    cout << endl;
    cout << "==================================================================" << endl;
    cout << "OpenSim PointOnLineConstraint vs. Simbody Constraint::PointOnLine " << endl;
    cout << "==================================================================" << endl;

    Random::Uniform randomDirection(-1, 1);
    Vec3 lineDirection(randomDirection.getValue(), randomDirection.getValue(),
        randomDirection.getValue());
    UnitVec3 normLineDirection(lineDirection.normalize());
    Vec3 pointOnLine(0,0,0);
    Vec3 pointOnFollower(0,0,0);

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Create a free joint between the foot and ground
    MobilizedBody::Free foot(matter.Ground(), Transform(Vec3(0)), 
        SimTK::Body::Rigid(footMass), Transform(Vec3(0)));
    
    // Constrain foot to line on ground
    SimTK::Constraint::PointOnLine simtkPointOnLine(matter.Ground(), 
        normLineDirection, pointOnLine, foot, pointOnFollower);

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //=========================================================================
    // Setup OpenSim model
    Model *osimModel = new Model;
    //OpenSim bodies
    const Ground& ground = osimModel->getGround();;

    //OpenSim foot
    OpenSim::Body osim_foot("foot", footMass.getMass(),
        footMass.getMassCenter(), footMass.getInertia());

    // create foot as a free joint
    FreeJoint footJoint("footToGround", ground, Vec3(0), Vec3(0),
                                     osim_foot, Vec3(0), Vec3(0));
    
    // Add the thigh body which now also contains the hip joint to the model
    osimModel->addBody(&osim_foot);
    osimModel->addJoint(&footJoint);

    // add a point on line constraint
    PointOnLineConstraint lineConstraint(ground, normLineDirection, pointOnLine,
        osim_foot, pointOnFollower);
    osimModel->addConstraint(&lineConstraint);

    // BAD: have to set memoryOwner to false or program will crash when this test is complete.
    osimModel->disownAllComponents();

    osimModel->setGravity(gravity_vec);

    //Add analyses before setting up the model for simulation
    Kinematics *kinAnalysis = new Kinematics(osimModel);
    kinAnalysis->setInDegrees(false);
    osimModel->addAnalysis(kinAnalysis);

    // Need to setup model before adding an analysis since it creates the AnalysisSet
    // for the model if it does not exist.
    State& osim_state = osimModel->initSystem();

    //==========================================================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, osimModel, osim_state, "testPointOnLineConstraint FAILED\n");
} // end testPointOnLineConstraint

void testCoordinateCouplerConstraint()
{
    using namespace SimTK;

    cout << endl;
    cout << "=================================================================" << endl;
    cout << " OpenSim CoordinateCouplerConstraint vs. FunctionBasedMobilizer  " << endl;
    cout << "=================================================================" << endl;

    // Define spline data for the custom knee joint
    int npx = 12;
    double angX[] = {-2.094395102393, -1.745329251994, -1.396263401595, -1.047197551197, -0.698131700798, -0.349065850399, -0.174532925199, 0.197344221443, 0.337394955864, 0.490177570472, 1.521460267071, 2.094395102393};
    double kneeX[] = {-0.003200000000, 0.001790000000, 0.004110000000, 0.004100000000, 0.002120000000, -0.001000000000, -0.003100000000, -0.005227000000, -0.005435000000, -0.005574000000, -0.005435000000, -0.005250000000};
    int npy = 7;
    double angY[] = {-2.094395102393, -1.221730476396, -0.523598775598, -0.349065850399, -0.174532925199, 0.159148563428, 2.094395102393};
    double kneeY[] = {-0.422600000000, -0.408200000000, -0.399000000000, -0.397600000000, -0.396600000000, -0.395264000000, -0.396000000000 };


    for(int i = 0; i<npy; i++) {
        // Spline data points from experiment w.r.t. hip location. Change to make it w.r.t knee location
        kneeY[i] += (-kneeInFemur[1]+hipInFemur[1]); 
    }

    SimmSpline tx(npx, angX, kneeX);
    SimmSpline ty(npy, angY, kneeY);;

    // Define the functions that specify the FunctionBased Mobilized Body.
    std::vector<std::vector<int> > coordIndices;
    std::vector<const SimTK::Function*> functions;
    std::vector<bool> isdof(6,false);

    // Set the 1 spatial rotation about Z-axis
    isdof[2] = true;  //rot Z
    int nm = 0;
    for(int i=0; i<6; i++){
        if(isdof[i]) {
            Vector coeff(2);
            coeff[0] = 1;
            coeff[1] = 0;
            std::vector<int> findex(1);
            findex[0] = nm++;
            functions.push_back(new SimTK::Function::Linear(coeff));
            coordIndices.push_back(findex);
        }
        else if(i==3 || i ==4){
            std::vector<int> findex(1,0);
            if(i==3)
                functions.push_back(tx.createSimTKFunction());
            else
                functions.push_back(ty.createSimTKFunction());

            coordIndices.push_back(findex); 
        }
        else{
            std::vector<int> findex(0);
            functions.push_back(new SimTK::Function::Constant(0, 0));
            coordIndices.push_back(findex);
        }
    }

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);
    //system.updDefaultSubsystem().addEventReporter(new VTKEventReporter(system, 0.01));

    // Thigh connected by hip
    MobilizedBody::Pin thigh(matter.Ground(), Transform(hipInGround), 
        SimTK::Body::Rigid(MassProperties(femurMass, femurCOM, femurInertiaAboutCOM.shiftFromMassCenter(femurCOM, femurMass))), Transform(hipInFemur));
    //Function-based knee connects shank
    MobilizedBody::FunctionBased shank(thigh, Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), Transform(kneeInTibia), nm, functions, coordIndices);
    //MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur), SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================================================
    // Setup OpenSim model
    Model *osimModel = new Model;
    //OpenSim bodies
    const Ground& ground = osimModel->getGround();;
    
    OpenSim::Body osim_thigh("thigh", femurMass, femurCOM, femurInertiaAboutCOM);

    // create hip as a pin joint
    PinJoint hip("hip",ground, hipInGround, Vec3(0), osim_thigh, hipInFemur, Vec3(0));

    // Rename hip coordinates for a pin joint
    hip.updCoordinate().setName("hip_flex");
    
    // Add the thigh body which now also contains the hip joint to the model
    osimModel->addBody(&osim_thigh);
    osimModel->addJoint(&hip);

    // Add another body via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
        tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // Define knee coordinates and axes for custom joint spatial transform
    SpatialTransform kneeTransform;
    string knee_q_name = "knee_q";
    string tx_name = "knee_tx";
    string ty_name = "knee_ty";

    Array<string> indepCoords(knee_q_name, 1, 1);

    //  knee flexion/extension
    kneeTransform[2].setCoordinateNames(indepCoords);
    kneeTransform[2].setFunction(new LinearFunction());
    // translation X
    kneeTransform[3].setCoordinateNames(OpenSim::Array<std::string>(tx_name, 1, 1));
    kneeTransform[3].setFunction(new LinearFunction());
    // translation Y
    kneeTransform[4].setCoordinateNames(OpenSim::Array<std::string>(ty_name, 1, 1));
    kneeTransform[4].setFunction(new LinearFunction());

    // create custom knee joint
    CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank, kneeInTibia, Vec3(0), kneeTransform);

    // Add the shank body which now also contains the knee joint to the model
    osimModel->addBody(&osim_shank);
    osimModel->addJoint(&knee);

    // Constrain the knee translations to follow the desired manifold
    CoordinateCouplerConstraint knee_tx_constraint;
    CoordinateCouplerConstraint knee_ty_constraint;

    knee_tx_constraint.setName("knee_tx_coupler");
    knee_ty_constraint.setName("knee_ty_coupler");

    knee_tx_constraint.setIndependentCoordinateNames(indepCoords);
    knee_ty_constraint.setIndependentCoordinateNames(indepCoords);

    knee_tx_constraint.setDependentCoordinateName(tx_name);
    knee_ty_constraint.setDependentCoordinateName(ty_name);

    knee_tx_constraint.setFunction(tx);
    knee_ty_constraint.setFunction(ty);

    // Add the constraints
    osimModel->addConstraint(&knee_tx_constraint);
    osimModel->addConstraint(&knee_ty_constraint);

    //Add analyses before setting up the model for simulation
    Kinematics *kinAnalysis = new Kinematics(osimModel);
    kinAnalysis->setInDegrees(false);
    osimModel->addAnalysis(kinAnalysis);

    // OpenSim model must realize the topology to get valid osim_state
    osimModel->setGravity(gravity_vec);

    PointKinematics *pointKin = new PointKinematics(osimModel);
    // Get the point location of the shank origin in space
    pointKin->setBodyPoint("shank", Vec3(0));
    osimModel->addAnalysis(pointKin);

    // Model cannot own model components created on the stack in this test program
    osimModel->disownAllComponents();

    // write out the model to file
    osimModel->finalizeConnections();
    osimModel->print("testCouplerConstraint.osim");

    //wipe-out the model just constructed
    delete osimModel;

    // reconstruct from the model file
    osimModel = new Model("testCouplerConstraint.osim");
    
    ForceReporter *forceReport = new ForceReporter(osimModel);
    forceReport->includeConstraintForces(true);
    osimModel->addAnalysis(forceReport);

    // Need to setup model before adding an analysis since it creates the AnalysisSet
    // for the model if it does not exist.
    State& osim_state = osimModel->initSystem();

    //==========================================================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, osimModel, osim_state, "testCoordinateCouplerConstraint FAILED\n");

    // Forces were held in storage during simulation, now write to file
    forceReport->printResults("CouplerModelForces");
}

void testRollingOnSurfaceConstraint()
{
    using namespace SimTK;

    cout << endl;
    cout << "=================================================================" << endl;
    cout << " OpenSim RollingOnSurfaceConstraint Simulation " << endl;
    cout << "=================================================================" << endl;

    // angle of the rot w.r.t. vertical
    double theta = -SimTK::Pi / 6; // 30 degs
    double omega = -2.1234567890;
    double halfRodLength = 1.0 / (omega*omega);

    UnitVec3 surfaceNormal(0,1,0);
    double planeHeight = 0.0;
    Vec3 comInRod(0, halfRodLength, 0);
    Vec3 contactPointOnRod(0, 0, 0);

    double mass = 7.0;
    SimTK::Inertia inertiaAboutCom = mass*SimTK::Inertia::cylinderAlongY(0.1, 1.0);

    SimTK::MassProperties rodMass(7.0, comInRod,
        inertiaAboutCom.shiftFromMassCenter(comInRod, mass));

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Create a free joint between the rod and ground
    MobilizedBody::Planar rod(matter.Ground(), Transform(Vec3(0)), 
        SimTK::Body::Rigid(rodMass), Transform());

    // Get underlying mobilized bodies
    SimTK::MobilizedBody surface = matter.getGround();

    // Add a fictitious massless body to be the "Case" reference body coincident with surface for the no-slip constraint
    SimTK::MobilizedBody::Weld  cb(surface, SimTK::Body::Massless());

    // Constrain the rod to move on the ground surface
    SimTK::Constraint::PointInPlane contactY(surface, surfaceNormal, planeHeight, rod, contactPointOnRod);
    SimTK::Constraint::ConstantAngle contactTorqueAboutY(surface, SimTK::UnitVec3(1, 0, 0), rod, SimTK::UnitVec3(0, 0, 1));
    // Constrain the rod to roll on surface and not slide 
    SimTK::Constraint::NoSlip1D contactPointXdir(cb, SimTK::Vec3(0), SimTK::UnitVec3(1, 0, 0), surface, rod);
    SimTK::Constraint::NoSlip1D contactPointZdir(cb, SimTK::Vec3(0), SimTK::UnitVec3(0, 0, 1), surface, rod);
    
    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();

    //state = system.realizeTopology();
    state.updQ()[0] = theta;
    state.updQ()[1] = 0;
    state.updQ()[2] = 0;
    state.updU()[0] = omega;

    system.realize(state, Stage::Acceleration);
    state.getUDot().dump("Simbody Accelerations");

    Vec3 pcom = system.getMatterSubsystem().calcSystemMassCenterLocationInGround(state);
    Vec3 vcom = system.getMatterSubsystem().calcSystemMassCenterVelocityInGround(state);
    Vec3 acom = system.getMatterSubsystem().calcSystemMassCenterAccelerationInGround(state);

    //==========================================================================================================
    // Setup OpenSim model
    Model *osimModel = new Model;
    osimModel->setGravity(gravity_vec);

    //OpenSim bodies
    Ground& ground = osimModel->updGround();;
    Mesh arrowGeom("arrow.vtp");
    arrowGeom.setColor(Vec3(1, 0, 0));
    ground.attachGeometry(arrowGeom.clone());

    //OpenSim rod
    auto osim_rod = new OpenSim::Body("rod", mass, comInRod, inertiaAboutCom);
    OpenSim::PhysicalOffsetFrame* cylFrame = new PhysicalOffsetFrame(*osim_rod, Transform(comInRod));
    cylFrame->setName("comInRod");
    osimModel->addComponent(cylFrame);
    Mesh cylGeom("cylinder.vtp");
    cylGeom.set_scale_factors(2 * halfRodLength*Vec3(0.1, 1, 0.1));
    cylFrame->attachGeometry(cylGeom.clone());

    // create rod as a free joint
    auto rodJoint = new PlanarJoint("rodToGround", ground, *osim_rod);

    // Add the thigh body which now also contains the hip joint to the model
    osimModel->addBody(osim_rod);
    osimModel->addJoint(rodJoint);

    // add a point on line constraint
    auto roll = new RollingOnSurfaceConstraint();
    roll->connectSocket_rolling_body(*osim_rod);
    roll->connectSocket_surface_body(ground);

    /*double h = */roll->get_surface_height();
    
    osimModel->addConstraint(roll);
    osimModel->setGravity(gravity_vec);

    //Add analyses before setting up the model for simulation
    Kinematics *kinAnalysis = new Kinematics(osimModel);
    kinAnalysis->setInDegrees(false);
    osimModel->addAnalysis(kinAnalysis);

    // Need to setup model before adding an analysis since it creates the AnalysisSet
    // for the model if it does not exist.
    //osimModel->setUseVisualizer(true);
    State osim_state = osimModel->initSystem();
    roll->setIsEnforced(osim_state, true);
    osim_state.updY() = state.getY();

    // compute model accelerations
    osimModel->computeStateVariableDerivatives(osim_state);
    osim_state.getUDot().dump("Osim Accelerations");

    //osimModel->updVisualizer().updSimbodyVisualizer()
    //    .setBackgroundType(SimTK::Visualizer::GroundAndSky);
    //osimModel->getVisualizer().show(osim_state);

    Vec3 osim_pcom = osimModel->calcMassCenterPosition(osim_state);
    Vec3 osim_vcom = osimModel->calcMassCenterVelocity(osim_state);
    Vec3 osim_acom = osimModel->calcMassCenterAcceleration(osim_state);

    Vec3 tol(SimTK::SignificantReal);

    ASSERT_EQUAL(pcom, osim_pcom, tol);
    ASSERT_EQUAL(vcom, osim_vcom, tol);
    ASSERT_EQUAL(acom, osim_acom, tol);

    //==========================================================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, osimModel, osim_state, "testRollingOnSurfaceConstraint FAILED\n");
}

void testSerializeDeserialize() {
    std::cout << "Test serialize & deserialize." << std::endl;

    std::string origModelFile{"testJointConstraints.osim"};
    std::string oldModelFile{"testConstraints_SerializeDeserialize_old.osim"};
    std::string newModelFile{"testConstraints_SerializeDeserialize_new.osim"};

    // Toggle of 'isDisabled' property for some muscles in model file.
    std::set<std::string> flippedConstraints{"tib_pat_r_r3_con",
                                             "tib_pat_r_tx_con",
                                             "tib_pat_r_ty_con"};
    {
        auto xml = SimTK::Xml::Document{origModelFile};
        auto constraints = xml.getRootElement().
                               getRequiredElement("Model").
                               getRequiredElement("ConstraintSet").
                               getRequiredElement("objects").
                               getAllElements("CoordinateCouplerConstraint");
        for(unsigned i = 0; i < constraints.size(); ++i) {
            const auto& constraintName =
                constraints[i].getRequiredAttributeValue("name");
            if(flippedConstraints.find(constraintName) !=
               flippedConstraints.end()) {
                auto elem = constraints[i].getRequiredElement("isDisabled");
                elem.setValue("true");
            }
        }
        xml.writeToFile(oldModelFile);
    }

    // Model with Force::isDisabled (version < 30508)
    Model oldModel{oldModelFile};
    oldModel.print(newModelFile);
    Model newModel{newModelFile};

    const auto& oldConstraintSet = oldModel.getConstraintSet();
    const auto& newConstraintSet = newModel.getConstraintSet();

    ASSERT(oldConstraintSet.getSize() == newConstraintSet.getSize());
    for(int i = 0; i < oldConstraintSet.getSize(); ++i) {
        ASSERT(oldConstraintSet.get(i).get_isEnforced() ==
               newConstraintSet.get(i).get_isEnforced());

        if(flippedConstraints.find(newConstraintSet.get(i).getName()) !=
           flippedConstraints.end())
            ASSERT(newConstraintSet.get(i).get_isEnforced() == false);
    }

    std::remove(oldModelFile.c_str());
    std::remove(newModelFile.c_str());
}
