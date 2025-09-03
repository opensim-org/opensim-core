/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testJoints.cpp                          *
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
//  testJoints builds OpenSim models using the API and builds an equivalent
//  Simbody system using the Simbody API for each test case. A test fails if
//  the OpenSim and Simbody final states of the simulation are not equivalent
//  (norm(error) greater than 10x integration error tolerance)
//
//  Tests Include:
//      1. CustomJoint against Simbody built-in Pin and Universal mobilizers
//      2. CustomJoint versus Simbody FunctionBased with spline functions
//      3. EllipsoidJoint against Simbody built-in Ellipsoid mobilizer
//      4. WeldJoint versus Weld Mobilizer by welding bodies
//      5. Randomized order of bodies in the BodySet (in 3.) to test connect()
//      6. PinJoint against Simbody built-in Pin mobilizer
//      7. SliderJoint against Simbody built-in Slider mobilizer
//      8. FreeJoint against Simbody built-in Free mobilizer
//      9. BallJoint against Simbody built-in Ball mobilizer
//     10. Equivalent Spatial body force due to applied gen force.
//
//     Add tests here as new joint types are added to OpenSim
//
//=============================================================================

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/FunctionAdapter.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/BallJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/EllipsoidJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/GimbalJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/ScapulothoracicJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/UniversalJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/Test/SimulationComponentsForTesting.h>
#include <OpenSim/Actuators/PointActuator.h>

#include <memory>
#include <catch2/catch_all.hpp>

using namespace OpenSim;
using namespace std;

//=============================================================================

namespace {
    // Common Parameters for the simulations are just global.
    const static double integ_accuracy = 1.0e-6;
    const static double duration = 0.50;
    const static SimTK::Vec3 gravity_vec = SimTK::Vec3(0, -9.8065, 0);
    // Thigh
    const static SimTK::MassProperties femurMass(8.806, SimTK::Vec3(0),
            SimTK::Inertia(SimTK::Vec3(0.1268, 0.0332, 0.1337)));

    // Shank
    const static SimTK::MassProperties tibiaMass(3.510, SimTK::Vec3(0),
            SimTK::Inertia(SimTK::Vec3(0.0477, 0.0048, 0.0484)));
    // Foot
    const static SimTK::MassProperties footMass(1.20, SimTK::Vec3(0),
            SimTK::Inertia(SimTK::Vec3(0.001361, 0.003709, 0.003916)));
    // Toes
    const static SimTK::MassProperties toesMass(0.205126, SimTK::Vec3(0),
            SimTK::Inertia(SimTK::Vec3(0.000117, 0.000179, 0.000119)));

    // Joint locations
    const SimTK::Vec3 hipInPelvis(0.0, -0.02, 0.2);
    const SimTK::Vec3 hipInFemur(0.0020, 0.1715, 0);
    const SimTK::Vec3 kneeInFemur(0.0033, -0.2294, 0);
    const SimTK::Vec3 kneeInTibia(0.0, 0.1862, 0.0);
    const SimTK::Vec3 ankleInTibia(0.0, -0.243800, 0);
    const SimTK::Vec3 ankleInFoot(-0.035902, 0.051347, 0);
    const SimTK::Vec3 mtpInFoot(0.098032, -0.038000, 0);
    const SimTK::Vec3 mtpInToes(-0.035902, 0.051347, 0);

    const SimTK::Vec3 footInGround(0.25, 0, 0);

    //==========================================================================

    // Class for testing joints with coupled coordinates
    class MultidimensionalFunction : public OpenSim::Function {
        OpenSim_DECLARE_CONCRETE_OBJECT(
                MultidimensionalFunction, OpenSim::Function);

    public:
        MultidimensionalFunction(){};
        virtual ~MultidimensionalFunction(){};

        double calcValue(const SimTK::Vector& x) const override {
            return 2 * x[0] * x[0] + x[1];
        }
        double calcDerivative(const std::vector<int>& derivComponents,
                const SimTK::Vector& x) const override {
            int nd = (int)derivComponents.size();
            if (nd < 1) return SimTK::NaN;

            if (derivComponents[0] == 0) {
                if (nd == 1)
                    return 4 * x[0];
                else if (derivComponents[1] == 0)
                    return 4;
            } else if (derivComponents[0] == 1) {
                if (nd == 1) return 1;
            }
            return 0;
        }
        int getArgumentSize() const override { return 2; }
        int getMaxDerivativeOrder() const override { return 2; }
        SimTK::Function* createSimTKFunction() const override {
            return new FunctionAdapter(*this);
        }
    }; // End of MultidimensionalFunction

    //==========================================================================
    // Common Functions
    //==========================================================================
    int initTestStates(SimTK::Vector& qi, SimTK::Vector& ui) {
        using namespace SimTK;

        Random::Uniform randomAngle(-Pi / 4, Pi / 4);
        Random::Uniform randomSpeed(-1.0, 1.0);

        // Provide initial states as random angles and speeds for OpenSim and
        // Simbody models
        for (int i = 0; i < qi.size(); i++) qi[i] = randomAngle.getValue();

        for (int i = 0; i < ui.size(); i++) ui[i] = randomSpeed.getValue();

        return qi.size();
    }

    void integrateSimbodySystem(
            SimTK::MultibodySystem& system, SimTK::State& state) {
        using namespace SimTK;

        // realize simbody system to velocity stage
        system.realize(state, Stage::Velocity);

        RungeKuttaFeldbergIntegrator integ(system);
        integ.setAccuracy(integ_accuracy);

        TimeStepper ts(system, integ);
        ts.initialize(state);
        ts.stepTo(duration);
        state = ts.getState();
    }

    void integrateOpenSimModel(Model* osimModel, SimTK::State& osim_state) {
        using namespace SimTK;

        // SETUP OpenSim SIMULATION Manager
        osimModel->getMultibodySystem().realize(osim_state, Stage::Velocity);
        Manager manager(*osimModel);
        manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKuttaFeldberg);
        manager.setIntegratorAccuracy(integ_accuracy);

        // Specify the initial and final times of the simulation.
        // In this case, the initial and final times are set based on
        // the range of times over which the controls are available.
        // Control *control;
        osim_state.setTime(0.0);
        manager.initialize(osim_state);

        // Integrate
        /*const SimbodyMatterSubsystem& matter2 = */ osimModel->getMultibodySystem()
                .getMatterSubsystem();
        // for (int i = 0; i < matter2.getNumConstraints(); i++)
        //    printf("%d: %d\n", i, matter2.isConstraintDisabled(osim_state,
        //    SimTK::ConstraintIndex(i)));
        // cout << osim_state.getQ()<<endl;
        // cout << "\n\nOpenSim Integration 0.0 to " << duration << endl;

        osim_state = manager.integrate(duration);
    }

    void compareSimulationStates(const SimTK::Vector& q_sb,
            const SimTK::Vector& u_sb, const SimTK::Vector& q_osim,
            const SimTK::Vector& u_osim, string errorMessagePrefix = "") {
        using namespace SimTK;

        Vector q_err = q_osim;
        Vector u_err = u_sb - u_osim;

        int nq = q_osim.size();
        if (q_sb.size() > nq) { // we have an unused quaternion slot in Simbody

            q_sb.dump("Simbody q's:");
            q_osim.dump("OpenSim q's:");
            // This is a hack knowing the free and ball joint tests have the
            // quaternion joint first
            // And that the q's are packed as qqqq or aaa* for a ball and
            // qqqqxyz or aaaxyz* for a free joint
            int quat_ind = ((nq > 6) ? 6 : 3);
            int j = 0;
            if (quat_ind > 5) { // this is a free joint
                // OpenSim specifies Translation mobilizer first so first and
                // second triplet of q's have to be swapped
                for (int i = 0; i < 3; i++) {
                    q_err[i] = q_osim[i] - q_sb[i + 3];
                    q_err[i + 3] = q_osim[i + 3] - q_sb[i];
                    u_err[i] = u_osim[i] - u_sb[i + 3];
                    u_err[i + 3] = u_osim[i + 3] - u_sb[i];
                }
                j = quat_ind;
            }
            for (int i = j; i < q_sb.size(); i++) {
                if (i != quat_ind) {
                    q_err[j] = q_sb[i] - q_osim[j];
                    j++;
                }
            }
        } else {
            q_err = q_sb - q_osim;
        }

        double qerrnorm = q_err.norm();
        double uerrnorm = u_err.norm();

        cout << "\nSimbody - OpenSim:  |q_err| = " << qerrnorm
             << "  |u_err| =" << uerrnorm << endl;

        stringstream errorMessage1, errorMessage2;
        errorMessage1 << "testJoints::compareSimulationStates failed q_err.norm = "
                      << qerrnorm;
        errorMessage2 << "testJoints::compareSimulationStates failed u_err.norm = "
                      << uerrnorm;
        ASSERT(qerrnorm <= 10 * integ_accuracy, __FILE__, __LINE__,
                errorMessagePrefix + errorMessage1.str());
        ASSERT(uerrnorm <= 100 * integ_accuracy, __FILE__, __LINE__,
                errorMessagePrefix + errorMessage2.str());
    }

    void compareSimulations(SimTK::MultibodySystem& system, SimTK::State& state,
            Model* osimModel, SimTK::State& osim_state,
            string errorMessagePrefix = "") {
        using namespace SimTK;

        // Set the initial states for both Simbody system and OpenSim model
        Vector& q = state.updQ();
        Vector& u = state.updU();
        /*int nq_sb = */ initTestStates(q, u);
        /*int nq = */ osim_state.getNQ();

        // Push down to OpenSim "state"
        osim_state.updY() = state.getY();
        Vector delta = osim_state.updY() - state.getY();
        /*double errnorm = */ delta.norm();
        cout << "osim_state - sb_state: " << delta << endl;

        /* Debugging Info */
        // system.realize(state, Stage::Acceleration);
        // osimModel->getSystem().realize(osim_state, Stage::Acceleration);

        // state.getUDot().dump("Simbody UDot");
        // osim_state.getUDot().dump("OpenSim UDot");

        //======================================================================
        // Integrate Simbody system
        integrateSimbodySystem(system, state);

        // Simbody model final states
        q = state.updQ();
        u = state.updU();

        cout << "\nSimbody Final q's: " << q << endl;
        cout << "Simbody Final u's: " << u << endl;

        //======================================================================
        // Integrate OpenSim model
        integrateOpenSimModel(osimModel, osim_state);

        // Get the state at the end of the integration from OpenSim.
        Vector& qos = osim_state.updQ();
        Vector& uos = osim_state.updU();
        cout << "\nOpenSim Final q's: " << qos << endl;
        cout << "OpenSim Final u's: " << uos << endl;

        //======================================================================
        // Compare Simulation Results
        compareSimulationStates(q, u, qos, uos, errorMessagePrefix);
    }
    //==========================================================================

    void testEquivalentBodyForceForGenForces(Model& model) {
        using namespace SimTK;

        State& state = model.initSystem();
        Vector& qi = state.updQ();
        Vector& ui = state.updU();
        // Randomly select the initial state of this model
        /*int nq = */ initTestStates(qi, ui);

        const SimbodyMatterSubsystem& matter = model.getMatterSubsystem();

        // The number of mobilities for the entire system.
        int nm = matter.getNumMobilities();

        Vector genForces(nm, 0.0);
        Random::Uniform genForceRandom(-1000, 1000);
        for (int i = 0; i < nm; ++i) { genForces[i] = genForceRandom.getValue(); }

        int nb = matter.getNumBodies();
        Vector_<SpatialVec> bodyForces(nb, SpatialVec(Vec3(0), Vec3(0)));

        Vector udot1(nm);
        Vector_<SpatialVec> bodyAccs(nb);

        model.getMultibodySystem().realize(state, SimTK::Stage::Acceleration);
        matter.calcAcceleration(state, genForces, bodyForces, udot1, bodyAccs);

        // Construct the system vector of body forces from a Joint's equivalence
        // to generalized force calculations
        for (int j = 0; j < model.getJointSet().getSize(); ++j) {
            Joint& joint = model.getJointSet()[j];
            const PhysicalFrame& B = joint.getChildFrame();
            MobilizedBodyIndex mbx = B.getMobilizedBodyIndex();
            const Frame& Bo = B.findBaseFrame();

            const PhysicalFrame& P = joint.getParentFrame();
            MobilizedBodyIndex mpx = P.getMobilizedBodyIndex();
            const Frame& Po = P.findBaseFrame();

            Vec3 rB_Bo(0), rB_Po(0);
            rB_Bo = joint.getChildFrame().findTransformInBaseFrame().p();

            // Get Joint frame B location in parent, Po, to apply to parent Body
            rB_Po = Bo.findStationLocationInAnotherFrame(state, rB_Bo, Po);

            // get the equivalent spatial force on the joint frame of the (child)
            // body expressed in ground
            SpatialVec FB_G = joint.calcEquivalentSpatialForce(state, genForces);

            cout << joint.getName() << " equivalent FB_G = " << FB_G << endl;

            // Apply spatial forces at joint to the body
            matter.addInStationForce(state, mbx, rB_Bo, FB_G[1], bodyForces);
            matter.addInBodyTorque(state, mbx, FB_G[0], bodyForces);

            // Apply equal and opposite spatial forces at joint to the parent body
            matter.addInStationForce(state, mpx, rB_Po, -FB_G[1], bodyForces);
            matter.addInBodyTorque(state, mpx, -FB_G[0], bodyForces);
        }

        Vector udot2(nm);
        matter.calcAcceleration(
                state, 0.0 * genForces, bodyForces, udot2, bodyAccs);

        // If calcEquivalentSpatialForce is correct then the two methods of applying
        // forces to the model should be equivalent and the accelerations should be
        // identical
        Vector error = udot2 - udot1;
        double norm_rel_error = error.norm() / udot1.norm();

        cout << "**************************************************************"
                "**************"
             << endl;
        cout << "uDot Error = " << norm_rel_error
             << ": from body forces vs. mobility forces." << endl;
        cout << "**************************************************************"
                "**************"
             << endl;

        ASSERT(!SimTK::isNaN(norm_rel_error), __FILE__, __LINE__,
                "testEquivalentBodyForceForGenForces FAILED, udot_error = NaN");
        ASSERT(norm_rel_error <= SimTK::SignificantReal, __FILE__, __LINE__,
                "testEquivalentBodyForceForGenForces FAILED, udot_error > "
                "SimTK::SignificantReal");
    }

    void testWeldJoint(bool randomizeBodyOrder) {
        using namespace SimTK;

        cout << endl;
        cout << "================================================================"
             << endl;
        cout << "  OpenSim WeldJoint vs. Simbody's Weld Mobilizer " << endl;
        cout << "================================================================"
             << endl;

        // Define the Simbody system
        MultibodySystem system;
        SimbodyMatterSubsystem matter(system);
        GeneralForceSubsystem forces(system);
        SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

        // Thigh connected by hip
        MobilizedBody::Universal thigh(matter.Ground(),
                SimTK::Transform(hipInPelvis), SimTK::Body::Rigid(femurMass),
                SimTK::Transform(hipInFemur));
        // Function-based knee connects shank
        MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur),
                SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));
        // Weld foot to shank at ankle
        MobilizedBody::Weld foot(shank, SimTK::Transform(ankleInTibia),
                SimTK::Body::Rigid(footMass), SimTK::Transform(ankleInFoot));
        // fixed toes at right mtp
        MobilizedBody::Weld toes(foot, SimTK::Transform(mtpInFoot),
                SimTK::Body::Rigid(toesMass), SimTK::Transform(mtpInToes));

        // Simbody model state setup
        system.realizeTopology();
        State state = system.getDefaultState();
        matter.setUseEulerAngles(state, true);
        system.realizeModel(state);

        //======================================================================
        // Setup OpenSim model
        Model* osimModel = new Model;

        // OpenSim bodies
        const Ground& ground = osimModel->getGround();
        ;
        OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
                femurMass.getMassCenter(), femurMass.getInertia());

        // Use a temporary BodySet to hold bodies
        BodySet tempBodySet;
        tempBodySet.setMemoryOwner(false);

        // Use a temporary BodySet to hold bodies
        JointSet tempJointSet;
        tempJointSet.setMemoryOwner(false);

        // Define hip coordinates and axes for custom joint
        SpatialTransform hipTransform;
        hipTransform[0].setCoordinateNames(
                OpenSim::Array<std::string>("hip_q0", 1, 1));
        hipTransform[0].setFunction(new LinearFunction());
        hipTransform[1].setCoordinateNames(
                OpenSim::Array<std::string>("hip_q1", 1, 1));
        hipTransform[1].setFunction(new LinearFunction());

        // create custom hip joint
        CustomJoint hip("hip", ground, hipInPelvis,
                Vec3(0), osim_thigh, hipInFemur,
                Vec3(0), hipTransform);

        tempBodySet.adoptAndAppend(&osim_thigh);
        tempJointSet.adoptAndAppend(&hip);

        // Add another body via a knee joint
        OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
                tibiaMass.getMassCenter(), tibiaMass.getInertia());

        // Define knee transform for flexion/extension
        SpatialTransform kneeTransform;
        kneeTransform[2].setCoordinateNames(
                OpenSim::Array<std::string>("knee_q", 1, 1));
        kneeTransform[2].setFunction(new LinearFunction());

        // create custom knee joint
        CustomJoint knee("knee", osim_thigh, kneeInFemur,
                Vec3(0), osim_shank, kneeInTibia,
                Vec3(0), kneeTransform);

        tempBodySet.adoptAndAppend(&osim_shank);
        tempJointSet.adoptAndAppend(&knee);

        // Add foot body at ankle
        OpenSim::Body osim_foot("foot", footMass.getMass(),
                footMass.getMassCenter(), footMass.getInertia());
        WeldJoint ankle("ankle", osim_shank, ankleInTibia,
                Vec3(0), osim_foot,
                ankleInFoot, Vec3(0));

        tempBodySet.adoptAndAppend(&osim_foot);
        tempJointSet.adoptAndAppend(&ankle);

        // Add toes body at mtp
        OpenSim::Body osim_toes("toes", toesMass.getMass(),
                toesMass.getMassCenter(), toesMass.getInertia());
        WeldJoint mtp("mtp", osim_foot, mtpInFoot,
                Vec3(0), osim_toes, mtpInToes,
                Vec3(0));

        tempBodySet.adoptAndAppend(&osim_toes);
        tempJointSet.adoptAndAppend(&mtp);

        int b_order[] = {0, 1, 2, 3};
        int j_order[] = {0, 1, 2, 3};
        if (randomizeBodyOrder) {
            cout << " Randomizing Bodies to exercise model's multibody graph maker "
                 << endl;
            cout << "=========================================================="
                    "======"
                 << endl;
            Random::Uniform randomOrder(0, 4);
            randomOrder.setSeed((int)clock());

            int bx = -1, jx = -1;
            bool duplicate = false;
            for (int i = 0; i < 4; ++i) {
                bx = randomOrder.getIntValue();
                duplicate = false;
                for (int j = 0; j < i; j++) {
                    // check if we can find this index in the order list already
                    if (bx == b_order[j]) {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate)
                    --i; // try again
                else
                    b_order[i] = bx;
            }
            for (int i = 0; i < 4; ++i) {
                jx = randomOrder.getIntValue();
                duplicate = false;
                for (int j = 0; j < i; j++) {
                    // check if we can find this index in the order list already
                    if (jx == j_order[j]) {
                        duplicate = true;
                        break; // if hit a duplicate stop
                    }
                }
                if (duplicate)
                    --i; // try again
                else
                    j_order[i] = jx;
            }
        }

        // Can add bodies in random order, but joints depending on those bodies
        // have to be added afterwards
        for (int i = 0; i < 4; i++) {
            osimModel->addBody(&tempBodySet[b_order[i]]);
        }

        // Add joints in any order as long as the bodies (PhysicalFrames) they
        // must connect to exist.
        for (int i = 0; i < 4; i++) {
            osimModel->addJoint(&tempJointSet[j_order[i]]);
        }

        // BAD: have to set memoryOwner to false or program will crash when this
        // test is complete.
        osimModel->disownAllComponents();

        // OpenSim model must realize the topology to get valid osim_state
        osimModel->setGravity(gravity_vec);

        if (randomizeBodyOrder) {
            osimModel->print("testRandomizedBodyAndJointOrder.osim");
        }

        cout << "testWeldJoint: testEquivalentBodyForceForGenForces" << endl;
        testEquivalentBodyForceForGenForces(*osimModel);

        SimTK::State osim_state = osimModel->initSystem();

        //======================================================================
        // Compare Simbody system and OpenSim model simulations
        stringstream errorMessage;
        errorMessage << "testWeldJoint "
                     << (randomizeBodyOrder ? "with random body order " : "")
                     << "FAILED\n";
        compareSimulations(
                system, state, osimModel,
                osim_state, errorMessage.str());

        // Test accessors.
        {
            WeldJoint myWeldJoint;
            ASSERT_THROW(
                    OpenSim::JointHasNoCoordinates, myWeldJoint.getCoordinate());
            ASSERT_THROW(
                    OpenSim::JointHasNoCoordinates, myWeldJoint.updCoordinate());
        }
    }
}

//==============================================================================
// Test Cases
//==============================================================================
/// First compare behavior of a double pendulum with Universal hip and
/// Pin-like knee
TEST_CASE("testCustomVsUniversalPin") {
    using namespace SimTK;

    cout << endl;
    cout << "=========================================================="
         << endl;
    cout << " OpenSim CustomJoint vs. Simbody Universal and Pin Joints "
         << endl;
    cout << "=========================================================="
         << endl;

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Thigh connected by hip
    MobilizedBody::Universal thigh(matter.Ground(),
            SimTK::Transform(hipInPelvis), SimTK::Body::Rigid(femurMass),
            SimTK::Transform(hipInFemur));
    // Function-based knee connects shank
    MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur),
            SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================
    // Setup OpenSim model
    Model osimModel;

    // OpenSim bodies
    const Ground& ground = osimModel.getGround();
    // OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    // Define hip transform in terms of coordinates and axes for custom joint
    SpatialTransform hipTransform;
    hipTransform[0].setCoordinateNames(
            OpenSim::Array<std::string>("hip_q0", 1, 1));
    hipTransform[0].setFunction(new LinearFunction());
    hipTransform[1].setCoordinateNames(
            OpenSim::Array<std::string>("hip_q1", 1, 1));
    hipTransform[1].setFunction(new LinearFunction());

    // create custom hip joint
    CustomJoint hip("hip", ground, hipInPelvis,
            Vec3(0), osim_thigh, hipInFemur,
            Vec3(0), hipTransform);

    // Add the thigh body which now also contains the hip joint to the model
    osimModel.addBody(&osim_thigh);
    osimModel.addJoint(&hip);

    // Add OpenSim shank via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // Define knee coordinates and axes for custom joint spatial transform
    SpatialTransform kneeTransform;
    string knee_rot = "knee_ext";
    // Only knee flexion/extension
    kneeTransform[2].setCoordinateNames(
            OpenSim::Array<std::string>(knee_rot, 1, 1));
    kneeTransform[2].setFunction(new LinearFunction());

    // create custom knee joint
    CustomJoint knee("knee", osim_thigh, kneeInFemur,
            Vec3(0), osim_shank,
            kneeInTibia, Vec3(0), kneeTransform);

    // Add the shank body which now also contains the knee joint to the model
    osimModel.addBody(&osim_shank);
    osimModel.addJoint(&knee);

    // Set gravity
    osimModel.setGravity(gravity_vec);

    osimModel.disownAllComponents();

    std::cout << osimModel.getCoordinateSet().getSize() << std::endl;

    osimModel.finalizeConnections();
    osimModel.print("testCustomVsUniversalPin.osim");

    testEquivalentBodyForceForGenForces(osimModel);

    Model testModel("testCustomVsUniversalPin.osim");

    SimTK::State osim_state = testModel.initSystem();

    //==========================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, &osimModel, osim_state,
            "testCustomVsUniversalPin FAILED\n");

} // end of testCustomVsUniversalPin

/// Compare behavior of a double pendulum with pin hip and function-based
/// translating tibia knee
TEST_CASE("testCustomJointVsFunctionBased") {
    using namespace SimTK;

    cout << endl;
    cout << "=========================================================="
         << endl;
    cout << " OpenSim CustomJoint vs. Simbody FunctionBased Mobilizer  "
         << endl;
    cout << "=========================================================="
         << endl;

    // Define spline data for the custom knee joint
    int npx = 12;
    double angX[] = {-2.094395102393, -1.745329251994, -1.396263401595,
            -1.047197551197, -0.698131700798, -0.349065850399, -0.174532925199,
            0.197344221443, 0.337394955864, 0.490177570472, 1.521460267071,
            2.094395102393};
    double kneeX[] = {-0.003200000000, 0.001790000000, 0.004110000000,
            0.004100000000, 0.002120000000, -0.001000000000, -0.003100000000,
            -0.005227000000, -0.005435000000, -0.005574000000, -0.005435000000,
            -0.005250000000};
    int npy = 7;
    double angY[] = {-2.094395102393, -1.221730476396, -0.523598775598,
            -0.349065850399, -0.174532925199, 0.159148563428, 2.094395102393};
    double kneeY[] = {-0.422600000000, -0.408200000000, -0.399000000000,
            -0.397600000000, -0.396600000000, -0.395264000000, -0.396000000000};

    for (int i = 0; i < npy; ++i) {
        // Spline data points from experiment w.r.t. hip location. Change to
        // make it w.r.t knee location
        kneeY[i] += (-kneeInFemur[1] + hipInFemur[1]);
    }

    SimmSpline tx(npx, angX, kneeX);
    SimmSpline ty(npy, angY, kneeY);
    ;

    // Define the functions that specify the FunctionBased Mobilized Body.
    std::vector<std::vector<int>> coordIndices;
    std::vector<const SimTK::Function*> functions;
    std::vector<bool> isdof(6, false);

    // Set the 1 spatial rotation about Z-axis
    isdof[2] = true; // rot Z
    int nm = 0;
    for (int i = 0; i < 6; i++) {
        if (isdof[i]) {
            Vector coeff(2);
            coeff[0] = 1;
            coeff[1] = 0;
            std::vector<int> findex(1);
            findex[0] = nm++;
            functions.push_back(new SimTK::Function::Linear(coeff));
            coordIndices.push_back(findex);
        } else if (i == 3 || i == 4) {
            std::vector<int> findex(1, 0);
            if (i == 3)
                functions.push_back(tx.createSimTKFunction());
            else
                functions.push_back(ty.createSimTKFunction());

            coordIndices.push_back(findex);
        } else {
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
    // system.updDefaultSubsystem().addEventReporter(new
    // VTKEventReporter(system, 0.01));

    // Thigh connected by hip
    MobilizedBody::Pin thigh(matter.Ground(), SimTK::Transform(hipInPelvis),
            SimTK::Body::Rigid(femurMass), SimTK::Transform(hipInFemur));
    // Function-based knee connects shank
    MobilizedBody::FunctionBased shank(thigh, SimTK::Transform(kneeInFemur),
            SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia), nm,
            functions, coordIndices);

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    system.realizeModel(state);

    //==========================================================================
    // Setup OpenSim model
    Model* osimModel = new Model;
    // OpenSim bodies
    const Ground& ground = osimModel->getGround();
    ;

    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    // Define hip coordinates and axes for custom joint
    SpatialTransform hipTransform;
    hipTransform[2].setCoordinateNames(Array<std::string>("hip_q0", 1, 1));
    hipTransform[2].setFunction(new LinearFunction());

    // create custom hip joint
    CustomJoint hip("hip", ground, hipInPelvis,
            Vec3(0), osim_thigh, hipInFemur,
            Vec3(0), hipTransform);

    // Add the thigh body which now also contains the hip joint to the model
    osimModel->addBody(&osim_thigh);
    osimModel->addJoint(&hip);

    // Add another body via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // Define knee coordinates and axes for custom joint spatial transform
    SpatialTransform kneeTransform;
    string coord_name = "knee_q";
    //  knee flexion/extension
    kneeTransform[2].setCoordinateNames(
            OpenSim::Array<std::string>(coord_name, 1, 1));
    kneeTransform[2].setFunction(new LinearFunction());
    // translation X
    kneeTransform[3].setCoordinateNames(
            OpenSim::Array<std::string>(coord_name, 1, 1));
    kneeTransform[3].setFunction(tx);
    // translation Y
    kneeTransform[4].setCoordinateNames(
            OpenSim::Array<std::string>(coord_name, 1, 1));
    kneeTransform[4].setFunction(ty);

    // create custom knee joint
    CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank,
            kneeInTibia, Vec3(0), kneeTransform);

    // Add the shank body which now also contains the knee joint to the model
    osimModel->addBody(&osim_shank);
    osimModel->addJoint(&knee);

    // BAD: have to set memoryOwner to false or program will crash when this
    // test is complete.
    osimModel->disownAllComponents();

    // Set gravity
    osimModel->setGravity(gravity_vec);

    // write out the model to file
    osimModel->print("testCustomJoint.osim");

    testEquivalentBodyForceForGenForces(*osimModel);

    SimTK::State osim_state = osimModel->initSystem();

    //==========================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, osimModel, osim_state,
            "testCustomJointVsFunctionBased FAILED\n");

} // end of testCustomJointVsFunctionBased

/// Compare behavior of a double pendulum with an Ellipsoid hip and pin knee
TEST_CASE("testEllipsoidJoint") {
    using namespace SimTK;

    cout << endl;
    cout << "============================================================="
         << endl;
    cout << " OpenSim EllipsoidJoint vs. Simbody MobilizedBody::Ellipsoid "
         << endl;
    cout << "============================================================="
         << endl;

    // Ellipsoid radii
    Vec3 ellipsoidRadii(0.5, 0.33, 0.25);

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Thigh connected by hip
    MobilizedBody::Ellipsoid thigh(matter.Ground(),
            SimTK::Transform(hipInPelvis), SimTK::Body::Rigid(femurMass),
            SimTK::Transform(hipInFemur));
    // Pin knee connects shank
    MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur),
            SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

    thigh.setDefaultRadii(ellipsoidRadii);
    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================
    // Setup OpenSim model
    Model* osimModel = new Model;
    // OpenSim bodies
    const Ground& ground = osimModel->getGround();
    ;

    // OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    // create hip as an ellipsoid joint
    EllipsoidJoint hip("hip", ground, hipInPelvis, Vec3(0), osim_thigh,
            hipInFemur, Vec3(0), ellipsoidRadii);

    // Rename hip coordinates for an ellipsoid joint
    for (int i = 0; i < hip.numCoordinates(); i++) {
        std::stringstream coord_name;
        coord_name << "hip_q" << i;
        hip.upd_coordinates(i).setName(coord_name.str());
    }

    // Add the thigh body which now also contains the hip joint to the model
    osimModel->addBody(&osim_thigh);
    osimModel->addJoint(&hip);

    // Add OpenSim shank via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // create pin knee joint
    PinJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank,
            kneeInTibia, Vec3(0));

    // Add the shank body which now also contains the knee joint to the model
    osimModel->addBody(&osim_shank);
    osimModel->addJoint(&knee);

    // BAD: have to set memoryOwner to false or program will crash when this
    // test is complete.
    osimModel->disownAllComponents();

    osimModel->setGravity(gravity_vec);

    // Write model to file
    osimModel->print("testEllipsoidJoint.osim");

    cout << "EllipsoidJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(*osimModel);

    // Wipe out model
    // delete osimModel;

    // Load model from file
    // osimModel = new Model("testEllipsoidJoint.osim");

    osimModel->setUseVisualizer(false);
    SimTK::State osim_state = osimModel->initSystem();

    //==========================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, osimModel, osim_state,
            "testEllipsoidJoint FAILED\n");

    // Test accessors.
    {
        EllipsoidJoint myEllipsoidJt;

        ASSERT(myEllipsoidJt.getCoordinate(EllipsoidJoint::Coord::Rotation1X) ==
                        myEllipsoidJt.get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myEllipsoidJt.getCoordinate(EllipsoidJoint::Coord::Rotation2Y) ==
                        myEllipsoidJt.get_coordinates(1),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myEllipsoidJt.getCoordinate(EllipsoidJoint::Coord::Rotation3Z) ==
                        myEllipsoidJt.get_coordinates(2),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myEllipsoidJt.getCoordinate());

        ASSERT(myEllipsoidJt.updCoordinate(EllipsoidJoint::Coord::Rotation1X) ==
                        myEllipsoidJt.upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myEllipsoidJt.updCoordinate());
    }

} // end testEllipsoidJoint

// Compare behavior of a double pendulum (1) with welded foot and toes
TEST_CASE("testWeldJoint unrandomized") {
    testWeldJoint(false);
}

// Compare previous OpenSim model but with randomized body order in BodySet
// to test connectBodies
TEST_CASE("testWeldJoint randomized") {
    testWeldJoint(true);
}

/// Compare behavior of a Free hip and pin knee
/// OpenSim, system restricted to using Euler angles exclusively to support
/// EllipsoidJoint and the fact that coordinates cannot map to/from
/// quaternions
TEST_CASE("testFreeJoint") {
    using namespace SimTK;

    cout << endl;
    cout << "============================================================="
         << endl;
    cout << " OpenSim FreeJoint vs. Simbody MobilizedBody::Free " << endl;
    cout << "============================================================="
         << endl;

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Thigh connected by hip
    MobilizedBody::Free thigh(matter.Ground(), SimTK::Transform(hipInPelvis),
            SimTK::Body::Rigid(femurMass), SimTK::Transform(hipInFemur));
    // Function-based knee connects shank
    MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur),
            SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================
    // Setup OpenSim model
    Model* osimModel = new Model;
    // OpenSim bodies
    const Ground& ground = osimModel->getGround();
    ;

    // OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    // create free hip joint
    FreeJoint hip("hip", ground, hipInPelvis,
            Vec3(0), osim_thigh, hipInFemur,
            Vec3(0));

    // Rename hip coordinates for a free joint
    for (int i = 0; i < hip.numCoordinates(); i++) {
        std::stringstream coord_name;
        coord_name << "hip_q" << i;
        hip.upd_coordinates(i).setName(coord_name.str());
    }

    // Add the thigh body which now also contains the hip joint to the model
    osimModel->addBody(&osim_thigh);
    osimModel->addJoint(&hip);

    // create OpenSim shank body
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // Define knee transform for flexion/extension
    SpatialTransform kneeTransform;
    kneeTransform[2].setCoordinateNames(
            OpenSim::Array<std::string>("knee_q", 1, 1));
    kneeTransform[2].setFunction(new LinearFunction());

    // create custom knee joint
    CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank,
            kneeInTibia, Vec3(0), kneeTransform);

    // Add the shank body which now also contains the knee joint to the model
    osimModel->addBody(&osim_shank);
    osimModel->addJoint(&knee);

    // BAD: have to set memoryOwner to false or program will crash when this
    // test is complete.
    osimModel->disownAllComponents();

    osimModel->setGravity(gravity_vec);

    cout << "testFreeJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(*osimModel);

    SimTK::State osim_state = osimModel->initSystem();

    cout << "NQ_osim = " << osim_state.getNQ()
         << "   NQ_simbody = " << state.getNQ() << endl;

    osimModel->print("testFreeJoint.osim");

    //==========================================================================
    // Compare Simbody system and OpenSim model simulations
    stringstream errorMessage;
    errorMessage << "testFreeJoint using Euler angles FAILED\n";
    compareSimulations(
            system, state, osimModel, osim_state, errorMessage.str());

    // Test accessors.
    {
        FreeJoint myFreeJoint;

        ASSERT(myFreeJoint.getCoordinate(FreeJoint::Coord::Rotation1X) ==
                        myFreeJoint.get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myFreeJoint.getCoordinate(FreeJoint::Coord::Rotation2Y) ==
                        myFreeJoint.get_coordinates(1),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myFreeJoint.getCoordinate(FreeJoint::Coord::Rotation3Z) ==
                        myFreeJoint.get_coordinates(2),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myFreeJoint.getCoordinate(FreeJoint::Coord::TranslationX) ==
                        myFreeJoint.get_coordinates(3),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myFreeJoint.getCoordinate(FreeJoint::Coord::TranslationY) ==
                        myFreeJoint.get_coordinates(4),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myFreeJoint.getCoordinate(FreeJoint::Coord::TranslationZ) ==
                        myFreeJoint.get_coordinates(5),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myFreeJoint.getCoordinate());

        ASSERT(myFreeJoint.updCoordinate(FreeJoint::Coord::Rotation1X) ==
                        myFreeJoint.upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myFreeJoint.updCoordinate());
    }

} // end testFreeJoint

TEST_CASE("testScapulothoracicJoint") {
    cout << endl;
    cout << "============================================================="
         << endl;
    cout << "    ScapulothoracicJoint Scaling and Regression Tests " << endl;
    cout << "============================================================="
         << endl;
    // can default/value instantiate non-throwingly
    {
        ScapulothoracicJoint sj1;
        ScapulothoracicJoint sj2{};
    }

    // Test that we can instantiate in a typical way (e.g. how a user would)
    // without throwing an exception. 
    // Then verify that ScapulothoracicJoint scales appropriately.
    {
        Model m;
        m.setName(__FUNCTION__);

        auto thorax = new Body{"thorax", femurMass.getMass(),
                femurMass.getMassCenter(), femurMass.getInertia()};

        auto scapula = new Body{"scapula", footMass.getMass(),
                footMass.getMassCenter(), footMass.getInertia()};

        m.addBody(thorax);
        m.addBody(scapula);

        SimTK::Vec2 unscaled_XY{0.1, 0.1};

        ScapulothoracicJoint* scapulothoracicJoint = new ScapulothoracicJoint{
                "scapulothoracic_joint",
                *thorax, SimTK::Vec3{0.05}, SimTK::Vec3{0},
                *scapula, SimTK::Vec3{0}, SimTK::Vec3{0},
                SimTK::Vec3{1.0, 1.0, 1.0},     // ellipsoid radii
                unscaled_XY,                    // winging origin
                0.5                             // winging direction
        };
        m.addJoint(scapulothoracicJoint);

        SimTK::State& s = m.initSystem();

        ScaleSet scaleFactors{};

        Scale* scaleThorax = new Scale();
        scaleThorax->setSegmentName("thorax");
        SimTK::Vec3 thoraxFactors{1.1, 0.95, 1.05};
        scaleThorax->setScaleFactors(thoraxFactors);

        Scale* scaleScapula = new Scale();
        scaleScapula->setSegmentName("scapula");
        SimTK::Vec3 scapulaFactors{1.2, 0.9, 1.15};
        scaleScapula->setScaleFactors(scapulaFactors);

        scaleFactors.adoptAndAppend(scaleThorax);
        scaleFactors.adoptAndAppend(scaleScapula);

        m.scale(s, scaleFactors, false);

        const SimTK::Vec3& ellipsoidRadii =
                scapulothoracicJoint->get_thoracic_ellipsoid_radii_x_y_z();

        ASSERT_EQUAL(ellipsoidRadii, thoraxFactors, SimTK::Eps,
            __FILE__, __LINE__,
            "ScapulothoracicJoint failed to scale ellipsoid radii correctly.");

        const double& origin_X =
                scapulothoracicJoint->get_scapula_winging_axis_origin(0);

        const double& origin_Y =
                scapulothoracicJoint->get_scapula_winging_axis_origin(1);

        ASSERT_EQUAL(origin_X, unscaled_XY[0] * scapulaFactors[0], SimTK::Eps,
                __FILE__, __LINE__,
                "ScapulothoracicJoint failed to scale origin X location.");

        ASSERT_EQUAL(origin_Y, unscaled_XY[1] * scapulaFactors[1], SimTK::Eps,
                __FILE__, __LINE__,
                "ScapulothoracicJoint failed scale to origin Y location.");
    }
}

/// Compare behavior of a double pendulum with an OpenSim Ball hip and custom
/// pin knee OpenSim, system restricted to using Euler angles exclusively to
/// support EllipsoidJoint and the fact that coordinates cannot map to/from
/// quaternions
TEST_CASE("testBallJoint") {
    using namespace SimTK;

    cout << endl;
    cout << "============================================================="
         << endl;
    cout << " OpenSim BallJoint vs. Simbody MobilizedBody::Ball " << endl;
    cout << "============================================================="
         << endl;

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Thigh connected by hip
    MobilizedBody::Ball thigh(matter.Ground(), SimTK::Transform(hipInPelvis),
            SimTK::Body::Rigid(femurMass), SimTK::Transform(hipInFemur));
    // Function-based knee connects shank
    MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur),
            SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================
    // Setup OpenSim model
    Model osimModel;
    // OpenSim bodies
    const Ground& ground = osimModel.getGround();

    // OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    // create hip as an Ball joint
    BallJoint hip("hip", ground, hipInPelvis,
            Vec3(0), osim_thigh, hipInFemur,
            Vec3(0));

    // Rename hip coordinates for a ball joint
    for (int i = 0; i < hip.numCoordinates(); i++) {
        std::stringstream coord_name;
        coord_name << "hip_q" << i;
        hip.upd_coordinates(i).setName(coord_name.str());
    }

    // Add the thigh body which now also contains the hip joint to the model
    osimModel.addBody(&osim_thigh);
    osimModel.addJoint(&hip);

    // Add OpenSim shank via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // create pin knee joint
    PinJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank,
            kneeInTibia, Vec3(0));

    // Rename knee coordinates for a pin joint
    knee.upd_coordinates(0).setName("knee_q");

    // Add the shank body which now also contains the knee joint to the model
    osimModel.addBody(&osim_shank);
    osimModel.addJoint(&knee);

    // BAD: have to set memoryOwner to false or program will crash when this
    // test is complete.
    osimModel.disownAllComponents();

    osimModel.setGravity(gravity_vec);

    cout << "testBallJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(osimModel);

    SimTK::State osim_state = osimModel.initSystem();

    //==========================================================================
    // Compare Simbody system and OpenSim model simulations
    stringstream errorMessage;
    errorMessage << "testBallJoint using Euler angles FAILED\n";
    compareSimulations(
            system, state, &osimModel, osim_state, errorMessage.str());

    // Test accessors.
    {
        BallJoint myBallJoint;

        ASSERT(myBallJoint.getCoordinate(BallJoint::Coord::Rotation1X) ==
                        myBallJoint.get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myBallJoint.getCoordinate(BallJoint::Coord::Rotation2Y) ==
                        myBallJoint.get_coordinates(1),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myBallJoint.getCoordinate(BallJoint::Coord::Rotation3Z) ==
                        myBallJoint.get_coordinates(2),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myBallJoint.getCoordinate());

        ASSERT(myBallJoint.updCoordinate(BallJoint::Coord::Rotation1X) ==
                        myBallJoint.upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myBallJoint.updCoordinate());
    }

} // end testBallJoint

/// Compare behavior of a double pendulum with OpenSim pin hip and pin knee
TEST_CASE("testPinJoint") {
    using namespace SimTK;

    cout << endl;
    cout << "============================================================="
         << endl;
    cout << " OpenSim PinJoint vs. Simbody MobilizedBody::Pin " << endl;
    cout << "============================================================="
         << endl;

    Random::Uniform randomAngle(-Pi / 2, Pi / 2);
    Vec3 oInB(randomAngle.getValue(), randomAngle.getValue(),
            randomAngle.getValue());
    Vec3 oInP(randomAngle.getValue(), randomAngle.getValue(),
            randomAngle.getValue());

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Thigh connected by hip
    MobilizedBody::Pin thigh(matter.Ground(), SimTK::Transform(hipInPelvis),
            SimTK::Body::Rigid(femurMass), SimTK::Transform(hipInFemur));
    // Pin knee connects shank
    MobilizedBody::Pin shank(thigh,
            SimTK::Transform(Rotation(BodyRotationSequence, oInP[0], XAxis,
                                     oInP[1], YAxis, oInP[2], ZAxis),
                    kneeInFemur),
            SimTK::Body::Rigid(tibiaMass),
            SimTK::Transform(Rotation(BodyRotationSequence, oInB[0], XAxis,
                                     oInB[1], YAxis, oInB[2], ZAxis),
                    kneeInTibia));

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //=========================================================================
    // Setup OpenSim model
    Model* osimModel = new Model;
    // OpenSim bodies
    const Ground& ground = osimModel->getGround();

    // OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    // Add the thigh body to the model
    osimModel->addBody(&osim_thigh);

    // create hip as a pin joint
    PinJoint hip("hip", ground, hipInPelvis,
            Vec3(0), osim_thigh, hipInFemur,
            Vec3(0));

    // Rename hip coordinates for a pin joint
    for (int i = 0; i < hip.numCoordinates(); i++) {
        std::stringstream coord_name;
        coord_name << "hip_q" << i;
        hip.upd_coordinates(i).setName(coord_name.str());
    }

    // Add hip joint to the model
    osimModel->addJoint(&hip);

    // Add OpenSim shank via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // Add the shank body to the model
    osimModel->addBody(&osim_shank);

    // create pin knee joint
    PinJoint knee("knee", osim_thigh, kneeInFemur, oInP, osim_shank,
            kneeInTibia, oInB);
    knee.upd_coordinates(0).setName("knee_q");

    // verify that default copy constructor handles coordinates appropriately
    auto knee2(knee);
    ASSERT(knee2.numCoordinates() == knee.numCoordinates());
    ASSERT(knee2.get_coordinates(0).getName() == "knee_q");

    auto* thigh_offset = new PhysicalOffsetFrame("thigh_offset", osim_thigh,
            SimTK::Transform(Rotation(BodyRotationSequence, oInP[0], XAxis,
                                     oInP[1], YAxis, oInP[2], ZAxis),
                    kneeInFemur));

    auto* shank_offset = new PhysicalOffsetFrame("shank_offset", osim_shank,
            SimTK::Transform(Rotation(BodyRotationSequence, oInB[0], XAxis,
                                     oInB[1], YAxis, oInB[2], ZAxis),
                    kneeInTibia));

    // Exercise new convenience constructor with common use case of adding
    // offsets to the body of interest
    PinJoint knee3("knee", *thigh_offset, *shank_offset);
    knee3.addFrame(thigh_offset);
    knee3.addFrame(shank_offset);
    // use the same coordinate name
    knee3.upd_coordinates(0).setName("knee_q");

    // once connected the two ways of constructing the knee joint should
    // yield identical definitions
    ASSERT(knee3 == knee);

    // Adding the offsets to the bodies instead of the joint should not change
    // the resulting system and results
    osimModel->addJoint(&knee3);

    knee3.printSocketInfo();
    knee3.printInputInfo();
    knee.printSocketInfo();
    knee.printInputInfo();

    // BAD: have to set memoryOwner to false or program will crash when this
    // test is complete.
    osimModel->disownAllComponents();

    osimModel->setGravity(gravity_vec);
    osimModel->finalizeFromProperties();

    testEquivalentBodyForceForGenForces(*osimModel);

    // Initialize the system and get the model state
    SimTK::State osim_state = osimModel->initSystem();

    // double check that the deserialized model is identical to the live model
    osimModel->print("testPinJointModel.osim");
    Model* osimModelcopy = new Model("testPinJointModel.osim");
    ASSERT(*osimModel == *osimModelcopy);
    SimTK::State copy_state = osimModelcopy->initSystem();

    SimTK_TEST_EQ(osim_state.getQ(), copy_state.getQ());

    //==========================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(
            system, state, osimModel, osim_state, "testPinJoint FAILED\n");

    // Test accessors.
    {
        PinJoint myPinJoint;

        ASSERT(myPinJoint.getCoordinate(PinJoint::Coord::RotationZ) ==
                        myPinJoint.get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myPinJoint.getCoordinate() == myPinJoint.get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");

        ASSERT(myPinJoint.updCoordinate(PinJoint::Coord::RotationZ) ==
                        myPinJoint.upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myPinJoint.updCoordinate() == myPinJoint.upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
    }

} // end testPinJoint

/// Compare behavior of a two body pendulum with OpenSim pin hip and slider knee
TEST_CASE("testSliderJoint") {
    using namespace SimTK;

    cout << endl;
    cout << "============================================================="
         << endl;
    cout << " OpenSim SliderJoint vs. Simbody MobilizedBody::Slider " << endl;
    cout << "============================================================="
         << endl;

    Random::Uniform randomAngle(-Pi / 2, Pi / 2);
    Vec3 oInB(randomAngle.getValue(), randomAngle.getValue(),
            randomAngle.getValue());
    Vec3 oInP(randomAngle.getValue(), randomAngle.getValue(),
            randomAngle.getValue());

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Thigh connected by hip
    MobilizedBody::Pin thigh(matter.Ground(), SimTK::Transform(hipInPelvis),
            SimTK::Body::Rigid(femurMass), SimTK::Transform(hipInFemur));
    // Pin knee connects shank
    MobilizedBody::Slider shank(thigh,
            SimTK::Transform(Rotation(BodyRotationSequence, oInP[0], XAxis,
                                     oInP[1], YAxis, oInP[2], ZAxis),
                    kneeInFemur),
            SimTK::Body::Rigid(tibiaMass),
            SimTK::Transform(Rotation(BodyRotationSequence, oInB[0], XAxis,
                                     oInB[1], YAxis, oInB[2], ZAxis),
                    kneeInTibia));

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================
    // Setup OpenSim model
    Model osimModel;
    // OpenSim bodies
    const Ground& ground = osimModel.getGround();

    // OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    // create hip as an Ball joint
    PinJoint hip("hip", ground, hipInPelvis,
            Vec3(0), osim_thigh, hipInFemur,
            Vec3(0));

    // Rename hip coordinates for a pin joint
    for (int i = 0; i < hip.numCoordinates(); i++) {
        std::stringstream coord_name;
        coord_name << "hip_q" << i;
        hip.upd_coordinates(i).setName(coord_name.str());
    }

    // Add the thigh body which now also contains the hip joint to the model
    osimModel.addBody(&osim_thigh);
    osimModel.addJoint(&hip);

    // Add OpenSim shank via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // create slider knee joint
    SliderJoint knee("knee", osim_thigh, kneeInFemur, oInP, osim_shank,
            kneeInTibia, oInB);

    knee.upd_coordinates(0).setName("knee_qx");

    // Add the shank body which now also contains the knee joint to the model
    osimModel.addBody(&osim_shank);
    osimModel.addJoint(&knee);

    // BAD: have to set memoryOwner to false or program will crash when this
    // test is complete.
    osimModel.disownAllComponents();

    osimModel.setGravity(gravity_vec);

    osimModel.print("testSliderJointModel.osim");

    cout << "testSliderJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(osimModel);

    // Need to setup model before adding an analysis since it creates the
    // AnalysisSet for the model if it does not exist.
    SimTK::State osim_state = osimModel.initSystem();

    //==========================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(
            system, state, &osimModel, osim_state, "testSliderJoint FAILED\n");

    // Test accessors.
    {
        SliderJoint mySliderJoint;

        ASSERT(mySliderJoint.getCoordinate(SliderJoint::Coord::TranslationX) ==
                        mySliderJoint.get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(mySliderJoint.getCoordinate() ==
                        mySliderJoint.get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");

        ASSERT(mySliderJoint.updCoordinate(SliderJoint::Coord::TranslationX) ==
                        mySliderJoint.upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(mySliderJoint.updCoordinate() ==
                        mySliderJoint.upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
    }

} // end testSliderJoint

/// Compare behavior of a two body model connected via planar joints
TEST_CASE("testPlanarJoint") {
    using namespace SimTK;

    cout << endl;
    cout << "============================================================="
         << endl;
    cout << " OpenSim PlanarJoint vs. Simbody MobilizedBody::Planar " << endl;
    cout << "============================================================="
         << endl;

    Random::Uniform randomAngle(-Pi / 2, Pi / 2);
    Vec3 oInB(randomAngle.getValue(), randomAngle.getValue(),
            randomAngle.getValue());
    Vec3 oInP(randomAngle.getValue(), randomAngle.getValue(),
            randomAngle.getValue());

    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Thigh connected by hip
    MobilizedBody::Planar thigh(matter.Ground(), SimTK::Transform(hipInPelvis),
            SimTK::Body::Rigid(femurMass), SimTK::Transform(hipInFemur));
    // Pin knee connects shank
    MobilizedBody::Planar shank(thigh,
            SimTK::Transform(Rotation(BodyRotationSequence, oInP[0], XAxis,
                                     oInP[1], YAxis, oInP[2], ZAxis),
                    kneeInFemur),
            SimTK::Body::Rigid(tibiaMass),
            SimTK::Transform(Rotation(BodyRotationSequence, oInB[0], XAxis,
                                     oInB[1], YAxis, oInB[2], ZAxis),
                    kneeInTibia));

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================
    // Setup OpenSim model
    Model osimModel;
    // OpenSim bodies
    const Ground& ground = osimModel.getGround();

    // OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    // create hip as an Ball joint
    PlanarJoint hip("hip", ground, hipInPelvis,
            Vec3(0), osim_thigh, hipInFemur,
            Vec3(0));

    // Rename hip coordinates for a pin joint
    for (int i = 0; i < hip.numCoordinates(); i++) {
        std::stringstream coord_name;
        coord_name << "hip_q" << i;
        hip.upd_coordinates(i).setName(coord_name.str());
    }

    // Add the thigh body which now also contains the hip joint to the model
    osimModel.addBody(&osim_thigh);
    osimModel.addJoint(&hip);

    // Add OpenSim shank via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // create planar knee joint
    PlanarJoint knee("knee", osim_thigh, kneeInFemur, oInP, osim_shank,
            kneeInTibia, oInB);

    knee.upd_coordinates(0).setName("knee_rz");
    knee.upd_coordinates(1).setName("knee_tx");
    knee.upd_coordinates(2).setName("knee_ty");

    // Add the shank body which now also contains the knee joint to the model
    osimModel.addBody(&osim_shank);
    osimModel.addJoint(&knee);

    // BAD: have to set memoryOwner to false or program will crash when this
    // test is complete.
    osimModel.disownAllComponents();

    osimModel.setGravity(gravity_vec);

    cout << "testPlanarJoint: testEquivalentBodyForceForGenForces" << endl;
    testEquivalentBodyForceForGenForces(osimModel);

    // Need to setup model before adding an analysis since it creates the
    // AnalysisSet for the model if it does not exist.
    SimTK::State osim_state = osimModel.initSystem();

    //==========================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(
            system, state, &osimModel, osim_state, "testPlanarJoint FAILED\n");

    // Test accessors.
    {
        PlanarJoint myPlanarJoint;

        ASSERT(myPlanarJoint.getCoordinate(PlanarJoint::Coord::RotationZ) ==
                        myPlanarJoint.get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myPlanarJoint.getCoordinate(PlanarJoint::Coord::TranslationX) ==
                        myPlanarJoint.get_coordinates(1),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myPlanarJoint.getCoordinate(PlanarJoint::Coord::TranslationY) ==
                        myPlanarJoint.get_coordinates(2),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myPlanarJoint.getCoordinate());

        ASSERT(myPlanarJoint.updCoordinate(PlanarJoint::Coord::RotationZ) ==
                        myPlanarJoint.upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myPlanarJoint.updCoordinate());
    }

} // end testPlanarJoint

/// Compare behavior of a Free hip and pin knee
TEST_CASE("testCustomWithMultidimFunction") {
    using namespace SimTK;

    cout << endl;
    cout << "=========================================================="
         << endl;
    cout << " OpenSim CustomJoint with Multidimensional Function " << endl;
    cout << "=========================================================="
         << endl;

    MultidimensionalFunction testFunction;
    // Define the Simbody system
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    SimTK::Force::UniformGravity gravity(forces, matter, gravity_vec);

    // Define the functions that specify the FunctionBased Mobilized Body.
    std::vector<std::vector<int>> coordIndices;
    std::vector<const SimTK::Function*> functions;
    std::vector<bool> isdof(6, false);

    // Set the 1 spatial rotation about Z-axis
    isdof[2] = false; // rot Z
    isdof[3] = true;  // trans X
    isdof[4] = true;  // trans Y

    int nm = 2;
    for (int i = 0; i < 6; i++) {
        if (i == 2) { // rotation coupled to translation dofs
            std::vector<int> findex(2, 0);
            findex[1] = 1;
            functions.push_back(testFunction.createSimTKFunction());
            coordIndices.push_back(findex);
        } else if (isdof[i]) {
            Vector coeff(2);
            coeff[0] = 1;
            coeff[1] = 0;
            std::vector<int> findex(1, i - 3);
            functions.push_back(new SimTK::Function::Linear(coeff));
            coordIndices.push_back(findex);
        } else {
            std::vector<int> findex(0);
            functions.push_back(new SimTK::Function::Constant(0, 0));
            coordIndices.push_back(findex);
        }
    }

    // Thigh connected by hip
    MobilizedBody::FunctionBased thigh(matter.Ground(),
            SimTK::Transform(hipInPelvis), SimTK::Body::Rigid(femurMass),
            SimTK::Transform(hipInFemur), nm, functions, coordIndices);
    // Function-based knee connects shank
    MobilizedBody::Pin shank(thigh, SimTK::Transform(kneeInFemur),
            SimTK::Body::Rigid(tibiaMass), SimTK::Transform(kneeInTibia));

    // Simbody model state setup
    system.realizeTopology();
    State state = system.getDefaultState();
    matter.setUseEulerAngles(state, true);
    system.realizeModel(state);

    //==========================================================================
    // Setup OpenSim model
    Model osimModel; // = new Model;

    // OpenSim bodies
    const Ground& ground = osimModel.getGround();
    // OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    // Define hip coordinates and axes for custom joint
    SpatialTransform hipTransform;
    OpenSim::Array<std::string> coordNames;
    coordNames.append("hip_qx");
    coordNames.append("hip_qy");
    hipTransform[2].setCoordinateNames(coordNames);
    hipTransform[2].setFunction(new MultidimensionalFunction());
    hipTransform[3].setCoordinateNames(
            OpenSim::Array<std::string>(coordNames[0], 1, 1));
    hipTransform[3].setFunction(new LinearFunction());
    hipTransform[4].setCoordinateNames(
            OpenSim::Array<std::string>(coordNames[1], 1, 1));
    hipTransform[4].setFunction(new LinearFunction());

    // create custom hip joint
    CustomJoint hip("hip", ground, hipInPelvis,
            Vec3(0), osim_thigh, hipInFemur,
            Vec3(0), hipTransform);

    // Add the thigh body which now also contains the hip joint to the model
    osimModel.addBody(&osim_thigh);
    osimModel.addJoint(&hip);

    // Add OpenSim shank via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    // Define knee coordinates and axes for custom joint spatial transform
    SpatialTransform kneeTransform;
    string coord_name = "knee_q";
    // Only knee flexion/extension
    kneeTransform[2].setCoordinateNames(
            OpenSim::Array<std::string>(coord_name, 1, 1));
    kneeTransform[2].setFunction(new LinearFunction());

    // create custom knee joint
    CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank,
            kneeInTibia, Vec3(0), kneeTransform);

    // Add the shank body which now also contains the knee joint to the model
    osimModel.addBody(&osim_shank);
    osimModel.addJoint(&knee);

    // BAD: have to set memoryOwner to false or program will crash when this
    // test is complete.
    osimModel.disownAllComponents();
    osimModel.setGravity(gravity_vec);

    cout << "testCustomWithMultidimFunction: "
            "testEquivalentBodyForceForGenForces"
         << endl;
    testEquivalentBodyForceForGenForces(osimModel);

    SimTK::State osim_state = osimModel.initSystem();

    //==========================================================================
    // Compare Simbody system and OpenSim model simulations
    compareSimulations(system, state, &osimModel, osim_state,
            "testCustomWithMultidimFunction FAILED\n");

} // end of testCustomWithMultidimFunction

/// Compare custom implementation of Gimbal to a Compound (multi-mobilizer)
/// Joint version
TEST_CASE("testCustomVsCompoundJoint") {
    using namespace SimTK;

    cout << endl;
    cout << "=========================================================="
         << endl;
    cout << " OpenSim CustomJoint vs. CompoundJoint for a ZXY Gimbal " << endl;
    cout << "=========================================================="
         << endl;

    //==========================================================================
    // Setup CustomJoint model
    //==========================================================================
    // Register new Joint types for testing
    Object::registerType(CompoundJoint());

    Model customModel;

    // OpenSim bodies
    const Ground& ground = customModel.getGround();
    // OpenSim thigh
    OpenSim::Body osim_thigh("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());
    osim_thigh.attachGeometry(new Mesh("femur.vtp"));

    // Define hip transform in terms of coordinates and axes for custom joint
    SpatialTransform hipTransform;
    hipTransform[0].setCoordinateNames(
            OpenSim::Array<std::string>("hip_qZ", 1, 1));
    hipTransform[0].setFunction(new LinearFunction());
    hipTransform[0].setAxis(Vec3(0, 0, 1));
    hipTransform[1].setCoordinateNames(
            OpenSim::Array<std::string>("hip_qX", 1, 1));
    hipTransform[1].setFunction(new LinearFunction());
    hipTransform[1].setAxis(Vec3(1, 0, 0));
    hipTransform[2].setCoordinateNames(
            OpenSim::Array<std::string>("hip_qY", 1, 1));
    hipTransform[2].setFunction(new LinearFunction());
    hipTransform[2].setAxis(Vec3(0, 1, 0));

    Random::Uniform randomAngle(-Pi / 2, Pi / 2);
    Vec3 oInP(randomAngle.getValue(), randomAngle.getValue(),
            randomAngle.getValue());
    Vec3 oInB(randomAngle.getValue(), randomAngle.getValue(),
            randomAngle.getValue());

    // create custom hip joint
    CustomJoint hip("hip", ground, hipInPelvis,
            oInP, osim_thigh, hipInFemur,
            oInB, hipTransform);

    // Add the thigh body which now also contains the hip joint to the model
    customModel.addBody(&osim_thigh);
    customModel.addJoint(&hip);

    // Add OpenSim shank via a knee joint
    OpenSim::Body osim_shank("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());
    osim_shank.attachGeometry(new Mesh("tibia.vtp"));

    // Define knee coordinates and axes for custom joint spatial transform
    SpatialTransform kneeTransform;
    string knee_rot = "knee_ext";
    // Only knee flexion/extension
    kneeTransform[2].setCoordinateNames(
            OpenSim::Array<std::string>(knee_rot, 1, 1));
    kneeTransform[2].setFunction(new LinearFunction());

    // create custom knee joint
    CustomJoint knee("knee", osim_thigh, kneeInFemur, Vec3(0), osim_shank,
            kneeInTibia, Vec3(0), kneeTransform);

    // Add the shank body which now also contains the knee joint to the model
    customModel.addBody(&osim_shank);
    customModel.addJoint(&knee);

    // Set gravity
    customModel.setGravity(gravity_vec);
    customModel.disownAllComponents();

    // Need to setup model before adding an analysis since it creates the
    // AnalysisSet for the model if it does not exist.
    SimTK::State state1 = customModel.initSystem();

    //==========================================================================
    // Setup CompoundJointed model
    //==========================================================================
    Model compoundModel;

    // OpenSim bodies
    const Ground& ground2 = compoundModel.getGround();

    // OpenSim thigh
    OpenSim::Body thigh2("thigh2", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());
    thigh2.attachGeometry(new Mesh("femur.vtp"));

    // create compound hip joint
    CompoundJoint hip2(
            "hip2", ground2, hipInPelvis, oInP,
            thigh2, hipInFemur, oInB);

    // Add the thigh body which now also contains the hip joint to the model
    compoundModel.addBody(&thigh2);
    compoundModel.addJoint(&hip2);

    // Add OpenSim shank via a knee joint
    OpenSim::Body shank2("shank2", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());
    shank2.attachGeometry(new Mesh("tibia.vtp"));

    // create custom knee joint
    CustomJoint knee2("knee2", thigh2, kneeInFemur, Vec3(0), shank2,
            kneeInTibia, Vec3(0), kneeTransform);

    // Add the shank body which now also contains the knee joint to the model
    compoundModel.addBody(&shank2);
    compoundModel.addJoint(&knee2);

    // Set gravity
    compoundModel.setGravity(gravity_vec);

    compoundModel.disownAllComponents();

    // Need to setup model before adding an analysis since it creates the
    // AnalysisSet for the model if it does not exist.
    SimTK::State state2 = compoundModel.initSystem();
    SimTK::Vec3 com2 = compoundModel.calcMassCenterPosition(state2);

    // Test how default states are updated and serialized as part of properties
    state1.updQ()[0] = 0.1;
    state1.updQ()[1] = 0.2;
    state1.updQ()[2] = 0.3;
    state1.updQ()[3] = -Pi / 2;

    state2.updQ() = state1.getQ();

    customModel.setPropertiesFromState(state1);
    compoundModel.setPropertiesFromState(state2);

    customModel.print("Gimbal_CustomZXY_test.osim");
    compoundModel.print("Gimbal_CompoundPinsZXY_test.osim");

    /*SimTK::Vec3 com1 = */ customModel.calcMassCenterPosition(state1);
    com2 = compoundModel.calcMassCenterPosition(state2);

    //==========================================================================
    // Compare compound and custom joint model simulations
    compareSimulations(compoundModel.updMultibodySystem(), state2, &customModel,
            state1, "testCustomVsCoupleJoint FAILED\n");

} // end of testCustomVsCompoundJoint

TEST_CASE("testEquivalentBodyForceFromGeneralizedForce") {
    using namespace SimTK;

    cout << endl;
    cout << "=================================================================="
            "==="
         << endl;
    cout << " OpenSim test equivalent spatial body force from applied gen. "
            "force."
         << endl;
    cout << " Applied to all the joints of a gait model with coupler "
            "constraints."
         << endl;
    cout << "=================================================================="
            "==="
         << endl;

    // The following model(s) contains Actuators that are registered when the
    // osimActuators library is loaded. But unless we call at least one
    // function defined in the osimActuators library, some linkers will omit
    // its dependency from the executable and it will not be loaded at
    // startup.
    { PointActuator t; }

    Model gaitModel("testJointConstraints.osim");

    testEquivalentBodyForceForGenForces(gaitModel);

} // end testEquivalentBodyForceFromGeneralizedForce

/// model connect should create a FreeJoint for bodies that are not
/// connected by a Joint.
TEST_CASE("testAddedFreeJointForBodyWithoutJoint") {
    using namespace OpenSim;

    cout << endl;
    cout << "=========================================================="
         << endl;
    cout << " A Body without a Joint should get a Free (6dof) Joint    "
         << endl;
    cout << "=========================================================="
         << endl;

    Model model;
    SimTK::Inertia inertia(SimTK::Inertia::brick(SimTK::Vec3(0.5, 0.15, 0.2)));
    Body* block = new Body("block", 1.0, SimTK::Vec3(0.0), inertia);
    model.addBody(block);

    model.initSystem();

    ASSERT_EQUAL(6, model.getNumCoordinates());
    model.printBasicInfo();
}

/// model creation should automatically reverse joints to build a tree
/// but preserve the sense of the joint as specified by the user.
TEST_CASE("testAutomaticJointReversal") {
    using namespace OpenSim;

    cout << endl;
    cout << "==========================================================="
         << endl;
    cout << " Test Joint Reversal against not reversed with constraints    "
         << endl;
    cout << "==========================================================="
         << endl;

    //==========================================================================
    // Setup new OpenSim model
    Model model;
    model.setName("LegWithWeldedFoot_reverse");

    // OpenSim bodies
    Ground& ground = model.updGround();
    ground.upd_frame_geometry().setColor(SimTK::Vec3(1, 1, 0));

    auto pelvis = new Body("pelvis", 10.0, SimTK::Vec3(0),
            SimTK::Inertia::brick(SimTK::Vec3(0.1, 0.15, 0.25)));

    auto thigh = new Body("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());

    auto shank = new Body("shank", tibiaMass.getMass(),
            tibiaMass.getMassCenter(), tibiaMass.getInertia());

    auto foot = new Body("foot", footMass.getMass(), footMass.getMassCenter(),
            footMass.getInertia());

    pelvis->upd_frame_geometry().setColor(SimTK::Vec3(0, 1, 0)); // GREEN

    thigh->upd_frame_geometry().setColor(SimTK::Vec3(0, 0, 1)); // BLUE

    // ModelComponent::addGeometry makes a copy of the passed in Geometry
    shank->attachGeometry(new Cylinder(0.02, 0.243800));
    shank->upd_attached_geometry(0).setColor(SimTK::Vec3(0, 1, 1)); // CYAN
    foot->attachGeometry(new Brick(SimTK::Vec3(0.09, 0.025, 0.06)));
    foot->upd_attached_geometry(0).setColor(SimTK::Vec3(1, 0, 0)); // RED

    // add them to the model
    model.addBody(foot);
    model.addBody(shank);
    model.addBody(thigh);
    model.addBody(pelvis);

    // define some joints
    // Define hip coordinates and axes for custom joint
    SpatialTransform hipTransform;
    hipTransform[0].setCoordinateNames(
            OpenSim::Array<std::string>("hip_q0", 1, 1));
    hipTransform[0].setFunction(new LinearFunction());
    hipTransform[1].setCoordinateNames(
            OpenSim::Array<std::string>("hip_q1", 1, 1));
    hipTransform[1].setFunction(new LinearFunction());

    SimTK::Vec3 zvec(0);
    // create custom hip joint
    auto hip = new CustomJoint("hip", *pelvis,
            hipInPelvis, zvec, *thigh,
            hipInFemur, zvec, hipTransform);

    // Define knee transform for flexion/extension
    SpatialTransform kneeTransform;
    kneeTransform[2].setCoordinateNames(
            OpenSim::Array<std::string>("knee_q", 1, 1));
    kneeTransform[2].setFunction(new LinearFunction());

    // create custom knee joint
    auto knee = new CustomJoint("knee", *thigh,
            kneeInFemur, zvec, *shank,
            kneeInTibia, zvec, kneeTransform);

    auto ankle = new PinJoint(
            "ankle", *shank, ankleInTibia, zvec,
            *foot, ankleInFoot, zvec);

    auto footToFloor = new WeldJoint(
            "footToFloor", *foot, zvec, zvec,
            ground, footInGround, zvec);

    // add joints to the model
    model.addJoint(hip);
    model.addJoint(knee);
    model.addJoint(ankle);
    model.addJoint(footToFloor);

    // model.setUseVisualizer(true);
    SimTK::State& s = model.initSystem();

    SimTK::Transform pelvisX = pelvis->getTransformInGround(s);
    cout << "Pelvis Transform (reverse): " << pelvisX << endl;

    model.print("reversedLegWeldJointToGround.osim");

    // Test against an equivalent model with the foot constrained to ground
    // instead of having a WeldJoint to ground
    Model modelConstrained = model;
    modelConstrained.setName("LegWithConstrainedFoot");
    int rix = modelConstrained.updJointSet().getIndex("footToFloor");
    modelConstrained.updJointSet().remove(rix);
    modelConstrained.finalizeFromProperties();

    const Ground& cground = modelConstrained.getGround();
    const Body& cpelvis =
            modelConstrained.getComponent<Body>("./bodyset/pelvis");
    const Body& cfoot = modelConstrained.getComponent<Body>("./bodyset/foot");

    // free the pelvis
    auto pelvisFree = new FreeJoint(
            "pelvisFree", cground, zvec, zvec,
            cpelvis, zvec, zvec);

    modelConstrained.addJoint(pelvisFree);

    for (int i = 0; i < pelvisFree->numCoordinates(); ++i) {
        // don't tie assembly to coordinate default values
        pelvisFree->upd_coordinates(i).set_is_free_to_satisfy_constraints(true);
    }
    pelvisFree->upd_coordinates(3).setDefaultValue(pelvisX.p()[0]);
    pelvisFree->upd_coordinates(4).setDefaultValue(pelvisX.p()[1]);
    pelvisFree->upd_coordinates(5).setDefaultValue(pelvisX.p()[2]);

    auto footConstraint = new WeldConstraint(
            "footConstraint", cfoot, zvec, zvec,
            cground, footInGround, zvec);
    modelConstrained.addConstraint(footConstraint);

    auto fcpath = footConstraint->getRelativePathString(cfoot);

    auto& off1 = footConstraint->getFrame1();
    auto& sock1 = off1.getSocket<PhysicalFrame>("parent");
    auto& off2 = footConstraint->getFrame2();
    auto& sock2 = off2.getSocket<PhysicalFrame>("parent");

    auto off1Path = off1.getAbsolutePathString();
    auto off2Path = off2.getAbsolutePathString();

    /*auto& pathOff1 = */ sock1.getConnecteePath();
    /*auto& pathOff2 = */ sock2.getConnecteePath();

    auto relPathOff1 = cfoot.getRelativePathString(off1);
    auto relPathOff2 = cground.getRelativePathString(off2);

    // modelConstrained.setUseVisualizer(true);
    modelConstrained.printSubcomponentInfo();
    SimTK::State& sc = modelConstrained.initSystem();

    SimTK::Transform pelvisXc = cpelvis.getTransformInGround(sc);
    cout << "Shank Transform (constrained): " << pelvisXc << endl;

    modelConstrained.print("forwardLegWeldConstraintToGround.osim");

    std::vector<std::string> qNames{
            "hip_q0", "hip_q1", "knee_q", "ankle_coord_0"};

    for (unsigned i = 0; i < qNames.size(); ++i) {
        cout << "reversed " << qNames[i] << " = ";
        cout << model.getCoordinateSet().get(qNames[i]).getValue(s);
        cout << " constrained = ";
        cout << modelConstrained.getCoordinateSet().get(qNames[i]).getValue(sc)
             << endl;
    }

    cout << "Constrained Model Coordinates:" << endl;
    for (int i = 0; i < modelConstrained.getCoordinateSet().getSize(); ++i) {
        cout << modelConstrained.getCoordinateSet()[i].getName() << " = ";
        cout << modelConstrained.getCoordinateSet()[i].getValue(sc) << endl;
    }

    // The two systems should yield equivalent model dynamics
    integrateOpenSimModel(&model, s);
    integrateOpenSimModel(&modelConstrained, sc);

    double qknee = model.getCoordinateSet().get("knee_q").getValue(s);
    double qkneec =
            modelConstrained.getCoordinateSet().get("knee_q").getValue(sc);

    SimTK::Vec3 com = model.calcMassCenterPosition(s);
    SimTK::Vec3 comc = modelConstrained.calcMassCenterPosition(sc);

    ASSERT_EQUAL(qknee, qkneec, 10 * integ_accuracy);

    double distErr = (com - comc).norm();
    ASSERT_EQUAL(distErr, 0.0, 10 * integ_accuracy);

    SimTK::Vec3 acom = model.calcMassCenterAcceleration(s);
    SimTK::Vec3 acomc = modelConstrained.calcMassCenterAcceleration(sc);

    double accErr = ((acom - acomc).norm()) / (acom.norm() + SimTK::Eps);
    ASSERT_EQUAL(accErr, 0.0, sqrt(integ_accuracy));
}

/// The parent and child frames should be swapped if the "reverse" element
/// has been set to "true" in an old model file.
TEST_CASE("testUserJointReversal") {
    using namespace OpenSim;

    cout << "\n==========================================================="
         << "\n Test joint reversal set by user in old model file"
         << "\n==========================================================="
         << endl;

    // Open model.
    auto model = Model("double_pendulum_testReverse.osim");
    model.finalizeConnections(); // calls finalizeFromProperties internally

    // In this model file:
    // - pin1's parent is ground and child is rod1
    // - pin2's parent is rod1 and child is rod2
    // but the "reverse" element of pin2 is set to "true" so, after
    // deserialization, the following should be true:
    // - pin1's parent is ground and child is rod1
    // - pin2's parent is rod2 and child is rod1 (parent and child are swapped)
    auto& pin1 = model.getComponent<Joint>("./jointset/pin1");
    ASSERT(pin1.getParentFrame().findBaseFrame().getName() == "ground",
            __FILE__, __LINE__,
            "Incorrect parent frame when 'reverse' element is set to 'false'");
    ASSERT(pin1.getChildFrame().findBaseFrame().getName() == "rod1", __FILE__,
            __LINE__,
            "Incorrect child frame when 'reverse' element is set to 'false'");

    auto& pin2 = model.getComponent<Joint>("./jointset/pin2");
    ASSERT(pin2.getParentFrame().findBaseFrame().getName() == "rod2", __FILE__,
            __LINE__,
            "Incorrect parent frame when 'reverse' element is set to 'true'");
    ASSERT(pin2.getChildFrame().findBaseFrame().getName() == "rod1", __FILE__,
            __LINE__,
            "Incorrect child frame when 'reverse' element is set to 'true'");
}

/// test that kinematic loops are broken to form a tree with constraints
TEST_CASE("testAutomaticLoopJointBreaker") {
    cout << endl;
    cout << "==========================================================="
         << endl;
    cout << " Test Automatic Loop Joint Breaker  " << endl;
    cout << "==========================================================="
         << endl;
    // Setup OpenSim model
    Model model;
    // OpenSim bodies
    const Ground& ground = model.getGround();

    // OpenSim thigh
    OpenSim::Body thigh("thigh", femurMass.getMass(), femurMass.getMassCenter(),
            femurMass.getInertia());

    // Add the thigh body to the model
    model.addBody(&thigh);

    SimTK::Vec3 zvec(0);

    // create hip as an Ball joint
    BallJoint hip("hip", ground, hipInPelvis, zvec,
                thigh, hipInFemur, zvec);

    // Rename hip coordinates for a ball joint
    for (int i = 0; i < hip.numCoordinates(); i++) {
        std::stringstream coord_name;
        coord_name << "hip_q" << i;
        hip.upd_coordinates(i).setName(coord_name.str());
    }

    // Add the thigh hip to the model
    model.addJoint(&hip);

    // Add OpenSim shank via a knee joint
    Body shank("shank", tibiaMass.getMass(), tibiaMass.getMassCenter(),
            tibiaMass.getInertia());

    // Add the shank body to the model
    model.addBody(&shank);

    // create pin knee joint
    PinJoint knee("knee", thigh, kneeInFemur, zvec,
        shank, kneeInTibia, zvec);

    // Rename knee coordinate for a pin joint
    knee.upd_coordinates(0).setName("knee_q");

    // Add the knee joint to the model
    model.addJoint(&knee);

    // Add foot body at ankle
    Body foot("foot", footMass.getMass(), footMass.getMassCenter(),
            footMass.getInertia());

    // Add foot
    model.addBody(&foot);

    UniversalJoint ankle(
            "ankle", shank, ankleInTibia, zvec,
            foot, ankleInFoot, zvec);

    // Add ankle joint
    model.addJoint(&ankle);

    // Join the foot to the floor via a pin joint
    PinJoint footToFloor("footToFloor", foot, zvec, zvec,
        ground, zvec, zvec);

    // This forms the closed loop kinematic chain
    model.addJoint(&footToFloor);

    model.disownAllComponents();
    model.setGravity(gravity_vec);

    SimTK::State& s = model.initSystem();

    int ncoords = model.getNumCoordinates();
    int nconstraints = model.getNumConstraints();

    int nu = model.getMatterSubsystem().getNumMobilities();

    // User should get the dofs they requested
    ASSERT(ncoords == nu, __FILE__, __LINE__,
            "Multibody tree failed to preserve user-specified coordinates.");

    SimTK::Vec3 acc = model.calcMassCenterAcceleration(s);
    // number of active constraints
    int nc = model.getMatterSubsystem().getConstraintMultipliers(s).size();

    cout << "Number of model constraints:" << nconstraints
         << "  Number of system constraints: " << nc << endl;

    ASSERT(nc == 6, __FILE__, __LINE__,
            "Loop closure failed to adequately constrain tree.");

    std::string file("testModelWithLoopJoint.osim");
    model.print(file);

    Model loadedModel(file);
    SimTK::State& s2 = loadedModel.initSystem();

    /*int ncoords2 = */ loadedModel.getNumCoordinates();
    /*int nconstraints2 = */ loadedModel.getNumConstraints();

    ASSERT(model == loadedModel);

    model.print("testModelWithLoopJoint_loadedInitSys.osim");

    SimTK::Vec3 acc2 = model.calcMassCenterAcceleration(s2);

    ASSERT_EQUAL(acc2, acc, SimTK::Vec3(SimTK::Eps));
}

// Test accessors.
TEST_CASE("testCustomJointAccessors") {
    {
        Model myModel;
        SpatialTransform myTransform0; // 0 Coordinates
        CustomJoint* myCustomJoint0 = new CustomJoint("myCustomJoint0",
                myModel.getGround(), myModel.getGround(), myTransform0);
        myModel.addJoint(myCustomJoint0);

        ASSERT_THROW(OpenSim::JointHasNoCoordinates,
                myCustomJoint0->getCoordinate());
        ASSERT_THROW(OpenSim::JointHasNoCoordinates,
                myCustomJoint0->getCoordinate(0));
        ASSERT_THROW(OpenSim::JointHasNoCoordinates,
                myCustomJoint0->getCoordinate(1));

        ASSERT_THROW(OpenSim::JointHasNoCoordinates,
                myCustomJoint0->updCoordinate());
        ASSERT_THROW(OpenSim::JointHasNoCoordinates,
                myCustomJoint0->updCoordinate(0));
    }
    {
        Model myModel;
        SpatialTransform myTransform1; // 1 Coordinate
        myTransform1[0].setCoordinateNames(
                OpenSim::Array<std::string>("coord0", 1, 1));
        myTransform1[0].setFunction(new LinearFunction());
        CustomJoint* myCustomJoint1 = new CustomJoint("myCustomJoint1",
                myModel.getGround(), myModel.getGround(), myTransform1);
        myModel.addJoint(myCustomJoint1);

        ASSERT(myCustomJoint1->getCoordinate() ==
                        myCustomJoint1->get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myCustomJoint1->getCoordinate(0) ==
                        myCustomJoint1->get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myCustomJoint1->getCoordinate(1));

        ASSERT(myCustomJoint1->updCoordinate() ==
                        myCustomJoint1->upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myCustomJoint1->updCoordinate(0) ==
                        myCustomJoint1->upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myCustomJoint1->updCoordinate(1));
    }
    {
        Model myModel;
        SpatialTransform myTransform2; // 2 Coordinates
        myTransform2[0].setCoordinateNames(
                OpenSim::Array<std::string>("coord0", 1, 1));
        myTransform2[0].setFunction(new LinearFunction());
        myTransform2[1].setCoordinateNames(
                OpenSim::Array<std::string>("coord1", 1, 1));
        myTransform2[1].setFunction(new LinearFunction());
        CustomJoint* myCustomJoint2 = new CustomJoint("myCustomJoint2",
                myModel.getGround(), myModel.getGround(), myTransform2);
        myModel.addJoint(myCustomJoint2);

        ASSERT(myCustomJoint2->getCoordinate(0) ==
                        myCustomJoint2->get_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myCustomJoint2->getCoordinate(1) ==
                        myCustomJoint2->get_coordinates(1),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myCustomJoint2->getCoordinate());
        ASSERT_THROW(OpenSim::InvalidCall, myCustomJoint2->getCoordinate(2));

        ASSERT(myCustomJoint2->updCoordinate(0) ==
                        myCustomJoint2->upd_coordinates(0),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT(myCustomJoint2->updCoordinate(1) ==
                        myCustomJoint2->upd_coordinates(1),
                __FILE__, __LINE__, "Coordinate accessor failed");
        ASSERT_THROW(OpenSim::InvalidCall, myCustomJoint2->updCoordinate());
        ASSERT_THROW(OpenSim::InvalidCall, myCustomJoint2->updCoordinate(2));
    }
}

TEST_CASE("testGimbalJointAccessors") {
    GimbalJoint myGimbalJoint;

    ASSERT(myGimbalJoint.getCoordinate(GimbalJoint::Coord::Rotation1X) ==
                    myGimbalJoint.get_coordinates(0),
            __FILE__, __LINE__, "Coordinate accessor failed");
    ASSERT(myGimbalJoint.getCoordinate(GimbalJoint::Coord::Rotation2Y) ==
                    myGimbalJoint.get_coordinates(1),
            __FILE__, __LINE__, "Coordinate accessor failed");
    ASSERT(myGimbalJoint.getCoordinate(GimbalJoint::Coord::Rotation3Z) ==
                    myGimbalJoint.get_coordinates(2),
            __FILE__, __LINE__, "Coordinate accessor failed");
    ASSERT_THROW(OpenSim::InvalidCall, myGimbalJoint.getCoordinate());

    ASSERT(myGimbalJoint.updCoordinate(GimbalJoint::Coord::Rotation1X) ==
                    myGimbalJoint.upd_coordinates(0),
            __FILE__, __LINE__, "Coordinate accessor failed");
    ASSERT_THROW(OpenSim::InvalidCall, myGimbalJoint.updCoordinate());
}

TEST_CASE("testUniversalJointAccessors") {
    UniversalJoint myUniversalJoint;

    ASSERT(myUniversalJoint.getCoordinate(UniversalJoint::Coord::Rotation1X) ==
                    myUniversalJoint.get_coordinates(0),
            __FILE__, __LINE__, "Coordinate accessor failed");
    ASSERT(myUniversalJoint.getCoordinate(UniversalJoint::Coord::Rotation2Y) ==
                    myUniversalJoint.get_coordinates(1),
            __FILE__, __LINE__, "Coordinate accessor failed");
    ASSERT_THROW(OpenSim::InvalidCall, myUniversalJoint.getCoordinate());

    ASSERT(myUniversalJoint.updCoordinate(UniversalJoint::Coord::Rotation1X) ==
                    myUniversalJoint.upd_coordinates(0),
            __FILE__, __LINE__, "Coordinate accessor failed");
    ASSERT_THROW(OpenSim::InvalidCall, myUniversalJoint.updCoordinate());
}

/// Test that MotionTypes for Joint Coordinates are correctly defined
TEST_CASE("testMotionTypesForCustomJointCoordinates") {
    {
        Model osimModel;

        // OpenSim bodies
        const Ground& ground = osimModel.getGround();
        // OpenSim thigh
        auto osim_thigh = new OpenSim::Body("thigh", femurMass.getMass(),
                femurMass.getMassCenter(), femurMass.getInertia());

        // Define hip coordinates and axes for custom joint
        SpatialTransform hipTransform;
        OpenSim::Array<std::string> coordNames;
        coordNames.append("hip_qx");
        coordNames.append("hip_qy");
        hipTransform[2].setCoordinateNames(coordNames);
        hipTransform[2].setFunction(new MultidimensionalFunction());
        hipTransform[3].setCoordinateNames(
                OpenSim::Array<std::string>(coordNames[0], 1, 1));
        hipTransform[3].setFunction(new LinearFunction());
        hipTransform[4].setCoordinateNames(
                OpenSim::Array<std::string>(coordNames[1], 1, 1));
        hipTransform[4].setFunction(new LinearFunction(2.0, -0.5));
        // define a pure translational dof
        coordNames.append("hip_tz");
        hipTransform[5].setCoordinateNames(
                OpenSim::Array<std::string>(coordNames[2], 1, 1));
        hipTransform[5].setFunction(new LinearFunction());

        // define a pure rotational dof
        coordNames.append("hip_rx");
        hipTransform[0].setCoordinateNames(
                OpenSim::Array<std::string>(coordNames[3], 1, 1));
        hipTransform[0].setFunction(new LinearFunction());

        // create custom hip joint
        auto hip = new CustomJoint("hip", ground, hipInPelvis, SimTK::Vec3(0),
                *osim_thigh, hipInFemur, SimTK::Vec3(0), hipTransform);

        // Add the thigh body which now also contains the hip joint to the model
        osimModel.addBody(osim_thigh);
        osimModel.addJoint(hip);

        // hip_rx is the first coordinate and pure rotational about X
        auto coordName = hip->getCoordinate(0).getName();
        auto mt = hip->getCoordinate(0).getMotionType();
        ASSERT(mt == Coordinate::MotionType::Rotational, __FILE__, __LINE__,
                "Coordinate `" + coordName +
                        "' failed to register as MotionType::Rotational");

        // hip_qx is the second coordinate that influences Z rotation but is
        // pure translational along X
        coordName = hip->getCoordinate(1).getName();
        mt = hip->getCoordinate(1).getMotionType();
        ASSERT(mt == Coordinate::MotionType::Translational, __FILE__, __LINE__,
                "Coordinate `" + coordName +
                        "' failed to register as MotionType::Translational");

        // hip_qy is the third coordinate that also influences Z rotation but is
        // scaled to translate along Y and therefore NOT a pure translational
        // coordinate either
        coordName = hip->getCoordinate(2).getName();
        mt = hip->getCoordinate(2).getMotionType();
        ASSERT(mt == Coordinate::MotionType::Coupled, __FILE__, __LINE__,
                "Coordinate `" + coordName +
                        "' failed to register as MotionType::Coupled");

        // hip_tz is the fourth coordinate, which is pure translational along Z
        coordName = hip->getCoordinate(3).getName();
        mt = hip->getCoordinate(3).getMotionType();
        ASSERT(mt == Coordinate::MotionType::Translational, __FILE__, __LINE__,
                "Coordinate `" + coordName +
                        "' failed to register as MotionType::Translational");
    }
    {
        // Specifying a linear function with slope of -1 should still yield a
        // rotational coordinate.
        // This checks that Issue 2062 is fixed.
        // https://github.com/opensim-org/opensim-core/issues/2062.
        Model osimModel;

        // OpenSim bodies
        const Ground& ground = osimModel.getGround();
        // OpenSim thigh
        auto osim_thigh = new OpenSim::Body("thigh", femurMass.getMass(),
                femurMass.getMassCenter(), femurMass.getInertia());

        SpatialTransform hipTransform;
        std::string coordNameRX = "hip_rx";
        hipTransform[0].setCoordinateNames(
                OpenSim::Array<std::string>(coordNameRX, 1, 1));
        hipTransform[0].setFunction(new LinearFunction(-1.0, 0.0));

        std::string coordNameRY = "hip_ry";
        hipTransform[1].setCoordinateNames(
                OpenSim::Array<std::string>(coordNameRY, 1, 1));
        // A non-zero intercept does not prevent the coordinate from being
        // rotational.
        hipTransform[1].setFunction(new LinearFunction(-1.0, 8.313));

        // create custom hip joint
        auto hip = new CustomJoint("hip", ground, hipInPelvis, SimTK::Vec3(0),
                *osim_thigh, hipInFemur, SimTK::Vec3(0), hipTransform);

        // Add the thigh body which now also contains the hip joint to the model
        osimModel.addBody(osim_thigh);
        osimModel.addJoint(hip);

        // hip_rx is the first coordinate and pure rotational about X
        auto mt = hip->getCoordinate(0).getMotionType();
        ASSERT(mt == Coordinate::MotionType::Rotational, __FILE__, __LINE__,
                "Coordinate `" + coordNameRX +
                        "' failed to register as MotionType::Rotational");
        // hip_ry is the second coordinate and pure rotational about Y
        mt = hip->getCoordinate(1).getMotionType();
        ASSERT(mt == Coordinate::MotionType::Rotational, __FILE__, __LINE__,
                "Coordinate `" + coordNameRY +
                        "' failed to register as MotionType::Rotational");
    }
}

/// Test the assumption that a nonzero intercept of a linear function
/// for a transform axis of a CustomJoint acts as a simple offset of
/// the Coordinate value with otherwise identical dynamics
TEST_CASE("testNonzeroInterceptCustomJointVsPin") {
    using namespace SimTK;

    cout << endl;
    cout << "==========================================================="
         << endl;
    cout << " Test Nonzero Intercept of CustomJoint vs a PinJoint       "
         << endl;
    cout << "==========================================================="
         << endl;

    //=========================================================================
    // Setup OpenSim models
    Model pinModel;
    pinModel.setName("pin_based");

    // OpenSim thigh
    auto thigh1 = new OpenSim::Body("thigh", femurMass.getMass(),
            femurMass.getMassCenter(), femurMass.getInertia());
    thigh1->attachGeometry(
            new Cylinder(0.02, (hipInFemur - kneeInFemur).norm()));

    // Add the thigh body to the model
    pinModel.addBody(thigh1);

    // create hip as a pin joint
    auto hip1 = new PinJoint("hip", pinModel.getGround(), hipInPelvis, Vec3(0),
            *thigh1, hipInFemur, Vec3(0));
    hip1->updCoordinate().setName("pin_q");

    // Add pin hip joint to the model
    pinModel.addJoint(hip1);

    // Use a custom joint to model a pendulum "pin" joint
    Model cjModel;
    cjModel.setName("custom_joint_based");

    OpenSim::Body* thigh2 = thigh1->clone();

    // Add the thigh body to the model
    cjModel.addBody(thigh2);

    // offset the CustomJoint coordinate value by a fixed value
    double offset = 0.123456789;

    // Define hip coordinates and axes for custom joint
    SpatialTransform hipTransform;
    OpenSim::Array<std::string> coordNames;
    coordNames.append("cj_q");
    hipTransform[2].setCoordinateNames(coordNames);
    hipTransform[2].setFunction(new LinearFunction(1.0, offset));

    auto hip2 = new CustomJoint("hip", cjModel.getGround(), hipInPelvis,
            Vec3(0), *thigh2, hipInFemur, Vec3(0), hipTransform);

    // add CustomJoint "pin" to the model
    cjModel.addJoint(hip2);

    // pinModel.setUseVisualizer(true);
    // cjModel.setUseVisualizer(true);

    State s1 = pinModel.initSystem();
    State s2 = cjModel.initSystem();

    // Verify the MotionType for both Joints are the same (Rotational)
    auto mt1 = hip1->getCoordinate().getMotionType();
    auto mt2 = hip2->getCoordinate().getMotionType();
    ASSERT(mt1 == Coordinate::MotionType::Rotational, __FILE__, __LINE__,
            "PinJoint's Coordinate failed to have MotionType::Rotational");
    ASSERT(mt2 == mt1, __FILE__, __LINE__,
            "CustomJoint's Coordinate MotionType failed to match PinJoint's");

    // Set initial conditions of both pendulum models
    hip1->getCoordinate().setValue(s1, Pi / 3);
    // Subtract the expected offset of the cjModel since we expect the
    // CustomJoint's transformAxis' function to effectively offset the value
    hip2->getCoordinate().setValue(s2, Pi / 3 - offset);

    // integrate both Models over a standard duration
    integrateOpenSimModel(&pinModel, s1);
    integrateOpenSimModel(&cjModel, s2);

    // final coordinate values
    double pin_q = hip1->getCoordinate().getValue(s1);
    double cj_q = hip2->getCoordinate().getValue(s2);

    cout << "Pin final angle = " << pin_q << " vs. CustomJoint angle = " << cj_q
         << endl;
    cout << "Pin angle - CustomJoint angle = " << pin_q - cj_q
         << " vs. offset = " << offset << endl;

    ASSERT_EQUAL<double>(pin_q - cj_q, offset, integ_accuracy, __FILE__,
            __LINE__,
            "CustomJoint's linear function intercept failed to behave as an "
            "offset "
            "of the coordinate value.");
}

// reproduction to exercise the bug described in issue #3532
//
// the bug is that user code is permitted to delete/clear `OpenSim::Coordinate`s, such
// that the coordinate then does not have a minimum/maximum range, and that error state
// isn't detected until downstream code (e.g. calls `getMaxRange()` or similar)
TEST_CASE("testJointWithInvalidCoordinatesThrowsOnFinalization")
{
    OpenSim::Model model;

    auto body = std::unique_ptr<OpenSim::Body>(new OpenSim::Body{"body",
                1.0, SimTK::Vec3{}, SimTK::Inertia{}});
    auto joint = std::unique_ptr<OpenSim::PinJoint>(new OpenSim::PinJoint{});
    joint->setName("joint");
    joint->updCoordinate().setName("rotation");
    joint->connectSocket_parent_frame(model.getGround());
    joint->connectSocket_child_frame(*body);
    model.addJoint(joint.release());
    model.addBody(body.release());

    // should be fine: the model is correct
    model.finalizeConnections();

    auto& coord = model.updComponent<OpenSim::Coordinate>("/jointset/joint/rotation");

    // uh oh: a coordinate with no range (also applies when deleting only one element)
    coord.updProperty_range().clear();

    bool exceptionThrown = false;
    try {
        // should throw (the bug was that it doesn't)
        model.finalizeConnections();
    } catch (const OpenSim::Exception&) {
        exceptionThrown = true;
    }

    OPENSIM_THROW_IF(!exceptionThrown, Exception, "no exception was thrown (it should throw on finalization)");
}
