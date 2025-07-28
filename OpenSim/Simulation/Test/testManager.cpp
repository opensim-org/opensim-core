/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testManager.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Carmichael Ong                                                  *
 * Contributor(s): Nicholas Bianco                                            *
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

#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Simulation/StatesTrajectoryReporter.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Common/Constant.h>

#include <catch2/catch_all.hpp>
using Catch::Matchers::WithinAbs;

using namespace OpenSim;
namespace {
    Model createBallModel(bool prescribedActuator = false) {
        Model model;
        model.setName("ball");
        model.setGravity(SimTK::Vec3(0, -9.81, 0));

        auto ball = new Body("ball", 0.7, SimTK::Vec3(0.1),
            SimTK::Inertia::sphere(0.5));
        model.addBody(ball);

        auto freeJoint = new FreeJoint("freeJoint", 
            model.getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
            *ball, SimTK::Vec3(0), SimTK::Vec3(0));
        model.addJoint(freeJoint);
        Coordinate& sliderCoord = 
            freeJoint->updCoordinate(FreeJoint::Coord::TranslationY);

        CoordinateActuator* actu = new CoordinateActuator();
        actu->setCoordinate(&sliderCoord);
        actu->setName("actuator");
        actu->setOptimalForce(1);
        model.addForce(actu);
    
        if (prescribedActuator) {
            PrescribedController* controller = new PrescribedController();
            controller->setName("controller");
            controller->addActuator(*actu);
            controller->prescribeControlForActuator(actu->getName(),
                Constant(1.7));
            model.addController(controller);
        }

        BodyKinematics* bodyKinematics = new BodyKinematics();
        model.addAnalysis(bodyKinematics);

        return model;
    }

    class DiscreteController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(DiscreteController, Controller);
    public:
        DiscreteController() = default;

        void setDiscreteControls(SimTK::State& s, 
                const SimTK::Vector& controls) const {
            const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
            auto& dv = subSys.updDiscreteVariable(s, m_discreteVarIndex);
            auto& discreteControls = 
                    SimTK::Value<SimTK::Vector>::updDowncast(dv).upd();
            discreteControls = controls;
        }
        
        void computeControls(const SimTK::State& s, 
                    SimTK::Vector& controls) const override {
            const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
            const auto& dv = subSys.getDiscreteVariable(s, m_discreteVarIndex) ;
            const auto& discreteControls =
                    SimTK::Value<SimTK::Vector>::downcast(dv).get();
            controls += discreteControls;
        }
    protected:
        void extendRealizeTopology(SimTK::State& state) const override {
            Super::extendRealizeTopology(state);
            const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
            m_discreteVarIndex = subSys.allocateDiscreteVariable(
                    state, SimTK::Stage::Dynamics,
                    new SimTK::Value<SimTK::Vector>(
                            SimTK::Vector(getModel().getNumControls(), 0.0)));
        }
        mutable SimTK::DiscreteVariableIndex m_discreteVarIndex;
    }; 

    #define COMPARE_STATES(state1, state2, tol) \
        CHECK(state1.getNQ() == state2.getNQ()); \
        CHECK(state1.getNU() == state2.getNU()); \
        CHECK_THAT(state2.getTime(), WithinAbs(state1.getTime(), tol)); \
        for (int i = 0; i < state1.getNQ(); ++i) { \
            CHECK_THAT(state2.getQ()[i], WithinAbs(state1.getQ()[i], tol)); \
        } \
        for (int i = 0; i < state1.getNU(); ++i) { \
            CHECK_THAT(state2.getU()[i], WithinAbs(state1.getU()[i], tol)); \
        }
}

// Calculate the location, velocity, and acceleration of a Station with the same 
// Manager many times. Previously, this would fail as repeated calls of 
// TimeStepper::initialize() would trigger cache validation improperly.
TEST_CASE("Station calculations with Manager") {
    Model pendulum;
    pendulum.setName("pendulum");

    auto rod = new Body("rod", 0.54321, SimTK::Vec3(0.1, 0.5, 0.2),
        SimTK::Inertia::cylinderAlongY(0.025, 0.55));
    pendulum.addBody(rod);

    auto pin = new PinJoint("pin", 
        pendulum.getGround(), SimTK::Vec3(0), SimTK::Vec3(0), 
        *rod, SimTK::Vec3(0), SimTK::Vec3(0));
    pendulum.addJoint(pin);

    // Create station in the extra frame
    Station* myStation = new Station();
    const SimTK::Vec3 point(0.5, 1, -1.5);
    myStation->set_location(point);
    myStation->setParentFrame(*rod);
    pendulum.addModelComponent(myStation);

    // Initialize the system
    SimTK::State state = pendulum.initSystem();

    // set the model coordinates and coordinate speeds
    pin->getCoordinate(PinJoint::Coord::RotationZ).setValue(state, 0.29);
    pin->getCoordinate(PinJoint::Coord::RotationZ).setSpeedValue(state, 0.1);

    // Get the frame's mobilized body
    const OpenSim::PhysicalFrame&  frame = myStation->getParentFrame();
    SimTK::MobilizedBody mb = frame.getMobilizedBody();

    // Do a simulation
    double finalT = 1.0;
    double dt = 0.01;
    int n = int(round(finalT / dt));

    // Hold the computed kinematics from OpenSim and Simbody
    SimTK::Vec3 lo, vo, ao, l, v, a;
    
    Manager manager(pendulum);
    manager.setPerformAnalyses(false);
    manager.setWriteToStorage(false);
    state.setTime(0.0);
    manager.initialize(state);

    for (int i = 1; i <= n; ++i) {
        // Reuse the same Manager to integrate a state forward repeatedly.
        // This would previously cause issues with cache validation.
        state = manager.integrate(i*dt);

        // realize to acceleration to access acceleration stage cache
        pendulum.realizeAcceleration(state);

        // Use Simbody to get the location, velocity & acceleration in ground.
        mb.findStationLocationVelocityAndAccelerationInGround(state,
            point, l, v, a);
        lo = myStation->getLocationInGround(state);
        vo = myStation->getVelocityInGround(state);
        ao = myStation->getAccelerationInGround(state);

        INFO("t = " << state.getTime() << ": os_a = " << ao << " | sb_a = " 
            << a);

        // Compare Simbody values to values from Station
        SimTK_TEST_EQ(l, lo);
        SimTK_TEST_EQ(v, vo);
        SimTK_TEST_EQ(a, ao);
    }
}

// Change the initial value and speed of a falling ball between integrating 
// using the same State. This ensures that integrating with the same State with 
// different Managers triggers the cache updates correctly.
TEST_CASE("State changes between integration") {
    Model model = createBallModel();
    Station* myStation = new Station();
    const SimTK::Vec3 point(0);
    myStation->set_location(point);
    myStation->setParentFrame(model.getComponent<Body>("/bodyset/ball"));
    model.addModelComponent(myStation);

    SimTK::State& state = model.initSystem();

    const Coordinate& sliderCoord = 
            model.getComponent<FreeJoint>("/jointset/freeJoint")
            .getCoordinate(FreeJoint::Coord::TranslationY);

    std::vector<double> integInitTimes = {0.0, 1.0, 3.0};
    std::vector<double> integFinalTimes = {1.0, 3.0, 6.0};
    std::vector<double> initHeights = {0.0, 13.3, 6.5};
    std::vector<double> initSpeeds = {0.0, 0.5, -0.5};
    state.setTime(integInitTimes[0]);
    size_t n = integFinalTimes.size();

    for (size_t i = 0; i < n; ++i) {
        // Set initial state for integration and check that it's correct
        sliderCoord.setValue(state, initHeights[i]);
        sliderCoord.setSpeedValue(state, initSpeeds[i]);

        Manager manager(model);
        manager.initialize(state);
        const SimTK::State& initState = manager.getState();

        SimTK_TEST_EQ(initState.getTime(), integInitTimes[i]);
        SimTK_TEST_EQ(sliderCoord.getValue(initState), initHeights[i]);
        SimTK_TEST_EQ(sliderCoord.getSpeedValue(initState), initSpeeds[i]);

        // Use Station to get the location, velocity & acceleration in ground.
        double stationHeight = myStation->getLocationInGround(initState)[1];
        double stationSpeed = myStation->getVelocityInGround(initState)[1];

        SimTK_TEST_EQ(stationHeight, initHeights[i]);
        SimTK_TEST_EQ(stationSpeed, initSpeeds[i]);

        state = manager.integrate(integFinalTimes[i]);

        model.realizeVelocity(state);

        double duration = integFinalTimes[i] - integInitTimes[i];
        const auto& g = -model.getGravity()[1];
        double finalHeight = 
            initHeights[i] + initSpeeds[i]*duration - 0.5*g*duration*duration;
        double finalSpeed = initSpeeds[i] - g*duration;
        double sliderHeight = sliderCoord.getValue(state);
        double sliderSpeed = sliderCoord.getSpeedValue(state);
        INFO("Slider: t = " << state.getTime() << ", h = " << sliderHeight
            << ", v = " << sliderSpeed << " | Eq: t = " << integFinalTimes[i]
            << ", h = " << finalHeight << ", v = " << finalSpeed);

        SimTK_TEST_EQ(state.getTime(), integFinalTimes[i]);
        SimTK_TEST_EQ(sliderHeight, finalHeight);
        SimTK_TEST_EQ(sliderSpeed, finalSpeed);

        // Use Station to get the location, velocity & acceleration in ground.
        stationHeight = myStation->getLocationInGround(state)[1];
        stationSpeed = myStation->getVelocityInGround(state)[1];

        INFO("Station: t = " << state.getTime() << ", h = " 
            << stationHeight << ", v = " << stationSpeed);

        // Compare Station values with equation values
        SimTK_TEST_EQ(stationHeight, finalHeight);
        SimTK_TEST_EQ(stationSpeed, finalSpeed);
    }

}

// Update the excitation of a muscle in the arm26 model between subsequent 
// integrations.
TEST_CASE("Excitation updates with Manager") {
    Model arm("arm26.osim");

    const Set<Muscle> &muscleSet = arm.getMuscles();
    PrescribedController* controller = new PrescribedController();
    controller->addActuator(muscleSet.get(0));
    controller->prescribeControlForActuator(
        muscleSet.get(0).getAbsolutePathString(), Constant(0));
    arm.addController(controller);

    SimTK::State& state = arm.initSystem();
    state.setTime(0);
    double stepsize = 0.01;
    
    for (int i = 0; i < 3; ++i) {
        double initAct = 0.2 + i*0.2; // 0.2, 0.4, 0.6
        double excitation = initAct + 0.1; // 0.3, 0.5, 0.7
        
        muscleSet.get(0).setActivation(state, initAct);
        FunctionSet& fnset = controller->upd_ControlFunctions();
        Constant* fn = dynamic_cast<Constant*>(&fnset[0]);
        fn->setValue(excitation);

        Manager manager(arm);
        manager.initialize(state);

        // Make sure we set activation correctly
        arm.realizeDynamics(state);
        SimTK_TEST_EQ(initAct, muscleSet.get(0).getActivation(state));

        state = manager.integrate(stepsize*(i + 1));
        arm.realizeDynamics(state);
        double finalAct = muscleSet.get(0).getActivation(state);
        double finalExcitation = muscleSet.get(0).getExcitation(state);
        INFO("t = " << state.getTime() << ", initial_act = " << initAct 
            << ", initial_exc = " << excitation << ", final_act = " << finalAct);

        // Check if excitation is correct
        SimTK_TEST_EQ(excitation, finalExcitation);
        // Also check if the final activation is between the initial
        // activation and excitation
        SimTK_TEST(finalAct > initAct);
        SimTK_TEST(finalAct < excitation);
    }
}

// Ensure different constructors work as intended.
TEST_CASE("Constructors") {
    Model model = createBallModel();
    SimTK::State& state = model.initSystem();
    const Coordinate& sliderCoord = 
            model.getComponent<FreeJoint>("/jointset/freeJoint")
            .getCoordinate(FreeJoint::Coord::TranslationY);

    double initHeight = -0.5;
    double initSpeed = 0.3;
    SimTK::State initState = model.initSystem();
    sliderCoord.setValue(initState, initHeight);
    sliderCoord.setSpeedValue(initState, initSpeed);
    initState.setTime(0.0);

    double duration = 0.7;
    const auto& g = -model.getGravity()[1];
    double finalHeight = 
        initHeight + initSpeed*duration - 0.5*g*duration*duration;
    double finalSpeed = initSpeed - g*duration;

    SECTION("Primary constructor") {
        Manager manager1(model);
        CHECK_THROWS_AS(manager1.integrate(duration), Exception);
        manager1.initialize(initState);
        SimTK::State outState1 = manager1.integrate(duration);
        SimTK_TEST_EQ(sliderCoord.getValue(outState1), finalHeight);
        SimTK_TEST_EQ(sliderCoord.getSpeedValue(outState1), finalSpeed);

        Manager manager2(model);
        CHECK_THROWS_AS(manager2.integrate(duration), Exception);
        manager2.initialize(initState);
        SimTK::State outState2 = manager2.integrate(duration);
        SimTK_TEST_EQ(sliderCoord.getValue(outState2), finalHeight);
        SimTK_TEST_EQ(sliderCoord.getSpeedValue(outState2), finalSpeed);
    }

    SECTION("Convenience constructor") {
        Manager manager1(model, initState);
        SimTK::State outState1 = manager1.integrate(duration);
        SimTK_TEST_EQ(sliderCoord.getValue(outState1), finalHeight);
        SimTK_TEST_EQ(sliderCoord.getSpeedValue(outState1), finalSpeed);

        Manager manager2(model, initState);
        SimTK::State outState2 = manager2.integrate(duration);
        SimTK_TEST_EQ(sliderCoord.getValue(outState2), finalHeight);
        SimTK_TEST_EQ(sliderCoord.getSpeedValue(outState2), finalSpeed);
    }

    SECTION("Changing the integrator with the convenience constructor") {
        Manager manager(model, initState);
        manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKutta2);
        CHECK_THROWS_AS(manager.integrate(duration), Exception);
        manager.initialize(initState);
        SimTK::State outState = manager.integrate(duration);
        SimTK_TEST_EQ(sliderCoord.getValue(outState), finalHeight);
        SimTK_TEST_EQ(sliderCoord.getSpeedValue(outState), finalSpeed);
    }
}

// Ensure setting integrator options works as intended.
TEST_CASE("Integrator interface") {
    Model model = createBallModel();
    model.initSystem();
    Manager manager(model);

    // getMethodName returns char* so convert to string first for tests
    std::string method = manager.getIntegrator().getMethodName();
    // Default is RungeKuttaMerson
    SimTK_TEST(method == "RungeKuttaMerson");
    
    // Test setIntegratorMethod()
    manager.setIntegratorMethod(Manager::IntegratorMethod::ExplicitEuler);
    method = manager.getIntegrator().getMethodName();
    SimTK_TEST(method == "ExplicitEuler");

    manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKutta2);
    method = manager.getIntegrator().getMethodName();
    SimTK_TEST(method == "RungeKutta2");

    manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKutta3);
    method = manager.getIntegrator().getMethodName();
    SimTK_TEST(method == "RungeKutta3");

    manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKuttaFeldberg);
    method = manager.getIntegrator().getMethodName();
    SimTK_TEST(method == "RungeKuttaFeldberg");

    manager.setIntegratorMethod(Manager::IntegratorMethod::RungeKuttaMerson);
    method = manager.getIntegrator().getMethodName();
    SimTK_TEST(method == "RungeKuttaMerson");

    manager.setIntegratorMethod(Manager::IntegratorMethod::SemiExplicitEuler2);
    method = manager.getIntegrator().getMethodName();
    SimTK_TEST(method == "SemiExplicitEuler2");

    manager.setIntegratorMethod(Manager::IntegratorMethod::Verlet);
    method = manager.getIntegrator().getMethodName();
    SimTK_TEST(method == "Verlet");

    manager.setIntegratorMethod(Manager::IntegratorMethod::CPodes);
    method = manager.getIntegrator().getMethodName();
    SimTK_TEST(method == "CPodesBDF");

    // manager.setIntegratorMethod(Manager::IntegratorMethod::SemiExplicitEuler);
    // method = manager.getIntegrator().getMethodName();
    // SimTK_TEST(method == "SemiExplicitEuler");

    // Make some changes to the settings. We can't check to see if these 
    // actually changed because IntegratorRep is not exposed.
    manager.setIntegratorMethod(Manager::IntegratorMethod::ExplicitEuler);
    CHECK_NOTHROW(manager.setIntegratorAccuracy(0.314));
    CHECK_NOTHROW(manager.setIntegratorMinimumStepSize(0.11));
    CHECK_NOTHROW(manager.setIntegratorMaximumStepSize(0.22));
    CHECK_NOTHROW(manager.setIntegratorInternalStepLimit(999));
}

// Test that misuse actually triggers exceptions.
TEST_CASE("Exceptions") {
    Model arm1("arm26.osim");

    // model must have initSystem() called first
    CHECK_THROWS_AS(Manager(arm1), Exception);
    SimTK::State s1 = arm1.initSystem();
    Manager manager(arm1);

    // can't integrate if not initialized
    CHECK_THROWS_AS(manager.integrate(0.1), Exception);
    manager.initialize(s1);
    CHECK_NOTHROW(manager.integrate(0.1));

    // can't change model or integrator method now
    manager.setIntegratorMethod(Manager::IntegratorMethod::ExplicitEuler);
    CHECK_THROWS_AS(manager.integrate(0.1), Exception);
    manager.initialize(s1);

    // but integrator options can change
    manager.setIntegratorAccuracy(1e-4);
    manager.setIntegratorMinimumStepSize(0.01);
    CHECK_NOTHROW(manager.integrate(0.1));
}

TEST_CASE("Reporting") {
    SECTION("Reporting disabled") {
        Model model = createBallModel(true);
        SimTK::State state = model.initSystem();
        Manager manager(model);
        manager.setWriteToStorage(false);
        manager.setPerformAnalyses(false);
        manager.setRecordStatesTrajectory(false);
        manager.initialize(state);
        manager.integrate(1.0);
        // storage
        CHECK(manager.getStatesTable().getNumRows() == 0);
        CHECK(manager.getStatesTable().getNumColumns() == 0);
        auto controlsTable = model.getControlsTable();
        CHECK(controlsTable.getNumRows() == 0);
        CHECK(controlsTable.getNumColumns() == 0);
        // analyses
        BodyKinematics& bodyKin = dynamic_cast<BodyKinematics&>(
            model.getAnalysisSet().get(0));
        auto* positionStorage = bodyKin.getPositionStorage();
        CHECK(positionStorage->getSize() == 0);
        CHECK(positionStorage->getColumnLabels().getSize() == 10);
        // states trajectory
        CHECK(manager.getStatesTrajectory().getSize() == 0);
    }
    SECTION("Defaults") {
        Model model = createBallModel(true);
        SimTK::State state = model.initSystem();
        Manager manager(model);
        manager.initialize(state);
        manager.integrate(1.0);
        // storage
        CHECK(manager.getStatesTable().getNumRows() == 6);
        CHECK(manager.getStatesTable().getNumColumns() == 12);
        auto controlsTable = model.getControlsTable();
        CHECK(controlsTable.getNumRows() == 6);
        CHECK(controlsTable.getNumColumns() == 1);
        // analyses
        BodyKinematics& bodyKin = dynamic_cast<BodyKinematics&>(
            model.getAnalysisSet().get(0));
        auto* positionStorage = bodyKin.getPositionStorage();
        CHECK(positionStorage->getSize() == 6);
        CHECK(positionStorage->getColumnLabels().getSize() == 10);
        // states trajectory
        CHECK(manager.getStatesTrajectory().getSize() == 0);
    }
    SECTION("Write to storage") {
        Model model = createBallModel(true);
        SimTK::State state = model.initSystem();
        Manager manager(model);
        manager.setWriteToStorage(true);
        manager.setPerformAnalyses(false);
        manager.setRecordStatesTrajectory(false);
        manager.initialize(state);
        manager.integrate(1.0);
        // storage
        CHECK(manager.getStatesTable().getNumRows() == 6);
        CHECK(manager.getStatesTable().getNumColumns() == 12);
        auto controlsTable = model.getControlsTable();
        CHECK(controlsTable.getNumRows() == 6);
        CHECK(controlsTable.getNumColumns() == 1);
        // analyses
        BodyKinematics& bodyKin = dynamic_cast<BodyKinematics&>(
            model.getAnalysisSet().get(0));
        auto* positionStorage = bodyKin.getPositionStorage();
        CHECK(positionStorage->getSize() == 0);
        CHECK(positionStorage->getColumnLabels().getSize() == 10);
        // states trajectory
        CHECK(manager.getStatesTrajectory().getSize() == 0);
    }
    SECTION("Perform analyses") {
        Model model = createBallModel(true);
        SimTK::State state = model.initSystem();
        Manager manager(model);
        manager.setWriteToStorage(false);
        manager.setPerformAnalyses(true);
        manager.setRecordStatesTrajectory(false);
        manager.initialize(state);
        manager.integrate(1.0);
        // storage
        CHECK(manager.getStatesTable().getNumRows() == 0);
        CHECK(manager.getStatesTable().getNumColumns() == 0);
        auto controlsTable = model.getControlsTable();
        CHECK(controlsTable.getNumRows() == 0);
        CHECK(controlsTable.getNumColumns() == 0);
        // analyses
        BodyKinematics& bodyKin = dynamic_cast<BodyKinematics&>(
            model.getAnalysisSet().get(0));
        auto* positionStorage = bodyKin.getPositionStorage();
        CHECK(positionStorage->getSize() == 6);
        CHECK(positionStorage->getColumnLabels().getSize() == 10);
        // states trajectory
        CHECK(manager.getStatesTrajectory().getSize() == 0);
    }
    SECTION("Record states trajectory") {
        Model model = createBallModel(true);
        SimTK::State state = model.initSystem();
        Manager manager(model);
        manager.setWriteToStorage(false);
        manager.setPerformAnalyses(false);
        manager.setRecordStatesTrajectory(true);
        manager.initialize(state);
        manager.integrate(1.0);
        // storage
        CHECK(manager.getStatesTable().getNumRows() == 0);
        CHECK(manager.getStatesTable().getNumColumns() == 0);
        auto controlsTable = model.getControlsTable();
        CHECK(controlsTable.getNumRows() == 0);
        CHECK(controlsTable.getNumColumns() == 0);
        // analyses
        BodyKinematics& bodyKin = dynamic_cast<BodyKinematics&>(
            model.getAnalysisSet().get(0));
        auto* positionStorage = bodyKin.getPositionStorage();
        CHECK(positionStorage->getSize() == 0);
        CHECK(positionStorage->getColumnLabels().getSize() == 10);
        // states trajectory
        CHECK(manager.getStatesTrajectory().getSize() == 6);
    } 
    SECTION("All enabled") {
        Model model = createBallModel(true);
        SimTK::State state = model.initSystem();
        Manager manager(model);
        manager.setWriteToStorage(true);
        manager.setPerformAnalyses(true);
        manager.setRecordStatesTrajectory(true);
        manager.initialize(state);
        manager.integrate(1.0);
        // storage
        CHECK(manager.getStatesTable().getNumRows() == 6);
        CHECK(manager.getStatesTable().getNumColumns() == 12);
        auto controlsTable = model.getControlsTable();
        CHECK(controlsTable.getNumRows() == 6);
        CHECK(controlsTable.getNumColumns() == 1);
        // analyses
        BodyKinematics& bodyKin = dynamic_cast<BodyKinematics&>(
            model.getAnalysisSet().get(0));
        auto* positionStorage = bodyKin.getPositionStorage();
        CHECK(positionStorage->getSize() == 6);
        CHECK(positionStorage->getColumnLabels().getSize() == 10);
        // states trajectory
        CHECK(manager.getStatesTrajectory().getSize() == 6);
    }    
}

TEST_CASE("Reinitializing Manager") {
    Model model = createBallModel(true);
    SimTK::State state = model.initSystem();
    Manager manager(model);
    manager.setRecordStatesTrajectory(true);
    manager.initialize(state);
    manager.integrate(1.0);
    CHECK(manager.getState().getTime() == 1.0);
    CHECK(manager.getStatesTable().getNumRows() == 6);
    CHECK(manager.getStatesTable().getNumColumns() == 12);
    CHECK(manager.getStatesTrajectory().getSize() == 6);
    
    // re-initializing does not fail and clears reporting data structures
    state.updTime() = 0.0;
    state.updQ() = SimTK::Test::randVector(state.getNQ());
    state.updU() = SimTK::Test::randVector(state.getNU());
    CHECK_NOTHROW(manager.initialize(state));
    CHECK(manager.getState().getTime() == 0.0);
    CHECK(manager.getStatesTable().getNumRows() == 0);
    CHECK(manager.getStatesTable().getNumColumns() == 0);
    CHECK(manager.getStatesTrajectory().getSize() == 0);

    // re-running simulation succeeds
    CHECK_NOTHROW(manager.integrate(1.0));
    CHECK(manager.getState().getTime() == 1.0);
    CHECK(manager.getStatesTable().getNumRows() == 6);
    CHECK(manager.getStatesTable().getNumColumns() == 12);
    CHECK(manager.getStatesTrajectory().getSize() == 6);

    // re-run with a different initial and final time
    state.updTime() = 0.5;
    state.updQ() = SimTK::Test::randVector(state.getNQ());
    state.updU() = SimTK::Test::randVector(state.getNU());
    CHECK_NOTHROW(manager.initialize(state));
    CHECK_NOTHROW(manager.integrate(5.0));
    CHECK(manager.getState().getTime() == 5.0);
    CHECK(manager.getStatesTable().getNumRows() == 44);
    CHECK(manager.getStatesTable().getNumColumns() == 12);
    CHECK(manager.getStatesTrajectory().getSize() == 44);
}

TEST_CASE("Updating the integrator") {
    Model model = createBallModel(true);
    SimTK::State state = model.initSystem();
    Manager manager(model);
    manager.initialize(state);
    manager.integrate(1.0);
    
    manager.setIntegratorMethod(Manager::IntegratorMethod::SemiExplicitEuler2);
    CHECK_THROWS(manager.integrate(2.0));
    manager.initialize(state);
    CHECK_NOTHROW(manager.integrate(2.0));
}

TEST_CASE("Updating the integrator settings after initializing") {
    Model model = createBallModel(true);
    SimTK::State state = model.initSystem();
    Manager manager(model);
    manager.initialize(state);
    manager.setIntegratorFixedStepSize(0.001);
    manager.integrate(1.0);
    CHECK(manager.getState().getTime() == 1.0);
    
    TimeSeriesTable statesTable = manager.getStatesTable();
    CHECK(statesTable.getNumRows() == 1001);
    CHECK(manager.getStatesTable().getNumColumns() == 12);
}

TEST_CASE("Updating states") {
    Model model = createBallModel(false);
    SimTK::State defaultState = model.initSystem();
    Manager manager(model);
    manager.setRecordStatesTrajectory(true); 

    manager.initialize(defaultState);
    SimTK::State newState = manager.integrate(1.0);
    StatesTrajectory states = manager.getStatesTrajectory();
    const double tol = 10 * defaultState.getNU() * SimTK::Test::defTol<double>(); 
    COMPARE_STATES(states.front(), defaultState, tol);

    SimTK::Random::Uniform rand(0, 1);
    newState.updTime() = rand.getValue();
    newState.updQ() = SimTK::Test::randVector(newState.getNQ());
    newState.updU() = SimTK::Test::randVector(newState.getNU());

    // we didn't call initialize(), so the initial states should not have changed
    manager.integrate(2.0);
    states = manager.getStatesTrajectory();
    COMPARE_STATES(states.front(), defaultState, tol);
    
    // now initialize the manager and integrate again
    manager.initialize(newState);
    manager.integrate(2.0);
    states = manager.getStatesTrajectory();
    COMPARE_STATES(states.front(), newState, tol);
}

TEST_CASE("Updating controls") {
    Model model = createBallModel(false);
    // this custom controller allows us to update the controls with the state
    DiscreteController* controller = new DiscreteController();
    controller->setName("discrete_controller");
    model.addController(controller);
    SimTK::State state = model.initSystem();
    Manager manager(model); 

    // default controls are zero
    manager.initialize(state);
    state = manager.integrate(1.0);
    auto controlsTable = model.getControlsTable();
    CHECK(controlsTable.getDependentColumnAtIndex(0)[0] == 0.0);

    // update the controls
    SimTK::Vector controls = SimTK::Test::randVector(model.getNumControls());
    controller->setDiscreteControls(state, controls);

    // we didn't call initialize(), so the controls should not have changed
    manager.integrate(2.0);
    controlsTable = model.getControlsTable();
    int nrow = static_cast<int>(controlsTable.getNumRows());
    CHECK(controlsTable.getDependentColumnAtIndex(0)[nrow - 1] == 0.0);
    
    // now initialize the manager and integrate again
    manager.initialize(state);
    manager.integrate(2.0);
    controlsTable = model.getControlsTable();
    nrow = static_cast<int>(controlsTable.getNumRows());
    CHECK(controlsTable.getDependentColumnAtIndex(0)[nrow - 1] == controls[0]);
}

TEST_CASE("Manager is compatible with StatesTrajectoryReporter") {
    Model model = createBallModel(false);
    StatesTrajectoryReporter* reporter =
        new StatesTrajectoryReporter();
    reporter->setName("states_reporter");
    double interval = 0.1;
    reporter->set_report_time_interval(interval);
    model.addComponent(reporter);
    SimTK::State state = model.initSystem();
    const double tol = 10 * state.getNU() * SimTK::Test::defTol<double>(); 

    Manager manager(model);
    manager.setRecordStatesTrajectory(true);
    manager.initialize(state);
    manager.integrate(2.0);

    StatesTrajectory statesFromReporter = reporter->getStates();
    StatesTrajectory statesFromManager = manager.getStatesTrajectory();

    // Manager will record internal steps between the reporter's intervals,
    // so the reporter's states should be a subset of the manager's states.
    CHECK(statesFromReporter.getSize() <= statesFromManager.getSize());
    for (const auto& stateFromReporter : statesFromReporter) {
        for (const auto& stateFromManager : statesFromManager) {
            if (stateFromReporter.getTime() == stateFromManager.getTime()) {
                COMPARE_STATES(stateFromReporter, stateFromManager, tol);
                break;
            }
        }
    }
}
