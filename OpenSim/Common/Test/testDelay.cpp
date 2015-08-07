/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testDelay.cpp                                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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

#include <OpenSim/OpenSim.h>

using namespace SimTK;
using namespace OpenSim;

Model createPendulumModel() {
    Model model;
    // The link rests along the x axis.
    auto* body = new OpenSim::Body("body", 1, Vec3(1, 0, 0), Inertia(0));
    PinJoint* joint = new PinJoint("joint",
            model.getGround(), Vec3(0), Vec3(0),
            *body, Vec3(0), Vec3(0));

    model.addBody(body);
    model.addJoint(joint);

    return model;
}

// Simple proportional controller whose output is a gain times a
// coordinate value.
// There is an option to delay the value of the coordinate.
// We've tacked on a bunch of other miscellaneous stuff solely for testing
// the Delay component.
class PendulumController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(PendulumController, Controller);
public:
    OpenSim_DECLARE_PROPERTY(delay, double,
            "Use a delayed coordinate value. Default: 0, for no delay.");
    PendulumController() {
        constructInfrastructure();
        _delay.setName("coordinate_delay");
        _vectorDelay.setName("vector_coordinate_delay");
        _comVelocityDelay.setName("com_velocity_delay");
    }
    PendulumController(double delay) : PendulumController() {
        set_delay(delay);
    }
    void computeControls(const State& s, Vector &controls) const override {
        double q;
        if (get_delay() > 0) {
            q = _delay.getValue(s);
        } else {
            q = getModel().getCoordinateSet()[0].getValue(s);
        }
        double u = -10 * q;
        getModel().getActuators()[0].addInControls(Vector(1, u), controls);
    }
    Vector getCoordValueVector(const SimTK::State& s) const {
        return Vector(5, getInputValue<double>(s, "coord_value"));
    }
    // Just for testing purposes to see if the Delay itself works.
    double getDelayedCoordinateValue(const State& s) const {
        return _delay.getOutputValue<double>(s, "output");
    }
    Vector getDelayedCoordValueVector(const SimTK::State& s) const {
        return _vectorDelay.getValue(s);
    }
    SimTK::Vec3 getDelayedCOMVelocityValue(const State& s) const {
        return _comVelocityDelay.getValue(s);
    }

private:
    void constructProperties() override {
        constructProperty_delay(0.0);
    }
    void constructInputs() override {
        constructInput<double>("coord_value", SimTK::Stage::Position);
    }
    void constructOutputs() override {
        constructOutput<Vector>("coord_value_vector",
                &PendulumController::getCoordValueVector,
                SimTK::Stage::Position);
    }
    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();
        _delay.set_delay(get_delay());
        _vectorDelay.set_delay(get_delay());
        _comVelocityDelay.set_delay(get_delay());
        addComponent(&_delay);
        addComponent(&_vectorDelay);
        addComponent(&_comVelocityDelay);
    }
    void extendConnectToModel(Model& model) override {
        Super::extendConnectToModel(model);
        const auto& coord = model.getCoordinateSet()[0];
        getInput("coord_value").connect(coord.getOutput("value"));
        _delay.getInput("input").connect(coord.getOutput("value"));
        _vectorDelay.getInput("input").connect(getOutput("coord_value_vector"));
        _comVelocityDelay.getInput("input").connect(
                model.getOutput("com_velocity"));
    }

    Delay _delay;
    DelayVector _vectorDelay;
    Delay_<SimTK::Vec3> _comVelocityDelay;
};

// Contains dummy outputs with different requiresAt stages so that we can
// test that stages are checked appropriately when asking for the Delay output.
class DelayStageTesting : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(DelayStageTesting, ModelComponent);
public:
    DelayStageTesting() {
        constructInfrastructure();
    }
    // This also tests the convenience constructor.
    Delay posDelay{0.01};
    Delay velDelay{0.02};
    Delay dynDelay{0.03};

private:
    void constructOutputs() override {
        constructOutput<double>("position",
                std::bind([](const SimTK::State& s)->double
                        { return s.getTime() + 1.0; },
                        std::placeholders::_1),
                SimTK::Stage::Position);
        constructOutput<double>("velocity",
                std::bind([](const SimTK::State& s)->double
                        { return s.getTime() + 2.0; },
                        std::placeholders::_1),
                SimTK::Stage::Velocity);
        /* TODO
        There's a bug where outputs with a dependsOn stage of Dynamics or
        above causes an exception to be thrown when initializing the
        TimeStepper.
        constructOutput<double>("dynamics",
                std::bind([](const SimTK::State& s)->double
                        { return s.getTime() + 3.0; },
                        std::placeholders::_1),
                SimTK::Stage::Dynamics);
        */
    }
    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();
        addComponent(&posDelay);
        addComponent(&velDelay);
        //TODO see above addComponent(&dynDelay);
    }
    void extendConnectToModel(Model& model) override {
        Super::extendConnectToModel(model);
        posDelay.getInput("input").connect(getOutput("position"));
        velDelay.getInput("input").connect(getOutput("velocity"));
        //TODO see above dynDelay.getInput("input").connect(getOutput("dynamics"));
    }
};

// Allows some testing of Delay with different models.
void testDelaySimulation(Model& model, double delayTime = 0.017,
        bool testRegression = true, double maxStepSize = SimTK::NaN,
        /* integrator accuracy, test tolerance */ double tol=1e-6) {

    double finalTime = 1.35;

    const Coordinate& coord = model.getCoordinateSet()[0];
    auto* controller = dynamic_cast<PendulumController*>(
            &model.updControllerSet().get("mycontroller"));

    // First simulate without controller delay.
    double finalCoordValue_noControllerDelay;
    SimTK::Vector finalCoordValueVector_noControllerDelay;
    SimTK::Vec3 finalCOMValue_noControllerDelay;
    {
        State& s = model.initSystem();
        SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
        integrator.setAccuracy(tol);
        if (!SimTK::isNaN(maxStepSize)) {
            integrator.setMaximumStepSize(maxStepSize);
        }
        SimTK::TimeStepper ts(model.getSystem(), integrator);
        ts.initialize(s);
        ts.stepTo(finalTime);
        s = ts.getState();

        finalCoordValue_noControllerDelay = coord.getValue(s);
        finalCoordValueVector_noControllerDelay =
                controller->getCoordValueVector(s);
        finalCOMValue_noControllerDelay = model.calcMassCenterVelocity(s);
    }

    // Simulate with controller delay and record expected delayed value.
    double expectedDelayedCoordValue;
    double actualDelayedCoordValue;
    double finalCoordValue_withControllerDelay;
    SimTK::Vector expectedDelayedCoordValueVector;
    SimTK::Vector actualDelayedCoordValueVector;
    SimTK::Vector finalCoordValueVector_withControllerDelay;
    SimTK::Vec3 expectedDelayedCOMValue;
    SimTK::Vec3 actualDelayedCOMValue;
    SimTK::Vec3 finalCOMValue_withControllerDelay;
    {
        controller->set_delay(delayTime);
        State& s = model.initSystem();
        SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
        integrator.setAccuracy(tol);
        SimTK::TimeStepper ts(model.getSystem(), integrator);
        ts.initialize(s);

        // Stop early so we can record the true value of the delayed
        // coordinate.
        ts.stepTo(finalTime - delayTime);
        s = ts.getState();

        expectedDelayedCoordValue = coord.getValue(s);
        expectedDelayedCoordValueVector = controller->getCoordValueVector(s);
        expectedDelayedCOMValue = model.calcMassCenterVelocity(s);

        // Simulate with controller delay to the finalTime.
        ts.stepTo(finalTime);
        s = ts.getState();

        // Must realize in order to access delayed value.
        model.realizeVelocity(s);

        actualDelayedCoordValue = controller->getDelayedCoordinateValue(s);
        actualDelayedCoordValueVector =
                controller->getDelayedCoordValueVector(s);
        actualDelayedCOMValue = controller->getDelayedCOMVelocityValue(s);

        finalCoordValue_withControllerDelay = coord.getValue(s);
        finalCoordValueVector_withControllerDelay =
                controller->getCoordValueVector(s);
        finalCOMValue_withControllerDelay = model.calcMassCenterVelocity(s);
    }

    // Basic check on the Delay within the controller to see that the Delay
    // itself is working as a subcomponent.
    SimTK_TEST_EQ_TOL(expectedDelayedCoordValue, actualDelayedCoordValue, tol);
    SimTK_TEST_EQ_TOL(expectedDelayedCoordValueVector,
            actualDelayedCoordValueVector, tol);
    SimTK_TEST_EQ_TOL(expectedDelayedCOMValue, actualDelayedCOMValue, tol);

    // Make sure the final coordinate value is not the same with vs.
    // without controller delay, to show that the controller delay had an
    // effect.
    SimTK_TEST_NOTEQ_TOL(finalCoordValue_noControllerDelay,
            finalCoordValue_withControllerDelay, tol);
    SimTK_TEST_NOTEQ_TOL(finalCoordValueVector_noControllerDelay,
            finalCoordValueVector_withControllerDelay, tol);
    SimTK_TEST_NOTEQ_TOL(finalCOMValue_noControllerDelay,
            finalCOMValue_withControllerDelay, tol);

    // Regression. Since above we are only checking if the controller with
    // delay and the controller without delay produce different results,
    // the above test is not sufficient to check if behavior changes.
    if (testRegression) {
        // The testRegression argument is a hacky way to only test regression
        // with the default delayTime, since these hardcoded numbers were
        // obtained with the default delayTime (0.017 s).
        SimTK_TEST_EQ_TOL(finalCoordValue_noControllerDelay, -0.245182, tol);
        SimTK_TEST_EQ_TOL(finalCoordValue_withControllerDelay, -0.171928, tol);
        SimTK_TEST_EQ_TOL(finalCOMValue_noControllerDelay,
                Vec3(0.495057698, 1.97852341, 0.0), tol);
        SimTK_TEST_EQ_TOL(finalCOMValue_withControllerDelay,
                Vec3(0.392224361, 2.2588064, 0.0), tol);
    }
}

void testWithController() {

    // Test the use of the Delay within a controller.
    // ==============================================
    {
        Model model = createPendulumModel();
        const Coordinate& coord = model.getCoordinateSet()[0];
        std::string coordName = coord.getName();

        // Add actuator.
        CoordinateActuator* act = new CoordinateActuator(coordName);
        act->setName("joint_0_actuator");
        model.addForce(act);

        // Add controller.
        PendulumController* controller = new PendulumController();
        controller->setName("mycontroller");
        controller->addActuator(*act);
        model.addController(controller);

        // Run test on original model.
        testDelaySimulation(model);

        // Run test on original model using a different delay time,
        // to ensure that the Delay component is able to modify the
        // Measure::Delay properly if a property changes.
        testDelaySimulation(model, 0.020, false);

        // Test a really small delay duration (the default accuracy).
        testDelaySimulation(model, 1e-6, false);

        // Show that accuracy equal to delay yields correct results.
        // TODO
        //testDelaySimulation(model, 0.0001, false, SimTK::NaN, 0.0001);

        // TODO
        testDelaySimulation(model, 0.0005, false, 0.02);

        /*
         The two exception tests below were used to understand the interaction
         between delay duration and integrator accuracy and min step size.
         They are not too useful as actual tests though.
        // Show that accuracy less tight than delay yields incorrect results,
        // even at the looser testing tolerance (= accuracy).
        SimTK_TEST_MUST_THROW_EXC(
            testDelaySimulation(model, 0.0001, false, SimTK::NaN, 0.0005),
            SimTK::Exception::Assert);

        // Test a delay duration that is shorter than a time step.
        // This shows that having a time step smaller than the delay
        // is not sufficient; integrator accuracy is what's important.
        SimTK_TEST_MUST_THROW_EXC(
            testDelaySimulation(model, 0.0001, false, 0.00005),
            SimTK::Exception::Assert);
        */

        // Test using a delay of 0.
        testDelaySimulation(model, 0.0, false);

        /* TODO will not pass until input/output copying is fixed.
        // Run same test after copying the model that contains the Delay.
        // --------------------------------------------------------------
        {
            Model modelCopy = model;
            testDelaySimulation(modelCopy);
        }

        // Run same test after serializing and deserializing the model.
        // ------------------------------------------------------------
        {
            std::string filename = "pendulum_for_delay_test.osim";
            model.print(filename);
            Model modelDeserialized(filename);
            testDelaySimulation(modelDeserialized);
        }
        */
    }

}

void testStages() {
    // Check for possible issues with requiredAt and dependsOn stages.
    // ===============================================================
    // Tests (1) exception thrown if stage is lower than the output's
    // dependsOn stage.
    // (2) that after realizing to the appropriate stage,
    // the Delay has the correct value.

    // Right now, these tests don't pass. Mainly, exceptions are not
    // thrown when they should be. This is because the
    // Delay's output dependsOnStage is Time,
    // when ideally it would be promoted to the dependsOnStage of the
    // output that is wired into the Delay's input if the latter stage
    // is greater.
    // TODO

    // Create model with the dummy DelayStageTesting component.
    double tol = 1e-6;
    Model model;
    auto* comp = new DelayStageTesting();
    comp->setName("delay_stage_testing");
    model.addModelComponent(comp);

    // Integrate past 3.0 seconds.
    State& s = model.initSystem();
    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    integrator.setAccuracy(tol);
    SimTK::TimeStepper ts(model.getSystem(), integrator);
    ts.initialize(s);
    ts.stepTo(5.0);
    s = ts.getState();

    std::cout << "DEBUG " << s.getSystemStage() << std::endl;
    // Try realizing to the various stages.
    // One stage below posDelay's dependsOn Stage.
    model.realizeTime(s);
    SimTK_TEST_MUST_THROW_EXC(comp->posDelay.getValue(s),
            SimTK::Exception::StageTooLow);

    model.realizePosition(s);
    // getOutput("position") - delay.
    SimTK_TEST_EQ(comp->posDelay.getValue(s), 5.0 + 1.0 - 0.01);
    SimTK_TEST_MUST_THROW_EXC(comp->velDelay.getValue(s),
            SimTK::Exception::StageTooLow);

    model.realizeVelocity(s);
    // getOutput("velocity") - delay.
    SimTK_TEST_EQ(comp->velDelay.getValue(s), 5.0 + 2.0 - 0.02);
    /* TODO
    There's a bug where outputs with a dependsOn stage of Dynamics or
    above causes an exception to be thrown when initializing the
    TimeStepper.
    SimTK_TEST_MUST_THROW_EXC(comp->dynDelay.getValue(s),
            SimTK::Exception::StageTooLow);

    model.realizeDynamics(s);
    // getOutput("dynamics") - delay.
    SimTK_TEST_EQ(comp->dynDelay.getValue(s), 5.0 + 3.0 - 0.03);
    */
}

void testNegativeDelayDurationException() {
    // Check for exception when delay is negative.
    // ===========================================
    Model model = createPendulumModel();

    // Add actuator.
    const Coordinate& coord = model.getCoordinateSet()[0];
    std::string coordName = coord.getName();
    CoordinateActuator* act = new CoordinateActuator(coordName);
    act->setName("joint_0_actuator");
    model.addForce(act);

    // Add controller.
    PendulumController* controller = new PendulumController();
    controller->addActuator(*act);
    controller->set_delay(-0.35); // this should cause the exception.
    model.addController(controller);

    SimTK_TEST_MUST_THROW_EXC(model.initSystem(),
                              SimTK::Exception::ValueWasNegative);
}

// Has an output that is a step function. Used to see if the delay can
// properly convey the discontinuity.
// This test was used to explore two types of discontinuities: a step input,
// and a linear function with a jump. In both cases, the discontinuity is at
// 0.5 seconds. For both discontinuities, to get an accurate delay output, we
// must use an event handler. For the step input, a
class DiscontinuousInputTesting : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(DiscontinuousInputTesting, ModelComponent);
public:
    DiscontinuousInputTesting() {
        constructInfrastructure();
    }
    Delay timeDelay{0.2};
    Delay stepDelay{0.2};
    Delay discontDelay{0.2};

private:
    void constructOutputs() override {
        constructOutput<double>("time",
                                std::bind([](const SimTK::State& s)->double
                                          { return s.getTime(); },
                                          std::placeholders::_1),
                                SimTK::Stage::Time);
        constructOutput<double>("step",
                                std::bind([](const SimTK::State& s)->double
                                          { return s.getTime() < 0.5 ? 1.5 : 4.3; },
                                          std::placeholders::_1),
                                SimTK::Stage::Time);
        constructOutput<double>("discont",
                                std::bind([](const SimTK::State& s)->double
                                          { return s.getTime() < 0.5 ? s.getTime() : 1 + s.getTime(); },
                                          std::placeholders::_1),
                                SimTK::Stage::Time);
    }
    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();
        addComponent(&timeDelay);
        addComponent(&stepDelay);
        addComponent(&discontDelay);
    }
    void extendConnectToModel(Model& model) override {
        Super::extendConnectToModel(model);
        timeDelay.getInput("input").connect(getOutput("time"));
        stepDelay.getInput("input").connect(getOutput("step"));
        discontDelay.getInput("input").connect(getOutput("discont"));
    }
    // This event gets triggered at 0.5 seconds, which matches the
    // time of the discontinuity in the "step" and "discont" outputs.
    class FixedEvent : public SimTK::ScheduledEventHandler {
    public:
        void handleEvent(State &state, Real accuracy, bool &shouldTerminate)
                const override { }

        Real getNextEventTime(const State &state, bool includeCurrentTime)
                const override {
            return 0.5;
        }
    };
    // This event is triggered whenever the input and output of the delay
    // begin to deviate. The clever idea comes from Dimitar Stanev.
    class DelayInputOutputEvent : public SimTK::TriggeredEventHandler {
    public:
        DelayInputOutputEvent(const Delay& delay, SimTK::Stage dependsOn) :
                TriggeredEventHandler(dependsOn), m_delay(delay) {
            getTriggerInfo().setTriggerOnRisingSignTransition(true);
            getTriggerInfo().setTriggerOnFallingSignTransition(false);
        }
        void handleEvent(State& state, Real accuracy, bool& shouldTerminate)
        const override {}
        SimTK::Real getValue(const SimTK::State& s) const override {
            if (m_delay.getOutputValue<double>(s, "output") ==
                m_delay.getInputValue<double>(s, "input")) {
                return -1;
            } else {
                return 1;
            }
        }
    private:
        const Delay& m_delay;
    };
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        // Leads to correct results for both "step" and "discont".
        system.addEventHandler(new FixedEvent());
        // Leads to correct results for step input.
        //system.addEventHandler(
        //      new DelayInputOutputEvent(stepDelay, SimTK::Stage::Position));
        // Leads to incorrect results for "discont" output. Specifically,
        // the first time step after t = 0.7 seconds will give an incorrect
        // value for the delayed value of the "discont" output.
        //system.addEventHandler(
        //      new DelayInputOutputEvent(discontDelay, SimTK::Stage::Position));
    }
};

template<typename T>
class Reporter_ : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT_T(Reporter_, T, ModelComponent);
public:
    Reporter_() {
        constructInfrastructure();
    }
    void extendRealizeAcceleration(const SimTK::State& s) const override {
        Super::extendRealizeAcceleration(s);
        //std::cout << std::setw(10) << std::left << s.getTime() << " " <<
        //        getInputValue<T>(s, "input") << std::endl;
    }
private:
    void constructInputs() override {
        constructInput<T>("input", SimTK::Stage::Time);
    }
};

typedef Reporter_<SimTK::Real> Reporter;

void testDiscontinuousInput() {
    double tol = 1e-6;

    // Create model.
    Model model;
    auto* body = new OpenSim::Body("body", 1, Vec3(0, 0, 0), Inertia(0));
    auto* joint = new SliderJoint("joint",
                                   model.getGround(), Vec3(0), Vec3(0),
                                   *body, Vec3(0), Vec3(0));
    auto* spring = new OpenSim::SpringGeneralizedForce("joint_coord_0");
    // Tune stiffness to get not-too-small-not-too-large time steps.
    // too small: delay is accurate even without the event handler.
    // too large: delay is inaccurate if time steps are same size as
    //            delay duration.
    spring->set_stiffness(10);
    spring->set_rest_length(1);

    model.addBody(body);
    model.addJoint(joint);
    model.addForce(spring);

    // Add Delay-related components.
    auto* comp = new DiscontinuousInputTesting();
    model.addModelComponent(comp);

    auto* reporter = new Reporter();
    model.addModelComponent(reporter);

    reporter->getInput("input").connect(comp->stepDelay.getOutput("output"));

    // Integrate past 3.0 seconds.
    State& s = model.initSystem();
    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    integrator.setAccuracy(tol);
    SimTK::TimeStepper ts(model.getSystem(), integrator);
    ts.initialize(s);

    /*
    // This block was used for developing the test, but is not part of the
    // test. You may want to use this if you edit the test.
    ts.setReportAllSignificantStates(true);
    integrator.setReturnEveryInternalStep(true);
    while (ts.getState().getTime() < 2.0) {
        ts.stepTo(2.0);
        std::cout << ts.getState().getTime() << " " <<
        comp->stepDelay.getValue(ts.getState()) << " " <<
        comp->discontDelay.getValue(ts.getState()) << std::endl;
    }
    s = model.initSystem();
    ts = SimTK::TimeStepper(model.getSystem(), integrator);
    ts.initialize(s);
    */

    // Before the discontinuity.
    // Ideally we'd step to just before the delayed time of the
    // discontinuity (0.7 seconds), but the delayed quantities are actually
    // incorrect then. We'd have to take much smaller time steps for the delay
    // to be accurate between t = 0.66 seconds and 0.7 seconds.
    ts.stepTo(0.66);
    // The "step" output switches at 0.5 seconds, but the delay is 0.2 seconds,
    // so the delay output should still be 1.5.
    //std::cout << comp->stepDelay.getValue(ts.getState()) << " " << std::endl;
    SimTK_TEST_EQ(comp->stepDelay.getValue(ts.getState()), 1.5);
    //std::cout << comp->discontDelay.getValue(ts.getState()) << " " << std::endl;
    SimTK_TEST_EQ(comp->discontDelay.getValue(ts.getState()), 0.46);

    // At the discontinuity. :)
    ts.stepTo(0.70);
    SimTK_TEST_EQ(comp->stepDelay.getValue(ts.getState()), 4.3);
    SimTK_TEST_EQ(comp->discontDelay.getValue(ts.getState()), 1.500);
}

int main() {

    SimTK_START_TEST("testDelay");
        // This Delay type is not registered in osimCommon.
        Object::registerType(Delay_<SimTK::Vec3>());

        // This class is defined in this file.
        Object::registerType(PendulumController());

        SimTK_SUBTEST(testWithController);
        SimTK_SUBTEST(testNegativeDelayDurationException);
        // TODO SimTK_SUBTEST(testStages); see test for why it's omitted.
        SimTK_SUBTEST(testDiscontinuousInput);
    SimTK_END_TEST();
}




















