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

/** Simple proportional controller whose output is a gain times a
 * coordinate value.
 * There is an option to delay the value of the coordinate.
 * We've tacked on a bunch of other miscellaneous stuff solely for testing
 * the Delay component.
 */
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

    ScalarDelay _delay;
    VectorDelay _vectorDelay;
    Delay<SimTK::Vec3> _comVelocityDelay;
};

/// Contains dummy outputs with different requiresAt stages so that we can
/// test that stages are checked appropriately when asking for the Delay output.
class DelayStageTesting : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(DelayStageTesting, ModelComponent);
public:
    DelayStageTesting() {
        constructInfrastructure();
    }
    // This also tests the convenience constructor.
    ScalarDelay posDelay{0.01};
    ScalarDelay velDelay{0.02};
    ScalarDelay dynDelay{0.03};

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

/// Allows some testing of Delay with different models.
void testDelaySimulation(Model& model, double delayTime = 0.017,
        bool testRegression = true, double maxStepSize = SimTK::NaN,
        /* integrator accuracy, test tolerance */ double tol=1e-6) {

    double finalTime = 1.35;

    const Coordinate& coord = model.getCoordinateSet()[0];
    auto* controller = dynamic_cast<PendulumController*>(
            &model.updControllerSet()[0]);

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
        model.realizeTime(s); // TODO should this be Velocity?

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

void testDelay() {
    // This type is not registered in osimCommon.
    Object::registerType(Delay<SimTK::Vec3>());
    Object::registerType(PendulumController());

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
        // TODO doesn't work testDelaySimulation(model, 0.0, false);

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
            Object::registerType(PendulumController());
            std::string filename = "pendulum_for_delay_test.osim";
            model.print(filename);
            Model modelDeserialized(filename);
            testDelaySimulation(modelDeserialized);
        }
        */
    }
    
    // Check for possible issues with requiredAt and dependsOn stages.
    // ===============================================================
    {
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

        /*
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
        */
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

    // Check for exception when delay is negative.
    // ===========================================
    {
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
}

int main() {
    SimTK_START_TEST("testOperators");
        SimTK_SUBTEST(testDelay);
    SimTK_END_TEST();
}
