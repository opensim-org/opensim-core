/* -------------------------------------------------------------------------- *
 *                OpenSim:  testOperators.cpp                                 *
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

// TODO SimTK_TEST exceptions give a message telling users to file a bug report
// to Simbody.
// TODO InputMeasure is not updated if depends-on-stage is Model.
// TODO is the Delay stage Time at minimum? how can you delay a model quantity?
// TODO should Delay hold onto the Measure::Delay handle, or onto the index?
// TODO check up on Delay.h doxygen documentation.
// TODO convert Ian Stavness' example to use this Delay component.
// TODO changes to Coordinate properties.
// TODO Test VectorDelay.
// TODO write stage test.
Model createPendulumModel() {
    Model model;
    // The link rests along the x axis.
    auto* body = new OpenSim::Body("body", 1, Vec3(1, 0, 0), Inertia(0));
    PinJoint* joint = new PinJoint("joint",
            model.getGroundBody(), Vec3(0), Vec3(0),
            *body, Vec3(0), Vec3(0));

    model.addBody(body);
    model.addJoint(joint);

    return model;
}

/** Simple proportional controller, with the option of using a delayed value
 * for the coordinate.
 */
class PendulumController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(PendulumController, Controller);
public:
    OpenSim_DECLARE_PROPERTY(delay, double,
            "Use a delayed coordinate value. Default: 0, for no delay.");
    PendulumController() {
        constructInfrastructure();
        _delay.setName("coordinate_delay");
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
    // Just for testing purposes to see if the Delay itself works.
    double getDelayedCoordinateValue(const State& s) {
        return _delay.getOutputValue<double>(s, "output");
    }
    SimTK::Vec3 getDelayedCOMPositionValue(const State& s) {
        return _comPositionDelay.getValue(s);
    }

private:
    void constructProperties() override {
        constructProperty_delay(0.0);
    }
    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();
        _delay.set_delay(get_delay());
        _comPositionDelay.set_delay(get_delay());
        addComponent(&_delay);
        addComponent(&_comPositionDelay);
    }
    void extendConnectToModel(Model& model) override {
        Super::extendConnectToModel(model);
        const auto& coord = model.getCoordinateSet()[0];
        _delay.getInput("input").connect(coord.getOutput("value"));
        _comPositionDelay.getInput("input").connect(
                model.getOutput("com_position"));
    }

    ScalarDelay _delay;
    Delay<SimTK::Vec3> _comPositionDelay;
};

/// Allows some testing of Delay with different models.
void testDelaySimulation(Model& model) {

    double finalTime = 1.35;
    double delayTime = 0.017;
    double tol = 1e-6; // integrator accuracy, test tolerance.

    const Coordinate& coord = model.getCoordinateSet()[0];
    std::string coordName = coord.getName();
    auto* controller = dynamic_cast<PendulumController*>(
            &model.updControllerSet()[0]);

    // First simulate without controller delay.
    double finalCoordValue_noControllerDelay;
    SimTK::Vec3 finalCOMValue_noControllerDelay;
    {
        State& s = model.initSystem();
        SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
        integrator.setAccuracy(tol);
        SimTK::TimeStepper ts(model.getSystem(), integrator);
        ts.initialize(s);
        ts.stepTo(finalTime);
        s = ts.getState();

        finalCoordValue_noControllerDelay = coord.getValue(s);
        finalCOMValue_noControllerDelay = model.calcMassCenterPosition(s);
    }

    // Simulate with controller delay and record expected delayed value.
    double expectedDelayedCoordValue;
    double actualDelayedCoordValue;
    double finalCoordValue_withControllerDelay;
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
        expectedDelayedCOMValue = model.calcMassCenterPosition(s);

        // Simulate with controller delay to the finalTime.
        ts.stepTo(finalTime);
        s = ts.getState();

        // Must realize in order to access delayed value.
        model.realizeTime(s);
        actualDelayedCoordValue = controller->getDelayedCoordinateValue(s);
        actualDelayedCOMValue = controller->getDelayedCOMPositionValue(s);
        finalCoordValue_withControllerDelay = coord.getValue(s);
        finalCOMValue_withControllerDelay = model.calcMassCenterPosition(s);
    }

    // Basic check on the Delay within the controller to see that the Delay
    // itself is working as a subcomponent.
    SimTK_TEST_EQ_TOL(expectedDelayedCoordValue, actualDelayedCoordValue, tol);
    SimTK_TEST_EQ_TOL(expectedDelayedCOMValue, actualDelayedCOMValue, tol);

    // Make sure the final coordinate value is not the same with vs.
    // without controller delay, to show that the controller delay had an
    // effect.
    SimTK_TEST_NOTEQ_TOL(finalCoordValue_noControllerDelay,
            finalCoordValue_withControllerDelay, tol);
    SimTK_TEST_NOTEQ_TOL(finalCOMValue_noControllerDelay,
            finalCOMValue_withControllerDelay, tol);

    // Regression. Since above we are only checking if the controller with
    // delay and the controller without delay produce different results,
    // the above test is not sufficient to check if behavior changes.
    SimTK_TEST_EQ_TOL(finalCoordValue_noControllerDelay, -0.245182, tol);
    SimTK_TEST_EQ_TOL(finalCoordValue_withControllerDelay, -0.171928, tol);
    SimTK_TEST_EQ_TOL(finalCOMValue_noControllerDelay,
                      Vec3(0.970093, -0.242733, 0.0), tol);
    SimTK_TEST_EQ_TOL(finalCOMValue_withControllerDelay,
                      Vec3(0.985257, -0.171082, 0.0), tol);
}

void testDelay() {
    Object::registerType(Delay<SimTK::Vec3>());

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
    // TODO can't do currently, since stages are set before connecting.
    
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
    // TODO SimTK_START_TEST("testOperators");
        SimTK_SUBTEST(testDelay);
    //SimTK_END_TEST();
}
