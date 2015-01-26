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
        constructProperties();
    }
    PendulumController(double delay) : PendulumController() {
        set_delay(delay);
    }
    void computeControls(const State& s, Vector &controls) const override {
        double q;
        if (get_delay() > 0) {
            q = _delay.getOutputValue<double>(s, "output");
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
private:
    void constructProperties() {
        constructProperty_delay(0.0);
    }
    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();
        _delay.set_delay(get_delay());
    }
    void extendConnectToModel(Model& model) override {
        Super::extendConnectToModel(model);
        if (get_delay() > 0) {
            const auto& coord = model.getCoordinateSet()[0];
            _delay.getInput("input").connect(coord.getOutput("value"));
            addComponent(&_delay);
        }
    }

    Delay _delay;
};

void testDelay() {

    double expectedDelayedValue;
    double finalTime = 1.0;
    double delayTime = 0.017;

    // Check basic functionality.
    // ==========================
    
    // Create a pendulum model.
    // ------------------------
    Model model = createPendulumModel();
    const Coordinate& coord = model.getCoordinateSet()[0];
    std::string coordName = coord.getName();
    /*
    Delay* delay = new Delay();
    delay->setName("delay");
    model.addModelComponent(delay);

    // Configure Delay.
    // ----------------
    // TODO when is it permissible to perform the code below?
    // TODO how to wire this up via a serialized file?
    delay->getInput("input").connect(coord.getOutput("value"));
    delay->set_delay(delayTime);


    {
        // Integrate and record expected delayed value.
        // --------------------------------------------
        State& s = model.initSystem();
        Manager manager(model);
        manager.setInitialTime(0.0);
        manager.setFinalTime(finalTime - delayTime);
        manager.integrate(s);

        expectedDelayedValue = coord.getValue(s);

        // Integrate further and test if the our delay worked.
        // ---------------------------------------------------
        manager.setInitialTime(finalTime - delayTime);
        manager.setFinalTime(finalTime);
        manager.integrate(s);

        SimTK_TEST_EQ(expectedDelayedValue,
                delay->getOutputValue<double>(s, "output"));
    }
    */

    // Check convenience constructor for same basic behavior.
    // ======================================================
    /*
    {
        Model model2 = model;
        Delay* delay2 = new Delay(coord.getOutput("value"), delayTime);
        delay2->setName("delay2");
        model2.addMiscModelComponent(delay2);

        State& s2 = model2.initSystem();
        Manager manager2(model2);
        manager2.setInitialTime(0.0);
        manager2.setFinalTime(finalTime);
        manager2.integrate();

        SimTK_TEST_EQ(expectedDelayedValue,
                delay.getOutputValue<double>(s, "output"));
    }
    */

    /*
    // Check for same behavior upon copying a model containing the Delay.
    // ==================================================================
    {
        Model model2 = model;
        State& s2 = model2.initSystem();
        Manager manager2(model2);
        manager2.setInitialTime(0.0);
        manager2.setFinalTime(finalTime);
        manager2.integrate(s2);
        const Delay* delay2 = dynamic_cast<const Delay*>(
                &model2.getMiscModelComponentSet().get("delay"));
        SimTK_TEST_EQ(expectedDelayedValue,
                delay2->getOutputValue<double>(s2, "output"));
    }

    // Check for same behavior upon serializing and deserializing.
    // ===========================================================
    {
        std::string filename = "pendulum_for_delay_test.osim";
        model.print(filename);
        Model model2(filename);
        State& s2 = model2.initSystem();
        Manager manager2(model2);
        manager2.setInitialTime(0.0);
        manager2.setFinalTime(finalTime);
        manager2.integrate(s2);
        const Delay* delay2 = dynamic_cast<Delay*>(
                &model2.getMiscModelComponentSet().get("delay"));
        SimTK_TEST_EQ(expectedDelayedValue,
                delay2->getOutputValue<double>(s2, "output"));
    }
    */

    // Check usage of Delay as a subcomponent.
    // =======================================
    {
        // Add actuator and controller to model.
        Model model2 = model;
        CoordinateActuator* act = new CoordinateActuator(coordName);
        act->setName("joint_0_actuator");
        model2.addForce(act);

        PendulumController* controller = new PendulumController();
        controller->addActuator(*act);
        model2.addController(controller);

        const Coordinate& coord2 = model2.getCoordinateSet()[0];

        // First simulate without controller delay.
        State& s2 = model2.initSystem();
        Manager manager2(model2);
        manager2.setInitialTime(0.0);
        manager2.setFinalTime(finalTime);
        manager2.integrate(s2);

        double finalCoordinateValue_noControllerDelay = coord2.getValue(s2);

        // Simulate with controller delay and record expected delayed value.
        controller->set_delay(delayTime);
        s2 = model2.initializeState();
        manager2.setFinalTime(finalTime - delayTime);
        manager2.integrate(s2);

        double expectedDelayedCoordinateValue = coord2.getValue(s2);

        // Simulate with controller delay to the finalTime.
        manager2.setInitialTime(finalTime - delayTime);
        manager2.setFinalTime(finalTime);
        manager2.integrate(s2);

        double finalCoordinateValue_withControllerDelay = coord2.getValue(s2);

        // Basic check on the Delay within the controller to see that the Delay
        // itself is working as a subcomponent. Can only call this if there was
        // a delay.
        SimTK_TEST_EQ(expectedDelayedCoordinateValue,
            controller->getDelayedCoordinateValue(s2));

        // Make sure the final coordinate values are not the same with and
        // without controller delay, to show that the controller delay had an
        // effect.
        SimTK_TEST_EQ(finalCoordinateValue_noControllerDelay,
            finalCoordinateValue_withControllerDelay);
    }
    
    // Check for possible issues with requiredAt and dependsOn stages.
    // ===============================================================
    // TODO can't do currently, since stages are set before connecting.
}

int main() {
    SimTK_START_TEST("testOperators");
        SimTK_SUBTEST(testDelay);
    SimTK_END_TEST();
}
