/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Control/SynergyController.h>
#include <OpenSim/Common/SignalGenerator.h>
#include <OpenSim/Common/Constant.h>

using namespace OpenSim;

namespace {

    // Based on ModelFactory::createNLinkPendulum(), but allows us to add the 
    // CoordinateActuators to the ForceSet, which is necessary for some of the 
    // tests below.
    Model createTriplePendulum() {
        Model model; 
        model.setName("triple_pendulum");
        const auto& ground = model.getGround();

        using SimTK::Inertia;
        using SimTK::Vec3;

        Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
        bodyGeometry.setColor(SimTK::Gray);

        const PhysicalFrame* prevBody = &ground;
        for (int i = 0; i < 3; ++i) {
            const std::string istr = std::to_string(i);
            auto* bi = new OpenSim::Body("b" + istr, 1, Vec3(0), Inertia(1));
            model.addBody(bi);

            // Assume each body is 1 m long.
            auto* ji = new PinJoint("j" + istr, *prevBody, Vec3(0), Vec3(0), 
                    *bi, Vec3(-1, 0, 0), Vec3(0));
            auto& qi = ji->updCoordinate();
            qi.setName("q" + istr);
            model.addJoint(ji);

            auto* taui = new CoordinateActuator();
            taui->setCoordinate(&ji->updCoordinate());
            taui->setName("tau" + istr);
            taui->setOptimalForce(1);
            model.addForce(taui);

            auto* marker = new Marker("marker" + istr, *bi, Vec3(0));
            model.addMarker(marker);

            // Attach an ellipsoid to a frame located at the center of each body.
            PhysicalOffsetFrame* bicenter = new PhysicalOffsetFrame(
                    "b" + istr + "center", *bi, 
                    SimTK::Transform(Vec3(-0.5, 0, 0)));
            bi->addComponent(bicenter);
            bicenter->attachGeometry(bodyGeometry.clone());

            prevBody = bi;
        }

        model.finalizeConnections();
        return model;
    }

    // class TriplePendulumController : public InputController {
    //     OpenSim_DECLARE_CONCRETE_OBJECT(
    //             TriplePendulumController, InputController);
    // public:
    //     TriplePendulumController() {
    //         m_synergyVectors.resize(3, 2);
    //         m_synergyVectors(0, 0) = 0.25;
    //         m_synergyVectors(0, 1) = 0.5;
    //         m_synergyVectors(1, 0) = 0.75;
    //         m_synergyVectors(1, 1) = 0.5;
    //         m_synergyVectors(2, 0) = 0.5;
    //         m_synergyVectors(2, 1) = 0.25;
    //     }
    //     std::vector<std::string> getInputControlLabels() const override {
    //         return {"synergy_control_0", "synergy_control_1"};
    //     }
    //     void computeControlsImpl(const SimTK::State& state,
    //             SimTK::Vector& controls) const override {
    //         const auto& input = getInput<double>("controls");
    //         for (int i = 0; i < static_cast<int>(input.getNumConnectees()); ++i) 
    //         {
    //             controls += m_synergyVectors.col(i) * input.getValue(state, i);
    //         }

    //     }
    //     const SimTK::Matrix& getSynergyVectors() const { 
    //         return m_synergyVectors; 
    //     }
    // private:
    //     SimTK::Matrix m_synergyVectors;
    // };

    Model createControlledTriplePendulumModel(bool controllerEnabled = true) {
        Model model = createTriplePendulum();
        auto* controller = new SynergyController();
        controller->setName("synergy_controller");
        controller->addActuator(
                model.getComponent<CoordinateActuator>("/forceset/tau0"));
        controller->addActuator(
                model.getComponent<CoordinateActuator>("/forceset/tau1"));
        controller->addActuator(
                model.getComponent<CoordinateActuator>("/forceset/tau2"));
        controller->addSynergyVector(createVector({0.25, 0.75, 0.5}));
        controller->addSynergyVector(createVector({0.5, 0.5, 0.25}));
        controller->setEnabled(controllerEnabled);
        model.addController(controller);
        model.finalizeConnections();
        return model;
    }

    template<typename SolverType = MocoCasADiSolver>
    MocoStudy createTriplePendulumMocoStudy(const Model& model,
            bool ignoreControlledActuators = false,
            bool ignoreInputControls = false) {
        MocoStudy study;
        auto& problem = study.updProblem(); 
        problem.setModelAsCopy(model);
        problem.setTimeBounds(0, 0.5);
        problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, -0.5);
        problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0);
        problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
        problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
        problem.setStateInfo("/jointset/j2/q2/value", {-10, 10}, 0);
        problem.setStateInfo("/jointset/j2/q2/speed", {-50, 50}, 0);
        MocoBounds bounds(-100, 100);
        const auto& controller = model.getComponent<SynergyController>(
                "/controllerset/synergy_controller");
        if (controller.isEnabled()) {
            problem.setInputControlInfo(
                "/controllerset/synergy_controller/synergy_excitation_0", 
                bounds);
            problem.setInputControlInfo(
                "/controllerset/synergy_controller/synergy_excitation_1", 
                bounds);
        } else {
            problem.setControlInfo("/forceset/tau0", bounds);
            problem.setControlInfo("/forceset/tau1", bounds);
            problem.setControlInfo("/forceset/tau2", bounds);
        }

        auto* effort = problem.addGoal<MocoControlGoal>();
        effort->setName("effort");
        effort->setIgnoreControlledActuators(ignoreControlledActuators);
        effort->setIgnoreInputControls(ignoreInputControls);
        auto& solver = study.initSolver<SolverType>();
        solver.set_num_mesh_intervals(50);
        return study;
    }
}

int main() {

    Model model = createControlledTriplePendulumModel();
    SimTK::Vector constants = SimTK::Test::randVector(2);
    SignalGenerator* constant0 = new SignalGenerator();
    constant0->set_function(Constant(constants[0]));
    model.addComponent(constant0);
    SignalGenerator* constant1 = new SignalGenerator();
    constant1->set_function(Constant(constants[1]));
    model.addComponent(constant1);

    auto& controller = model.updComponent<SynergyController>(
            "/controllerset/synergy_controller");
    controller.connectInput_controls(constant0->getOutput("signal"));
    controller.connectInput_controls(constant1->getOutput("signal"));
    model.finalizeConnections();
    SimTK::State state = model.initSystem();
    model.realizeVelocity(state);


    SimTK::Matrix synergyVectors = controller.getSynergyVectorsAsMatrix();
    SimTK::Vector expected = synergyVectors * constants;
    SimTK::Vector controls = model.getControls(state);

    return EXIT_SUCCESS;
}
