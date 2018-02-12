/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxMuscle.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

// Some of this code is based on testSingleMuscle,
// testSingleMuscleDeGrooteFregly2016.

#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <MuscolloSandboxShared.h>
#include <OpenSim/Simulation/Model/PathActuator.h>

using namespace OpenSim;

/// DGF stands for DeGroote-Fregly. The abbreviation is temporary, and is used
/// to avoid a name conflict with the existing DeGrooteFregly2016Muscle class
/// (which doesn't inherit from OpenSim::Force).
class /*OSIMMUSCOLLO_API*/DGF2016Muscle : public PathActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(DGF2016Muscle, PathActuator);
public:
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
    "Smaller value means activation can change more rapidly (units: seconds).");

    OpenSim_DECLARE_PROPERTY(default_activation, double,
    "Value of activation in the default state returned by initSystem().");

    DGF2016Muscle() {
        constructProperties();
    }

    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("activation", SimTK::Stage::Dynamics);
    }

    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "activation", get_default_activation());
    }

    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_activation(getStateVariableValue(s, "activation"));
    }

    // TODO no need to do clamping, etc; CoordinateActuator is bidirectional.
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        const auto& tau = get_activation_time_constant();
        const auto& u = getControl(s);
        const auto& a = getStateVariableValue(s, "activation");
        const SimTK::Real adot = (u - a) / tau;
        setStateVariableDerivativeValue(s, "activation", adot);
    }

    double computeActuation(const SimTK::State& s) const override {
        return getStateVariableValue(s, "activation") * get_optimal_force();
    }
private:
    void constructProperties() {
        constructProperty_activation_time_constant(0.010);
        constructProperty_default_activation(0.5);
    }
};

Model createHangingMuscleModel() {
    Model model;
    model.setName("isometric_muscle");
    model.set_gravity(SimTK::Vec3(9.81, 0, 0));
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("height");
    model.addComponent(joint);

    auto* actu = new DGF2016Muscle();
    actu->set_optimal_force(30);

    // auto* actu = new Thelen2003Muscle();

    // auto* actu = new Millard2012EquilibriumMuscle();
    // actu->set_fiber_damping(0); // TODO
    actu->setName("actuator");
    //actu->set_max_isometric_force(30.0);
    //actu->set_optimal_fiber_length(0.10);
    //actu->set_tendon_slack_length(0.05);
    //actu->set_pennation_angle_at_optimal(0.1);
    //actu->set_max_contraction_velocity(10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(actu);

    /*
    auto* actu = new ActivationCoordinateActuator();
    actu->setName("actuator");
    actu->setCoordinate(&coord);
    actu->set_activation_time_constant(0.001);
    actu->set_optimal_force(30);
    model.addComponent(actu);

    auto* contr = new PrescribedController();
    contr->setName("controller");
    contr->addActuator(*actu);
    contr->prescribeControlForActuator("actuator", new Constant(1.0));
    model.addComponent(contr);
    */

    body->attachGeometry(new Sphere(0.05));

    return model;
}

int main() {
    MucoTool muco;
    MucoProblem& mp = muco.updProblem();
    Model model = createHangingMuscleModel();
    SimTK::State state = model.initSystem();
    model.setStateVariableValue(state, "joint/height/value", 0.15);
    // model.equilibrateMuscles(state);

    // TODO
    //Manager manager(model, state);
    //manager.integrate(2.0);
    //
    //visualize(model, manager.getStateStorage());
    //std::exit(-1);

    //std::cout << "DEBUG " <<
    //        model.getStateVariableValue(state, "actuator/fiber_length")
    //        << std::endl;
    //model.equilibrateMuscles(state);
    //std::cout << "DEBUG " <<
    //        model.getStateVariableValue(state, "actuator/fiber_length")
    //        << std::endl;
    mp.setModel(model);
    mp.setTimeBounds(0, {0.1, 1.0});
    mp.setStateInfo("joint/height/value", {0, 0.3}, 0.15, 0.14);
    mp.setStateInfo("joint/height/speed", {-10, 10}, 0, 0);
    // TODO initial fiber length?
    // TODO how to enforce initial equilibrium?
    // mp.setStateInfo("actuator/fiber_length", {0, 0.3},
    //         model.getStateVariableValue(state, "actuator/fiber_length"));
    // OpenSim might not allow activations of 0.
    mp.setStateInfo("actuator/activation", {0, 1}, 0);
    mp.setControlInfo("actuator", {0, 1});

    mp.addCost(MucoFinalTimeCost());

    // TODO try ActivationCoordinateActuator first.
    // TODO i feel like the force-velocity effect is much more strict than it
    // should be.

    // MucoTropterSolver& ms = muco.initSolver();
    MucoSolution solution = muco.solve().unseal();
    solution.write("sandboxMuscle_solution.sto");
    std::cout << "DEBUG " << solution.getState("joint/height/value") << std::endl;
    std::cout << "DEBUG " << solution.getState("joint/height/speed") << std::endl;


    return EXIT_SUCCESS;
}

