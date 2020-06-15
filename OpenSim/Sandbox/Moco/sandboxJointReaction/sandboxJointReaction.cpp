/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxJointReaction.cpp                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include <Moco/osimMoco.h>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>

using namespace OpenSim;

/// This model is torque-actuated.
Model createInvertedPendulumModel() {
    Model model;
    model.setName("inverted_pendulum");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create one link with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* b0 = new OpenSim::Body("b0", 1, Vec3(0), Inertia(1));
    model.addBody(b0);

    // Connect the body to ground with a pin joint. Assume each body is 1 m 
    // long.
    auto* j0 = new PinJoint("j0", model.getGround(), Vec3(0), Vec3(0),
        *b0, Vec3(-1, 0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    model.addJoint(j0);

    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    model.addComponent(tau0);

    // Add display geometry.
    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    SimTK::Transform transform(SimTK::Vec3(-0.5, 0, 0));
    auto* b0Center = new PhysicalOffsetFrame("b0_center", "b0", transform);
    b0->addComponent(b0Center);
    b0Center->attachGeometry(bodyGeometry.clone());

    return model;
}

class JointReactionCost : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(JointReactionCost, MocoGoal);
protected:
    void calcIntegrandImpl(const IntegrandInput& input,
            SimTK::Real& integrand) const override {

        getModel().realizeAcceleration(state);
        const auto& j0 = getModel().getJointSet().get("j0");

        SimTK::SpatialVec reaction =
            j0.calcReactionOnChildExpressedInGround(state);

        input = reaction.norm();
    }
    void calcCostImpl(const GoalInput& input, double& cost) const override {
        cost = input.integral;
    }
};

void minimizePendulumReactionLoads() {
    MocoStudy study;
    study.setName("minimize_pendulum_reaction_loads");
    MocoProblem& mp = study.updProblem();
    mp.setModel(createInvertedPendulumModel());

    mp.setTimeBounds(0, 1);
    mp.setStateInfo("j0/q0/value", {-10, 10}, 0, SimTK::Pi);
    mp.setStateInfo("j0/q0/speed", {-50, 50}, 0, 0);
    mp.setControlInfo("tau0", {-100, 100});

    MocoJointReactionNormCost reactionNormCost;
    reactionNormCost.setJointPath("j0");
    mp.addGoal(reactionNormCost);

    MocoTropterSolver& ms = moco.initSolver();
    ms.set_num_mesh_intervals(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    //ms.set_optim_ipopt_print_level(5);
    ms.setGuess("bounds");

    MocoSolution solution = study.solve();
    solution.write("sandboxJointReaction_minimizePendulumReactionLoads.sto");
    study.visualize(solution);
}

void minimizeControlEffort() {
    MocoStudy study;
    study.setName("minimize_pendulum_reaction_loads");
    MocoProblem& mp = study.updProblem();
    mp.setModel(createInvertedPendulumModel());

    mp.setTimeBounds(0, 1);
    mp.setStateInfo("j0/q0/value", {-10, 10}, 0, SimTK::Pi);
    mp.setStateInfo("j0/q0/speed", {-50, 50}, 0, 0);
    mp.setControlInfo("tau0", {-100, 100});

    MocoControlGoal effort;
    mp.addGoal(effort);

    MocoTropterSolver& ms = study.initSolver();
    ms.set_num_mesh_intervals(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    //ms.set_optim_ipopt_print_level(5);
    ms.setGuess("bounds");

    MocoSolution solution = study.solve();
    solution.write("sandboxJointReaction_minimizeControlEffort.sto");
    study.visualize(solution);
}

void main() {

    minimizePendulumReactionLoads();
    minimizeControlEffort();
}
