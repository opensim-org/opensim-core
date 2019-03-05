/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoCosts.cpp                                            *
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

#include <Moco/osimMoco.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

using namespace OpenSim;

std::unique_ptr<Model> createSlidingMassModel() {
    auto model = make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model->addComponent(actu);

    return model;
}

/// Test the result of a sliding mass minimum effort problem.
template <typename SolverType>
void testMocoControlCost() {
    int N = 10;
    MocoSolution sol1;
    {
        MocoTool moco;
        moco.setName("sliding_mass");
        MocoProblem& mp = moco.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setTimeBounds(0, {0, 5});
        mp.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("/actuator", MocoBounds(-10, 10));

        mp.addCost<MocoControlCost>();

        auto& ms = moco.initSolver<SolverType>();
        ms.set_num_mesh_points(N);

        sol1 = moco.solve();
        sol1.write("testMocoCosts_testMocoControlCost_sol1.sto");

        // Minimum effort solution is a linear control.
        SimTK_TEST_EQ_TOL(sol1.getControl("/actuator"),
                createVectorLinspace(N, 2.23, -2.23), 0.25);
        // Symmetry.
        SimTK_TEST_EQ_TOL(sol1.getControl("/actuator").getElt(0, 0),
                -sol1.getControl("/actuator").getElt(N-1, 0), 1e-3);

        // Minimum effort solution takes as long as possible.
        SimTK_TEST_EQ_TOL(sol1.getTime().getElt(N-1, 0), 5, 1e-7);
    }

    // TODO test that we can ignore specific actuators.
    // TODO for now, the weight can just be set to 0 (not ideal).
    //{

    //    MocoTool moco;
    //    moco.setName("sliding_mass");
    //    MocoProblem& mp = moco.updProblem();
    //    MocoProblem& mp = moco.updProblem();
    //    auto model = createSlidingMassModel();

    //    auto* actu = new CoordinateActuator();
    //    actu->setCoordinate(&model.getCoordinateSet().get("position"));
    //    actu->setName("actuator2");
    //    actu->setOptimalForce(1);
    //    model.addComponent(actu);

    //    mp.setModel(model);
    //    mp.setTimeBounds(0, 5);
    //    mp.setStateInfo("slider/position/value", {0, 1}, 0, 1);
    //    mp.setStateInfo("slider/position/speed", {-100, 100}, 0, 0);
    //    mp.setControlInfo("actuator", MocoBounds(-10, 10));
    //    mp.setControlInfo("actuator2", MocoBounds(-10, 10));

    //    MocoControlCost effort;
    //    effort.addActuatorToInclude("actuator");
    //    effort.set
    //    mp.addCost(effort);

    //    MocoTropterSolver& ms = moco.initSolver();
    //    ms.set_num_mesh_points(N);

    //    MocoSolution solution = moco.solve();
    //    SimTK_TEST_EQ(solution.getControl("actuator2"), SimTK::Vector(N, 0));
    //}

    // TODO test from XML.

    // Ensure that the weights cause one actuator to be preferred over
    // another.
    MocoSolution sol2;
    std::string omocoFile = "testMocoCosts_testMocoControlCost.omoco";
    {
        MocoTool moco;
        moco.setName("sliding_mass");
        moco.set_write_solution("false");
        MocoProblem& mp = moco.updProblem();
        auto model = createSlidingMassModel();

        auto* actu = new CoordinateActuator();
        actu->setCoordinate(&model->updCoordinateSet().get("position"));
        actu->setName("actuator2");
        actu->setOptimalForce(1);
        model->addComponent(actu);

        mp.setModel(std::move(model));
        mp.setTimeBounds(0, 5);
        mp.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("/actuator", MocoBounds(-10, 10));
        mp.setControlInfo("/actuator2", MocoBounds(-10, 10));

        auto effort = mp.addCost<MocoControlCost>();
        effort->setWeight("actuator2", 2.0);

        auto& ms = moco.initSolver<SolverType>();
        ms.set_num_mesh_points(N);

        sol2 = moco.solve();

        moco.print(omocoFile);

        // The actuator with the lower weight is more active.
        SimTK_TEST_EQ_TOL(sol2.getControl("/actuator"),
                2 * sol2.getControl("/actuator2"), 1e-5);
        // Sum of control for these two actuators is the same as the control
        // in the single-actuator case.
        SimTK_TEST_EQ_TOL(sol2.getControlsTrajectory().rowSum(),
            sol1.getControl("/actuator"), 1e-3);
    }

    // Cannot set a weight for a nonexistent control.
    {
        MocoTool moco;
        moco.setName("sliding_mass");
        MocoProblem& mp = moco.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setTimeBounds(0, {0, 5});
        mp.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("/actuator", MocoBounds(-10, 10));
        auto effort = mp.addCost<MocoControlCost>();
        effort->setWeight("nonexistent", 1.5);
        SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
    }

    // De/serialization.
    {
        MocoTool moco(omocoFile);
        MocoSolution solDeserialized = moco.solve();
        sol2.write("DEBUG_sol2.sto");
        solDeserialized.write("DEBUG_solDeserialized.sto");
        SimTK_TEST(solDeserialized.isNumericallyEqual(sol2, 1e-5));
    }
}

/// Make sure that multiple costs are added together properly.
void testMultipleCosts() {
    MocoTool moco;
    MocoProblem& problem = moco.updProblem();

    auto* ft0 = problem.addCost<MocoFinalTimeCost>("ft0", 0.1);

    auto* ft1 = problem.addCost<MocoFinalTimeCost>("ft1", 0.2);

    MocoProblemRep rep = problem.createRep();
    SimTK::State state = rep.getModelBase().getWorkingState();
    const double ft = 0.35;
    state.setTime(ft);

    const double cost = rep.calcEndpointCost(state);
    SimTK_TEST_EQ(cost, (ft0->get_weight() + ft1->get_weight() ) * ft);
}

int main() {
    SimTK_START_TEST("testMocoCosts");
        SimTK_SUBTEST(testMocoControlCost<MocoTropterSolver>);
        SimTK_SUBTEST(testMocoControlCost<MocoCasADiSolver>);
        SimTK_SUBTEST(testMultipleCosts);
    SimTK_END_TEST();
}
