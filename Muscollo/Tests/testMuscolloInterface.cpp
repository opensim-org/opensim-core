/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: testMuscolloInterface.cpp                                *
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

#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

using namespace OpenSim;

// TODO
// - add setGuess
// - add documentation. pre/post conditions.
// - write test cases for exceptions, for calling methods out of order.
// - model_file vs model.

void testEmpty() {
    // It's possible to solve an empty problem.
    MucoTool muco;
    MucoSolution solution = muco.solve();
    // 100 is the default num_mesh_points.
    SimTK_TEST(solution.getTime().size() == 100);
    SimTK_TEST(solution.getStatesTrajectory().ncol() == 0);
    SimTK_TEST(solution.getStatesTrajectory().nrow() == 0);
    SimTK_TEST(solution.getControlsTrajectory().ncol() == 0);
    SimTK_TEST(solution.getControlsTrajectory().nrow() == 0);
}

Model createSlidingMassModel() {
    Model model;
    model.setName("sliding_mass");
    model.set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 2.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model.addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model.addComponent(actu);

    return model;
}

MucoTool createSlidingMassMucoTool() {
    MucoTool muco;
    muco.setName("sliding_mass");
    muco.set_write_solution("false");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createSlidingMassModel());
    mp.setTimeBounds(MucoInitialBounds(0), MucoFinalBounds(0, 5));
    mp.setStateInfo("slider/position/value", MucoBounds(-5, 5),
            MucoInitialBounds(0), MucoFinalBounds(1));
    mp.setStateInfo("slider/position/speed", {-50, 50}, 0, 0);
    mp.setControlInfo("actuator", MucoBounds(-50, 50));
    MucoFinalTimeCost ftCost;
    mp.addCost(ftCost);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(20);
    MucoSolution solution = muco.solve();

    return muco;
}

void testOrderingOfCalls() {

    // Solve a problem, edit the problem, re-solve.
    {
        // It's fine to
        MucoTool muco = createSlidingMassMucoTool();
        auto& solver = muco.initSolver();
        muco.solve();
        // This flips the "m_solverInitialized" flag:
        muco.updProblem();
        // This will call initSolver() internally:
        muco.solve();
    }

    // Solve a problem, edit the problem, ask the solver to do something.
    {
        MucoTool muco = createSlidingMassMucoTool();
        auto& solver = muco.initSolver();
        muco.solve();
        // This resets the problem to null on the solver.
        muco.updProblem();
        // The solver can't do anything if you've edited the model.
        SimTK_TEST_MUST_THROW_EXC(solver.getProblem(), Exception);
        SimTK_TEST_MUST_THROW_EXC(solver.solve(), Exception);
    }

    // Solve a problem, edit the solver, re-solve.
    {
        MucoTool muco = createSlidingMassMucoTool();
        auto& solver = muco.initSolver();
        const int initNumMeshPoints = solver.get_num_mesh_points();
        MucoSolution sol0 = muco.solve();
        solver.set_num_mesh_points(2 * initNumMeshPoints);
        MucoSolution sol1 = muco.solve();
        solver.set_num_mesh_points(initNumMeshPoints);
        MucoSolution sol2 = muco.solve();
        // Ensure that changing the mesh has an effect.
        SimTK_TEST(!sol0.isNumericallyEqual(sol1));
        // Ensure we get repeatable results with the initial settings.
        SimTK_TEST(sol0.isNumericallyEqual(sol2));

    }
}

/// Test that we can read in a Muscollo setup file, solve, edit the setup,
/// re-solve.
void testOMUCOSerialization() {
    std::string fname = "testMuscolloInterface_testOMUCOSerialization.omuco";
    MucoSolution sol0;
    MucoSolution sol1;
    {
        MucoTool muco = createSlidingMassMucoTool();
        sol0 = muco.solve();
        muco.print(fname);
    }
    {
        MucoTool mucoDeserialized(fname);
        MucoSolution sol1 = mucoDeserialized.solve();
    }
    SimTK_TEST(sol0.isNumericallyEqual(sol1));
}

void testCopy() {
    MucoTool muco = createSlidingMassMucoTool();
    MucoSolution solution = muco.solve();
    std::unique_ptr<MucoTool> copy(muco.clone());
    MucoSolution solutionFromCopy = copy->solve();
    SimTK_TEST(solution.isNumericallyEqual(solutionFromCopy));


    // TODO what happens if just the MucoProblem is copied, or if just the
    // MucoSolver is copied?
}

void testBounds() {
    // TODO what to do about clamped coordinates? Use the range in the
    // coordinate, or ignore that? I think that if the coordinate is clamped,
    // then

    // By default, the bounds for coordinates, if clamped, are the
    // coordinate's range.

    // TODO create intermediate class, Delegate or Proxy or Decorator
    // TODO or simply have a "finalize()" method on MucoProblem.
    {
        MucoTool muco;
        MucoProblem& mp = muco.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setStateInfo("slider/position/value", {}, 0, 0);
        // TODO something...
        SimTK_TEST(state_bounds.lower == coord.get_range(0));
        SimTK_TEST(state_bounds.upper == coord.get_range(0));
    }

    // TODO what to do if the user does not specify info for some variables?

    // Get error if state/control name does not exist.
    {
        MucoTool muco;
        MucoProblem& mp = muco.updProblem();
        mp.setModel(createSlidingMassModel());
        SimTK_TEST_MUST_THROW_EXC(mp.setStateInfo("nonexistant", {0, 1}),
                Exception);
        SimTK_TEST_MUST_THROW_EXC(mp.setControlInfo("nonexistant", {0, 1}),
                Exception);
    }


    // TODO what if bounds are missing for some states?
}

void testBuildingProblem() {
    {
        MucoTool muco;
        MucoProblem& mp = muco.updProblem();
        mp.setModel(createSlidingMassModel());

        // TODO ensure that model must be set ... what did I mean?

        // Costs have the name "cost" by default.
        {
            MucoFinalTimeCost c0;
            SimTK_TEST(c0.getName() == "cost");
            mp.addCost(c0);
        }
        // Names of costs must be unique.
        {
            MucoFinalTimeCost c1;
            SimTK_TEST_MUST_THROW_EXC(mp.addCost(c1), Exception);
        }
        // Costs must have a name.
        {
            MucoFinalTimeCost cEmptyName;
            cEmptyName.setName("");
            SimTK_TEST_MUST_THROW_EXC(mp.addCost(cEmptyName), Exception);
        }

    }
}

int main() {
    SimTK_START_TEST("testMuscolloInterface");
        SimTK_SUBTEST(testEmpty);
        SimTK_SUBTEST(testCopy);
        SimTK_SUBTEST(testSolveRepeatedly);
        SimTK_SUBTEST(testOMUCOSerialization);
        SimTK_SUBTEST(testBounds);
        SimTK_SUBTEST(testBuildingProblem);
        // TODO what happens when Ipopt does not converge.
        // TODO specifying optimizer options.
    SimTK_END_TEST();
}