/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoInverse.cpp                                          *
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

#include <OpenSim/Moco/osimMoco.h>

#define CATCH_CONFIG_MAIN
#include "Testing.h"

using namespace OpenSim;

/// This test case helps debug when it's necessary to call prescribe() versus
/// realize().
TEST_CASE("PrescribedKinematics prescribe() and realize()") {
    Model model = ModelFactory::createPendulum();
    auto* motion = new PositionMotion();
    const double c2 = 1.3;
    const double c1 = 0.17;
    const double c0 = 0.81;
    motion->setPositionForCoordinate(model.getCoordinateSet().get(0),
            PolynomialFunction(createVector({c2, c1, c0})));
    model.addModelComponent(motion);
    auto state = model.initSystem();

    const auto& system = model.getSystem();
    const auto& y = state.getY();
    const auto& ydot = state.getYDot();
    CHECK(y[0] == 0);
    CHECK(y[1] == 0);
    system.prescribe(state);
    CHECK(y[0] == Approx(c0));
    CHECK(y[1] == Approx(c1));
    model.realizeAcceleration(state);
    CHECK(y[0] == Approx(c0));
    CHECK(y[1] == Approx(c1));
    CHECK(ydot[0] == Approx(c1));
    CHECK(ydot[1] == Approx(2 * c2));

    const double t = 0.3;
    state.setTime(t);
    system.prescribe(state);
    CHECK(y[0] == Approx(c0 + c1 * t + c2 * pow(t, 2)));
    CHECK(y[1] == Approx(c1 + 2 * c2 * t));
    model.realizeAcceleration(state);
    CHECK(y[0] == Approx(c0 + c1 * t + c2 * pow(t, 2)));
    CHECK(y[1] == Approx(c1 + 2 * c2 * t));
    CHECK(ydot[0] == Approx(c1 + 2 * c2 * t));
    CHECK(ydot[1] == Approx(2 * c2));
}

#ifdef OPENSIM_WITH_CASADI
TEST_CASE("PrescribedKinematics direct collocation auxiliary dynamics") {

    // Make sure that custom dynamics are still handled properly even when
    // we are skipping over the kinematic/multibody states. That is, this test
    // ensures we handle indices properly.
    class CustomDynamics : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(CustomDynamics, Component);

    public:
        OpenSim_DECLARE_PROPERTY(
                default_s, double, "Default state variable value.");
        CustomDynamics() { constructProperty_default_s(0.4361); }
        void extendAddToSystem(SimTK::MultibodySystem& system) const override {
            Super::extendAddToSystem(system);
            addStateVariable("s");
        }
        void extendInitStateFromProperties(SimTK::State& s) const override {
            Super::extendInitStateFromProperties(s);
            setStateVariableValue(s, "s", get_default_s());
        }
        void extendSetPropertiesFromState(const SimTK::State& s) override {
            Super::extendSetPropertiesFromState(s);
            set_default_s(getStateVariableValue(s, "s"));
        }
        void computeStateVariableDerivatives(
                const SimTK::State& s) const override {
            setStateVariableDerivativeValue(
                    s, "s", getStateVariableValue(s, "s"));
        }
    };
    Model model = ModelFactory::createPendulum();
    model.initSystem();
    auto* motion = new PositionMotion();
    motion->setPositionForCoordinate(
            model.getCoordinateSet().get(0), LinearFunction(1.3, 0.17));
    model.addModelComponent(motion);
    model.addComponent(new CustomDynamics());

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelCopy(model);
    problem.setTimeBounds(0, 1);
    const double init_s = 0.2;
    problem.setStateInfo("/customdynamics/s", {0, 100}, init_s);
    auto& solver = study.initCasADiSolver();
    solver.set_transcription_scheme("trapezoidal");
    solver.set_multibody_dynamics_mode("implicit");

    MocoSolution solution = study.solve();
    OpenSim_CHECK_MATRIX_TOL(solution.getState("/customdynamics/s"),
            0.2 * SimTK::exp(solution.getTime()), 1e-4);
}
#endif

#ifdef OPENSIM_WITH_CASADI
TEST_CASE("MocoInverse Rajagopal2016, 18 muscles") {

    MocoInverse inverse;
    ModelProcessor modelProcessor =
        ModelProcessor("subject_walk_armless_18musc.osim") |
        ModOpReplaceJointsWithWelds({"subtalar_r", "subtalar_l",
            "mtp_r", "mtp_l"}) |
        ModOpReplaceMusclesWithDeGrooteFregly2016() |
        ModOpIgnorePassiveFiberForcesDGF() |
        ModOpTendonComplianceDynamicsModeDGF("implicit") |
        ModOpAddExternalLoads("subject_walk_armless_external_loads.xml");

    inverse.setModel(modelProcessor);
    inverse.setKinematics(
        TableProcessor("subject_walk_armless_coordinates.mot") |
        TabOpLowPassFilter(6));
    inverse.set_initial_time(0.450);
    inverse.set_final_time(1.0);
    inverse.set_kinematics_allow_extra_columns(true);
    inverse.set_mesh_interval(0.05);

    MocoSolution solution = inverse.solve().getMocoSolution();
    solution.write("testMocoInverse_subject_18musc_solution.sto");

    MocoTrajectory std("std_testMocoInverse_subject_18musc_solution.sto");
    const auto expected = std.getControlsTrajectory();
    CHECK(std.compareContinuousVariablesRMS(solution,
            {{"controls", {}}}) < 1e-2);
    CHECK(std.compareContinuousVariablesRMS(solution, {{"states", {}}}) < 1e-2);
}
#endif
