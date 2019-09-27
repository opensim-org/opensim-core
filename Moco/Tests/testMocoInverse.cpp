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

#include <Moco/osimMoco.h>

#include <OpenSim/Common/LogManager.h>
#include <OpenSim/Tools/CMC_TaskSet.h>
#include <OpenSim/Tools/CMC_Joint.h>
#include <OpenSim/Tools/CMC.h>
#include <OpenSim/Tools/CMCTool.h>

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

    MocoStudy moco;
    auto& problem = moco.updProblem();
    problem.setModelCopy(model);
    problem.setTimeBounds(0, 1);
    const double init_s = 0.2;
    problem.setStateInfo("/customdynamics/s", {0, 100}, init_s);
    auto& solver = moco.initCasADiSolver();
    solver.set_transcription_scheme("trapezoidal");
    solver.set_dynamics_mode("implicit");

    MocoSolution solution = moco.solve();
    OpenSim_CHECK_MATRIX_TOL(solution.getState("/customdynamics/s"),
            0.2 * SimTK::exp(solution.getTime()), 1e-4);
}

SimTK::Vector smoothColumn(SimTK::Vector col, double fwhm, 
        std::vector<bool> smoothIndices) {
    double sigma = fwhm / sqrt(8.0 * log(2.0));
    SimTK::Vector x(col.size());
    for (int i = 0; i < col.size(); ++i) x[i] = i;

    SimTK::Vector smoothCol(col.size());
    for (int i = 0; i < col.size(); ++i) {

        if (smoothIndices[i]) {
            auto x_sub = x.elementwiseSubtractScalar((double)i).getAsVector();
            auto x_sub_sq = x_sub.elementwiseMultiply(x_sub);
            double denom = 2.0 * sigma * sigma;
            x_sub_sq /= denom;
            auto x_exp = -x_sub_sq;

            SimTK::Vector kernel(col.size());
            for (int j = 0; j < col.size(); ++j) {
                kernel[j] = std::exp(x_exp[j]);
            }
            kernel /= kernel.sum();

            smoothCol[i] = col.elementwiseMultiply(kernel).sum();
        } else {
            smoothCol[i] = col[i];
        }
        
    }

    return smoothCol;
}

TEST_CASE("MocoInverse gait10dof18musc") {
    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cerr.rdbuf(LogManager::cerr.rdbuf());

    //TimeSeriesTable grfOrigTable(
    //        "testMocoInverse_subject01_walk2_ground_reaction.mot");

    //auto time = grfOrigTable.getIndependentColumn();
    //TimeSeriesTable grfTable(time);

    //std::vector<bool> smoothIndices(time.size(), false);
    //std::vector<std::pair<double, double>> windows;
    //windows.push_back({2.298, 2.301});
    //windows.push_back({2.445, 2.448});
    //windows.push_back({2.849, 2.852});
    //windows.push_back({3.001, 3.004});
    //windows.push_back({3.390, 3.393});
    //windows.push_back({3.549, 3.552});

    //double pad = 0.05;
    //for (const auto& window : windows) {
    //    int startIdx = (int)grfOrigTable.getNearestRowIndexForTime(window.first-pad, true);
    //    int endIdx = (int)grfOrigTable.getNearestRowIndexForTime(window.second+pad, true);

    //    int idx = startIdx;
    //    while (idx <= endIdx) {
    //        smoothIndices[idx] = true;
    //        ++idx;
    //    }
    //}

    //for (const auto& colLabel : grfOrigTable.getColumnLabels()) {
    //    auto col = grfOrigTable.getDependentColumn(colLabel);
    //    auto smoothCol = smoothColumn(col, 50, smoothIndices);

    //    grfTable.appendColumn(colLabel, smoothCol);
    //}

    //STOFileAdapter::write(filterLowpass(grfTable, 25),
    //        "testMocoInverse_subject01_walk2_ground_reaction_splined.mot");

    MocoInverse inverse;
    // Psoas muscle max isometric force increased 1000 N
    // Tib ant muscle max isometric force increased 500 N
    ModelProcessor modelProcessor =
        ModelProcessor("testMocoInverse_subject01_scaled_Fmax.osim") |
        ModOpReplaceJointsWithWelds({"subtalar_r", "subtalar_l",
            "mtp_r", "mtp_l"}) |
        ModOpReplaceMusclesWithDeGrooteFregly2016() |
        ModOpIgnoreTendonCompliance() |
        ModOpIgnorePassiveFiberForcesDGF() |
        ModOpAddExternalLoads("testMocoInverse_subject01_walk2_external_loads.xml");

    //modelProcessor.process().print("testMocoInverse_subject01_scaled_Fmax_welds.osim");

    inverse.setModel(modelProcessor);
    inverse.setKinematics(
        TableProcessor("testMocoInverse_subject01_walk2_ik_solution.mot") |
        TabOpLowPassFilter(6));
    inverse.set_initial_time(2.282);
    inverse.set_final_time(3.361);
    inverse.set_kinematics_allow_extra_columns(true);
    inverse.set_mesh_interval(0.02);
    inverse.set_tolerance(1e-4);
    //inverse.set_reserves_weight(1000);

    MocoSolution solution = inverse.solve().getMocoSolution();
    solution.write("testMocoInverse_subject01_walk2_solution.sto");

    ModelProcessor modelProcessorTendonCompliance =
        ModelProcessor("testMocoInverse_subject01_scaled_Fmax.osim") |
        ModOpReplaceJointsWithWelds({"subtalar_r", "subtalar_l",
            "mtp_r", "mtp_l"}) |
        ModOpReplaceMusclesWithDeGrooteFregly2016() |
        ModOpIgnorePassiveFiberForcesDGF() |
        ModOpUseImplicitTendonComplianceDynamicsDGF() |
        ModOpAddExternalLoads("testMocoInverse_subject01_walk2_external_loads.xml");
    inverse.setModel(modelProcessorTendonCompliance);

    MocoSolution solutionTendonCompliance = inverse.solve().getMocoSolution();
    solutionTendonCompliance.write(
        "testMocoInverse_subject01_walk2_solution_tendon_compliance.sto");

    ModelProcessor modelProcessorCMC =
        ModelProcessor("testMocoInverse_subject01_scaled_Fmax.osim") |
        ModOpReplaceJointsWithWelds({"subtalar_r", "subtalar_l",
            "mtp_r", "mtp_l"}) |
        ModOpReplaceMusclesWithDeGrooteFregly2016() |
        ModOpIgnoreTendonCompliance() |
        ModOpIgnorePassiveFiberForcesDGF();
    modelProcessorCMC.process().print(
        "testMocoInverse_subject01_scaled_Fmax_cmc.osim");

    Muscle::setPrintWarnings(false);
    auto cmc = CMCTool("testMocoInverse_subject01_walk2_cmc_setup.xml");
    cmc.run();
    Muscle::setPrintWarnings(true);


    //MocoInverse inverse;
    //ModelProcessor modelProcessor = 
    //    ModelProcessor("testGait10dof18musc_subject01_residuals.osim") |
    //    ModOpReplaceMusclesWithDeGrooteFregly2016() |
    //    ModOpIgnoreTendonCompliance() |
    //    ModOpIgnorePassiveFiberForcesDGF() |
    //    ModOpAddExternalLoads("walk_gait1018_subject01_grf.xml");

    //inverse.setModel(modelProcessor);
    //inverse.setKinematics(TableProcessor("walk_gait1018_state_reference.mot") |
    //                      TabOpLowPassFilter(6));
    //inverse.set_initial_time(0.1);
    //inverse.set_final_time(1.3);
    //inverse.set_tolerance(1e-5);
    //inverse.set_mesh_interval(0.01);

    //inverse.print("testMocoInverse_setup.xml");

    //MocoSolution solution = inverse.solve().getMocoSolution();
    ////hey gi
    //std::vector<std::string> outputPaths;
    //Model model = modelProcessor.process();
    //for (const auto& muscle : model.getComponentList<Muscle>()) {
    //    outputPaths.push_back(muscle.getAbsolutePathString() + 
    //            "\\|active_force_length_multiplier");
    //}

    //TimeSeriesTable outputTable = analyze<double>(model, solution, outputPaths);
    //STOFileAdapter::write(outputTable, "testMocoInverse_multipliers.sto");

    //solution.write("testMocoInverseGait10dof18musc_solution.sto");
    //const auto actual = solution.getControlsTrajectory();

    //MocoTrajectory std("std_testMocoInverseGait10dof18musc_solution.sto");
    //const auto expected = std.getControlsTrajectory();
    //CHECK(std.compareContinuousVariablesRMS(
    //              solution, {{"controls", {}}}) < 1e-4);
    //CHECK(std.compareContinuousVariablesRMS(
    //              solution, {{"states", {}}}) < 1e-4);

    //ModelProcessor modelProcessorTendonCompliance = 
    //    ModelProcessor("testGait10dof18musc_subject01_residuals.osim") |
    //    ModOpReplaceMusclesWithDeGrooteFregly2016() |
    //    ModOpUseImplicitTendonComplianceDynamicsDGF() |
    //    ModOpIgnorePassiveFiberForcesDGF() |
    //    ModOpAddExternalLoads("walk_gait1018_subject01_grf.xml");

    //inverse.setModel(modelProcessorTendonCompliance);

    //inverse.print("testMocoInverse_tendon_compliance_setup.xml");

    //MocoSolution solutionTendonCompliance = inverse.solve().getMocoSolution();

    //solutionTendonCompliance.write(
    //    "testMocoInverseGait10dof18musc_tendon_compliance_solution.sto");


    //ModelProcessor modelProcessorTendonCompliance2 =
    //    ModelProcessor("testGait10dof18musc_subject01_residuals.osim") |
    //    ModOpReplaceMusclesWithDeGrooteFregly2016() |
    //    //ModOpUseImplicitTendonComplianceDynamicsDGF() |
    //    ModOpAddExternalLoads("walk_gait1018_subject01_grf.xml");
    //TimeSeriesTable outputTableTendonCompliance = 
    //    analyze<double>(modelProcessorTendonCompliance2.process(),
    //        solutionTendonCompliance, outputPaths);
    //STOFileAdapter::write(outputTableTendonCompliance,
    //        "testMocoInverse_tendon_compliance_multipliers.sto");


}
