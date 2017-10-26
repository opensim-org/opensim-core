/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: testGait10dof18musc.cpp                                  *
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

#include <Muscollo/InverseMuscleSolver/GlobalStaticOptimization.h>
#include <Muscollo/InverseMuscleSolver/INDYGO.h>

#include <OpenSim/Simulation/osimSimulation.h>

#include "Testing.h"

using namespace OpenSim;

void testGait10dof18musc_GSO() {
    // TODO muscle-tendon velocities are zero, as expected.
    // TODO why is this not an issue for other test cases? because we provide
    // the speed states.
    GlobalStaticOptimization gso("testGait10dof18musc_GSO_setup.xml");
    GlobalStaticOptimization::Solution solution = gso.solve();
    // solution.write("testGait10dof18musc_GSO_solution");


    // Regression tests.
    TimeSeriesTable std_activation = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_GSO_solution_activation.sto");
    compare(solution.activation, "/walk_subject01/hamstrings_r",
            std_activation,      "/walk_subject01/hamstrings_r", 1e-3);
    compare(solution.activation, "/walk_subject01/bifemsh_r",
            std_activation,      "/walk_subject01/bifemsh_r", 1e-3);
    compare(solution.activation, "/walk_subject01/glut_max_r",
            std_activation,      "/walk_subject01/glut_max_r", 1e-3);
    compare(solution.activation, "/walk_subject01/iliopsoas_r",
            std_activation,      "/walk_subject01/iliopsoas_r", 1e-3);
    compare(solution.activation, "/walk_subject01/rect_fem_r",
            std_activation,      "/walk_subject01/rect_fem_r", 1e-3);
    compare(solution.activation, "/walk_subject01/vasti_r",
            std_activation,      "/walk_subject01/vasti_r", 1e-3);
    compare(solution.activation, "/walk_subject01/gastroc_r",
            std_activation,      "/walk_subject01/gastroc_r", 1e-3);
    compare(solution.activation, "/walk_subject01/soleus_r",
            std_activation,      "/walk_subject01/soleus_r", 1e-3);
    compare(solution.activation, "/walk_subject01/tib_ant_r",
            std_activation,      "/walk_subject01/tib_ant_r", 1e-3);

    TimeSeriesTable std_norm_fiber_length = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_GSO_solution_norm_fiber_length.sto");
    compare(solution.norm_fiber_length, "/walk_subject01/hamstrings_r",
            std_norm_fiber_length,      "/walk_subject01/hamstrings_r", 1e-5);
    compare(solution.norm_fiber_length, "/walk_subject01/bifemsh_r",
            std_norm_fiber_length,      "/walk_subject01/bifemsh_r", 1e-5);
    compare(solution.norm_fiber_length, "/walk_subject01/glut_max_r",
            std_norm_fiber_length,      "/walk_subject01/glut_max_r", 1e-5);
    compare(solution.norm_fiber_length, "/walk_subject01/iliopsoas_r",
            std_norm_fiber_length,      "/walk_subject01/iliopsoas_r", 1e-5);
    compare(solution.norm_fiber_length, "/walk_subject01/rect_fem_r",
            std_norm_fiber_length,      "/walk_subject01/rect_fem_r", 1e-5);
    compare(solution.norm_fiber_length, "/walk_subject01/vasti_r",
            std_norm_fiber_length,      "/walk_subject01/vasti_r", 1e-5);
    compare(solution.norm_fiber_length, "/walk_subject01/gastroc_r",
            std_norm_fiber_length,      "/walk_subject01/gastroc_r", 1e-5);
    compare(solution.norm_fiber_length, "/walk_subject01/soleus_r",
            std_norm_fiber_length,      "/walk_subject01/soleus_r", 1e-5);
    compare(solution.norm_fiber_length, "/walk_subject01/tib_ant_r",
            std_norm_fiber_length,      "/walk_subject01/tib_ant_r", 1e-5);

    TimeSeriesTable std_norm_fiber_velocity = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_GSO_solution_norm_fiber_velocity.sto");
    compare(solution.norm_fiber_velocity, "/walk_subject01/hamstrings_r",
            std_norm_fiber_velocity,      "/walk_subject01/hamstrings_r", 1e-5);
    compare(solution.norm_fiber_velocity, "/walk_subject01/bifemsh_r",
            std_norm_fiber_velocity,      "/walk_subject01/bifemsh_r", 1e-5);
    compare(solution.norm_fiber_velocity, "/walk_subject01/glut_max_r",
            std_norm_fiber_velocity,      "/walk_subject01/glut_max_r", 1e-5);
    compare(solution.norm_fiber_velocity, "/walk_subject01/iliopsoas_r",
            std_norm_fiber_velocity,      "/walk_subject01/iliopsoas_r", 1e-5);
    compare(solution.norm_fiber_velocity, "/walk_subject01/rect_fem_r",
            std_norm_fiber_velocity,      "/walk_subject01/rect_fem_r", 1e-5);
    compare(solution.norm_fiber_velocity, "/walk_subject01/vasti_r",
            std_norm_fiber_velocity,      "/walk_subject01/vasti_r", 1e-5);
    compare(solution.norm_fiber_velocity, "/walk_subject01/gastroc_r",
            std_norm_fiber_velocity,      "/walk_subject01/gastroc_r", 1e-5);
    compare(solution.norm_fiber_velocity, "/walk_subject01/soleus_r",
            std_norm_fiber_velocity,      "/walk_subject01/soleus_r", 1e-5);
    compare(solution.norm_fiber_velocity, "/walk_subject01/tib_ant_r",
            std_norm_fiber_velocity,      "/walk_subject01/tib_ant_r", 1e-5);

    TimeSeriesTable std_other_controls = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_GSO_solution_other_controls.sto");
    compare(solution.other_controls,
            "/walk_subject01/reserve_hip_r_hip_flexion_r",
            std_other_controls,
            "/walk_subject01/reserve_hip_r_hip_flexion_r", 1e-6);
    compare(solution.other_controls,
            "/walk_subject01/reserve_knee_r_knee_angle_r",
            std_other_controls,
            "/walk_subject01/reserve_knee_r_knee_angle_r", 1e-6);
    compare(solution.other_controls,
            "/walk_subject01/reserve_ankle_r_ankle_angle_r",
            std_other_controls,
            "/walk_subject01/reserve_ankle_r_ankle_angle_r", 1e-6);

    TimeSeriesTable std_tendon_force = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_GSO_solution_tendon_force.sto");
    compare(solution.tendon_force, "/walk_subject01/hamstrings_r",
            std_tendon_force,      "/walk_subject01/hamstrings_r", 1e-3);
    compare(solution.tendon_force, "/walk_subject01/bifemsh_r",
            std_tendon_force,      "/walk_subject01/bifemsh_r", 1e-3);
    compare(solution.tendon_force, "/walk_subject01/glut_max_r",
            std_tendon_force,      "/walk_subject01/glut_max_r", 1e-3);
    compare(solution.tendon_force, "/walk_subject01/iliopsoas_r",
            std_tendon_force,      "/walk_subject01/iliopsoas_r", 1e-3);
    compare(solution.tendon_force, "/walk_subject01/rect_fem_r",
            std_tendon_force,      "/walk_subject01/rect_fem_r", 1e-3);
    compare(solution.tendon_force, "/walk_subject01/vasti_r",
            std_tendon_force,      "/walk_subject01/vasti_r", 1e-3);
    compare(solution.tendon_force, "/walk_subject01/gastroc_r",
            std_tendon_force,      "/walk_subject01/gastroc_r", 1e-3);
    compare(solution.tendon_force, "/walk_subject01/soleus_r",
            std_tendon_force,      "/walk_subject01/soleus_r", 1e-3);
    compare(solution.tendon_force, "/walk_subject01/tib_ant_r",
            std_tendon_force,      "/walk_subject01/tib_ant_r", 5e-3, 1);

    TimeSeriesTable std_norm_tendon_force = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_GSO_solution_norm_tendon_force.sto");
    compare(solution.norm_tendon_force, "/walk_subject01/hamstrings_r",
            std_norm_tendon_force,      "/walk_subject01/hamstrings_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/bifemsh_r",
            std_norm_tendon_force,      "/walk_subject01/bifemsh_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/glut_max_r",
            std_norm_tendon_force,      "/walk_subject01/glut_max_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/iliopsoas_r",
            std_norm_tendon_force,      "/walk_subject01/iliopsoas_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/rect_fem_r",
            std_norm_tendon_force,      "/walk_subject01/rect_fem_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/vasti_r",
            std_norm_tendon_force,      "/walk_subject01/vasti_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/gastroc_r",
            std_norm_tendon_force,      "/walk_subject01/gastroc_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/soleus_r",
            std_norm_tendon_force,      "/walk_subject01/soleus_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/tib_ant_r",
            std_norm_tendon_force,      "/walk_subject01/tib_ant_r", 1e-3);
}

void testGait10dof18musc_INDYGO(const std::string& fiberDynamicsMode,
        const std::string& activationDynamicsMode,
        const int& meshPointFrequency) {
    INDYGO mrs("testGait10dof18musc_INDYGO_setup.xml");
    mrs.set_fiber_dynamics_mode(fiberDynamicsMode);
    mrs.set_activation_dynamics_mode(activationDynamicsMode);
    mrs.set_mesh_point_frequency(meshPointFrequency);
    INDYGO::Solution solution = mrs.solve();
    //solution.write("testGait10dof18musc_INDYGO_solution");

    // Regression tests.
    TimeSeriesTable std_activation = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_INDYGO_solution_activation.sto");
    compare(solution.activation, "/walk_subject01/gastroc_r",
            std_activation,      "/walk_subject01/gastroc_r", 1e-3);
    compare(solution.activation, "/walk_subject01/soleus_r",
            std_activation,      "/walk_subject01/soleus_r", 1e-3);
    compare(solution.activation, "/walk_subject01/tib_ant_r",
            std_activation,      "/walk_subject01/tib_ant_r", 1e-3);

    TimeSeriesTable std_excitation = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_INDYGO_solution_excitation.sto");
    compare(solution.excitation, "/walk_subject01/gastroc_r",
            std_excitation,      "/walk_subject01/gastroc_r", 1e-3);
    compare(solution.excitation, "/walk_subject01/soleus_r",
            std_excitation,      "/walk_subject01/soleus_r", 1e-3);
    compare(solution.excitation, "/walk_subject01/tib_ant_r",
            std_excitation,      "/walk_subject01/tib_ant_r", 1e-3);

    TimeSeriesTable std_norm_fiber_length = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_INDYGO_solution_norm_fiber_length.sto");
    compare(solution.norm_fiber_length, "/walk_subject01/gastroc_r",
            std_norm_fiber_length,      "/walk_subject01/gastroc_r", 1e-2);
    compare(solution.norm_fiber_length, "/walk_subject01/soleus_r",
            std_norm_fiber_length,      "/walk_subject01/soleus_r", 1e-2);
    compare(solution.norm_fiber_length, "/walk_subject01/tib_ant_r",
            std_norm_fiber_length,      "/walk_subject01/tib_ant_r", 1e-2);

    TimeSeriesTable std_norm_fiber_velocity = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_INDYGO_solution_norm_fiber_velocity.sto");
    compare(solution.norm_fiber_velocity, "/walk_subject01/gastroc_r",
            std_norm_fiber_velocity,      "/walk_subject01/gastroc_r", 1e-1);
    compare(solution.norm_fiber_velocity, "/walk_subject01/soleus_r",
            std_norm_fiber_velocity,      "/walk_subject01/soleus_r", 1e-2);
    compare(solution.norm_fiber_velocity, "/walk_subject01/tib_ant_r",
            std_norm_fiber_velocity,      "/walk_subject01/tib_ant_r", 1e-2);

    TimeSeriesTable std_other_controls = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_INDYGO_solution_other_controls.sto");
    compare(solution.other_controls,
            "/walk_subject01/reserve_ankle_r_ankle_angle_r",
            std_other_controls,
            "/walk_subject01/reserve_ankle_r_ankle_angle_r", 1e-4);

    TimeSeriesTable std_tendon_force = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_INDYGO_solution_tendon_force.sto");
    compare(solution.tendon_force, "/walk_subject01/gastroc_r",
            std_tendon_force,      "/walk_subject01/gastroc_r", 0.5);
    // Forces diverge at the very end of the motion.
    rootMeanSquare(solution.tendon_force, "/walk_subject01/soleus_r",
                   std_tendon_force,      "/walk_subject01/soleus_r", 0.5);
    rootMeanSquare(solution.tendon_force, "/walk_subject01/tib_ant_r",
                   std_tendon_force,      "/walk_subject01/tib_ant_r", 0.5);

    TimeSeriesTable std_norm_tendon_force = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_INDYGO_solution_norm_tendon_force.sto");
    compare(solution.norm_tendon_force, "/walk_subject01/gastroc_r",
            std_norm_tendon_force,      "/walk_subject01/gastroc_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/soleus_r",
            std_norm_tendon_force,      "/walk_subject01/soleus_r", 1e-3);
    compare(solution.norm_tendon_force, "/walk_subject01/tib_ant_r",
            std_norm_tendon_force,      "/walk_subject01/tib_ant_r", 1e-3);
}

int main() {
    SimTK_START_TEST("testGait10dof18musc");
        SimTK_SUBTEST(testGait10dof18musc_GSO);
        SimTK_SUBTEST3(testGait10dof18musc_INDYGO,
                "fiber_length", "explicit", 300);
        SimTK_SUBTEST3(testGait10dof18musc_INDYGO,
                "tendon_force", "explicit", 300);
//        SimTK_SUBTEST2(testGait10dof18musc_INDYGO, "fiber_length", "implicit");
//        SimTK_SUBTEST2(testGait10dof18musc_INDYGO, "tendon_force", "implicit");
    SimTK_END_TEST();
}
