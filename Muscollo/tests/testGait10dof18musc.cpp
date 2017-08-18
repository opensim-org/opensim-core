
#include <GlobalStaticOptimization.h>
#include <INDYGO.h>
#include "testing.h"

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
            std_tendon_force,      "/walk_subject01/tib_ant_r", 1e-3);
}

void testGait10dof18musc_INDYGO() {
    INDYGO mrs("testGait10dof18musc_INDYGO_setup.xml");
    INDYGO::Solution solution = mrs.solve();
    // solution.write("testGait10dof18musc_INDYGO_solution");

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
            "/walk_subject01/reserve_ankle_r_ankle_angle_r", 1e-6);

    TimeSeriesTable std_tendon_force = STOFileAdapter_<double>::read
            ("std_testGait10dof18musc_INDYGO_solution_tendon_force.sto");
    compare(solution.tendon_force, "/walk_subject01/gastroc_r",
            std_tendon_force,      "/walk_subject01/gastroc_r", 1e-1);
    // Forces diverge at the very end of the motion.
    rootMeanSquare(solution.tendon_force, "/walk_subject01/soleus_r",
                   std_tendon_force,      "/walk_subject01/soleus_r", 0.5);
    rootMeanSquare(solution.tendon_force, "/walk_subject01/tib_ant_r",
                   std_tendon_force,      "/walk_subject01/tib_ant_r", 0.5);
}

int main() {
    SimTK_START_TEST("testGait10dof18musc");
        SimTK_SUBTEST(testGait10dof18musc_GSO);
        SimTK_SUBTEST(testGait10dof18musc_INDYGO);
    SimTK_END_TEST();
}
