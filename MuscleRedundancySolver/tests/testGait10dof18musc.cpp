
#include <GlobalStaticOptimizationSolver.h>
#include <MuscleRedundancySolver.h>
#include "testing.h"

using namespace OpenSim;

void testGait10dof18musc_GSO() {
    // TODO muscle-tendon velocities are zero, as expected.
    // TODO why is this not an issue for other test cases? because we provide
    // the speed states.
    GlobalStaticOptimizationSolver gso("testGait10dof18musc_GSO_setup.xml");
    GlobalStaticOptimizationSolver::Solution solution = gso.solve();
    solution.write("testGait10dof18musc_GSO_solution");
}

void testGait10dof18musc_MRS() {
    MuscleRedundancySolver mrs("testGait10dof18musc_MRS_setup.xml");
    MuscleRedundancySolver::Solution solution = mrs.solve();
    solution.write("testGait10dof18musc_MRS_solution");
}

int main() {
    SimTK_START_TEST("testGait10dof18musc");
        SimTK_SUBTEST(testGait10dof18musc_GSO);
        SimTK_SUBTEST(testGait10dof18musc_MRS);
    SimTK_END_TEST();
}
