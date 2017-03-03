#include <iostream>
#include "MuscleRedundancySolver.h"

int main(int argc, char* argv[]) {
    OpenSim::MuscleRedundancySolver mrs;
    mrs.solve();
    return EXIT_SUCCESS;
}
