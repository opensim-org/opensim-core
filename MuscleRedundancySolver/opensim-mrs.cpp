#include <iostream>
#include "MuscleRedundancySolver.h"

int main(int argc, char* argv[]) {
    OpenSim::MuscleRedundancySolver mrs;
    mrs.run();
    return EXIT_SUCCESS;
}
