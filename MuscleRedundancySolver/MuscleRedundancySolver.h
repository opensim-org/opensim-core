#ifndef TOMU_MUSCLEREDUNDANCYSOLVER_H
#define TOMU_MUSCLEREDUNDANCYSOLVER_H

#include <OpenSim/OpenSim.h>

// TODO edit DECLARE_PROPERTY macros so that this class does not need to be
// in the OpenSim namespace.
namespace OpenSim {

class MuscleRedundancySolver : public OpenSim::Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleRedundancySolver, OpenSim::Object);
public:

    OpenSim_DECLARE_PROPERTY(model_file, std::string,
        "Path to a model file (.osim).");
    OpenSim_DECLARE_PROPERTY(coordinates_file, std::string,
        "Path to a Storage (.sto) file containing kinematics.");

    MuscleRedundancySolver();
    void run() const;
};

} // namespace OpenSim

#endif // TOMU_MUSCLEREDUNDANCYSOLVER_H
