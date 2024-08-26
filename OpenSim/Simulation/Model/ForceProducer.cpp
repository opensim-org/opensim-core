#include "ForceProducer.h"

#include <OpenSim/Simulation/Model/ForceConsumer.h>

using namespace OpenSim;

namespace
{
    class BodyForceAndGeneralizedForceApplicator : public ForceConsumer {
        // TODO: use this `ForceConsumer` to apply each force that pops
        // out of `ForceProducer::produceForces` to the `bodyForces` and
        // `generalizedForces` vectors
    };
}

void OpenSim::ForceProducer::computeForce(
    const SimTK::State& state,
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    SimTK::Vector& generalizedForces) const
{
    // TODO: call `emitPointForceDirections` and apply each PFD to the
    //       relevant part of `bodyForces`
}
