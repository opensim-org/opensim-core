#include "ForceProducer.h"

#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

using namespace OpenSim;

namespace
{
    // this is an internal `ForceConsumer` that is used by `ForceProducer`'s default
    // `computeForce` implementation
    //
    // it takes each consumed force and adapts it to the underlying `OpenSim::Force::computeForce`
    // API, which makes the default implementation of `ForceProducer::computeForce` entirely
    // compatible with that API.
    class ForceComputingConsumer final : public ForceConsumer {
    public:
        ForceComputingConsumer(
            const OpenSim::Force& force,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& generalizedForces) :

            _force{&force},
            _bodyForces{&bodyForces},
            _generalizedForces{&generalizedForces}
        {}
    private:
        void implConsumeGeneralizedForce(
            const SimTK::State& state,
            const Coordinate& coord,
            double force) final
        {
            _force->applyGeneralizedForce(state, coord, force, *_generalizedForces);
        }

        void implConsumeBodySpatialVec(
            const SimTK::State& state,
            const PhysicalFrame& body,
            const SimTK::SpatialVec& spatialVec) final
        {
            const auto mobilizedBodyIndex = body.getMobilizedBodyIndex();
            OPENSIM_ASSERT_ALWAYS(0 <= mobilizedBodyIndex && mobilizedBodyIndex < _bodyForces->size() && "the provided mobilized body index is out-of-bounds");
            (*_bodyForces)[mobilizedBodyIndex] += spatialVec;
        }

        void implConsumePointForce(
            const SimTK::State& state,
            const PhysicalFrame& frame,
            const SimTK::Vec3& point,
            const SimTK::Vec3& force) final
        {

            _force->applyForceToPoint(state, frame, point, force, *_bodyForces);
        }

        const OpenSim::Force* _force;
        SimTK::Vector_<SimTK::SpatialVec>* _bodyForces;
        SimTK::Vector* _generalizedForces;
    };
}

void OpenSim::ForceProducer::produceForces(
    const SimTK::State& state,
    ForceConsumer& forceConsumer) const
{
    if (appliesForce(state)) {
        implProduceForces(state, forceConsumer);
    }
}

void OpenSim::ForceProducer::computeForce(
    const SimTK::State& state,
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    SimTK::Vector& generalizedForces) const
{
    // create a consumer that uses each produced force to compute the
    // underlying body- and generalized-forces
    ForceComputingConsumer consumer{*this, bodyForces, generalizedForces};

    // produce forces and feed them into the consumer, satisfying the `computeForce` API
    produceForces(state, consumer);
}
