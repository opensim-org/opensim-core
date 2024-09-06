#include "ForceApplier.h"

#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Simulation/Model/ForceApplier.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

void OpenSim::ForceApplier::implConsumeGeneralizedForce(
    const SimTK::State& state,
    const Coordinate& coord,
    double force)
{
    _matter->addInMobilityForce(
        state,
        SimTK::MobilizedBodyIndex(coord.getBodyIndex()),
        SimTK::MobilizerUIndex(coord.getMobilizerQIndex()),
        force,
        *_generalizedForces
    );
}

void OpenSim::ForceApplier::implConsumeBodySpatialVec(
    const SimTK::State& state,
    const PhysicalFrame& body,
    const SimTK::SpatialVec& spatialVec)
{
    const auto mobilizedBodyIndex = body.getMobilizedBodyIndex();
    OPENSIM_ASSERT_ALWAYS(0 <= mobilizedBodyIndex && mobilizedBodyIndex < _bodyForces->size() && "the provided mobilized body index is out-of-bounds");
    (*_bodyForces)[mobilizedBodyIndex] += spatialVec;
}

void OpenSim::ForceApplier::implConsumePointForce(
    const SimTK::State& state,
    const PhysicalFrame& frame,
    const SimTK::Vec3& point,
    const SimTK::Vec3& force)
{
    _matter->addInStationForce(
        state,
        frame.getMobilizedBodyIndex(),
        frame.findTransformInBaseFrame() * point,
        force,
        *_bodyForces
    );
}