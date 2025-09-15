#include "ForceApplier.h"

#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Simulation/Model/ForceApplier.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

void OpenSim::ForceApplier::implConsumeGeneralizedForce(
    const SimTK::State& state,
    SimTK::MobilizedBodyIndex mobodIndex,
    SimTK::MobilizerUIndex uIndex,
    double force)
{
    _matter->addInMobilityForce(
        state,
        mobodIndex,
        uIndex,
        force,
        *_generalizedForces
    );
}

void OpenSim::ForceApplier::implConsumeBodySpatialVec(
    const SimTK::State& state,
    SimTK::MobilizedBodyIndex mobodIndex,
    const SimTK::SpatialVec& spatialVec)
{
    OPENSIM_ASSERT_ALWAYS(0 <= mobodIndex && mobodIndex < _bodyForces->size() && "the provided mobilized body index is out-of-bounds");
    (*_bodyForces)[mobodIndex] += spatialVec;
}

void OpenSim::ForceApplier::implConsumePointForce(
    const SimTK::State& state,
    SimTK::MobilizedBodyIndex mobodIndex,
    const SimTK::Vec3& point,
    const SimTK::Vec3& force)
{
    _matter->addInStationForce(
        state,
        mobodIndex,
        point,
        force,
        *_bodyForces
    );
}
