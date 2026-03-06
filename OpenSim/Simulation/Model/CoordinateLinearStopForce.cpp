/* -------------------------------------------------------------------------- *
 *                     OpenSim:  CoordinateLinearStop.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2026 Stanford University and the Authors                     *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "CoordinateLinearStopForce.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
CoordinateLinearStopForce::CoordinateLinearStopForce() {
    constructProperties();
}

CoordinateLinearStopForce::CoordinateLinearStopForce(
        const std::string& coordinateNameOrPath, double stiffness,
        double damping, double lowerLimit, double upperLimit) {
    constructProperties();
    set_coordinate(coordinateNameOrPath);
    set_stiffness(stiffness);
    set_damping(damping);
    set_lower_limit(lowerLimit);
    set_upper_limit(upperLimit);
}

void CoordinateLinearStopForce::constructProperties() {
    constructProperty_coordinate("");
    constructProperty_stiffness(0.0);
    constructProperty_damping(0.0);
    constructProperty_upper_limit(0.0);
    constructProperty_lower_limit(0.0);
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void CoordinateLinearStopForce::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    OPENSIM_THROW_IF_FRMOBJ(get_stiffness() < 0.0, Exception,
        "Expected a non-negative stiffness, but received {}.", get_stiffness());
    OPENSIM_THROW_IF_FRMOBJ(get_damping() < 0.0, Exception,
        "Expected a non-negative damping, but received {}.", get_damping());
    OPENSIM_THROW_IF_FRMOBJ(get_lower_limit() > get_upper_limit(), Exception,
        "Expected lower limit to be less than or equal to upper limit, but "
        "received {} and {}, respectively.", get_lower_limit(),
        get_upper_limit());

    const auto& coord = getCoordinate();
    const SimTK::MobilizedBody& mobod =
        system.getMatterSubsystem().getMobilizedBody(coord.getBodyIndex());
    SimTK::GeneralForceSubsystem& forces = _model->updForceSubsystem();
    SimTK::Force::MobilityLinearStop stop(forces, mobod,
        coord.getMobilizerQIndex(), get_stiffness(), get_damping(),
        get_lower_limit(), get_upper_limit());

    CoordinateLinearStopForce* mutableThis =
        const_cast<CoordinateLinearStopForce*>(this);
    mutableThis->_index = stop.getForceIndex();
}

//=============================================================================
// FORCE INTERFACE
//=============================================================================
double CoordinateLinearStopForce::computePotentialEnergy(
        const SimTK::State& state) const {
    return getMobilityLinearStop().calcPotentialEnergyContribution(state);
}

//=============================================================================
// REPORTING
//=============================================================================
Array<std::string> CoordinateLinearStopForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(getName());
    labels.append("PotentialEnergy");
    return labels;
}

Array<double> CoordinateLinearStopForce::getRecordValues(
        const SimTK::State& state) const {
    const SimTK::Force::MobilityLinearStop& stop = getMobilityLinearStop();
    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);
    stop.calcForceContribution(state, bodyForces, particleForces,
        mobilityForces);

    OpenSim::Array<double> values(0.0, 0, 2);
    values.append(mobilityForces[getCoordinate().getMobilizerQIndex()]);
    values.append(computePotentialEnergy(state));
    return values;
}

//=============================================================================
// HELPERS
//=============================================================================
const Coordinate& CoordinateLinearStopForce::getCoordinate() const {
    const auto& name = get_coordinate();
    if (_model->getCoordinateSet().contains(name))
        return _model->getCoordinateSet().get(name);
    if (_model->hasComponent<Coordinate>(name))
        return _model->getComponent<Coordinate>(name);
    OPENSIM_THROW_FRMOBJ(Exception,
        "Received '{}' from property 'coordinate', but no coordinate "
        "with this name or path was found in the model.", name);
}

const SimTK::Force::MobilityLinearStop&
CoordinateLinearStopForce::getMobilityLinearStop() const {
    SimTK_ASSERT(_index.isValid(), "Invalid Force index.");
    const auto& forceSubsys = getModel().getForceSubsystem();
    const SimTK::Force& abstractForce = forceSubsys.getForce(_index);
    return static_cast<const SimTK::Force::MobilityLinearStop&>(abstractForce);
}
