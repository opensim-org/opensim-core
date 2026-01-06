/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ExponentialCoordinateLimitForce.cpp             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
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

#include "ExponentialCoordinateLimitForce.h"

#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
ExponentialCoordinateLimitForce::ExponentialCoordinateLimitForce() {
    constructProperties();
}

ExponentialCoordinateLimitForce::ExponentialCoordinateLimitForce(
    const std::string& coordinateNameOrPath, double lowerLimit,
    double upperLimit, const SimTK::Vec2& shapeParametersLower,
    const SimTK::Vec2& shapeParametersUpper) {
    constructProperties();
    set_coordinate(coordinateNameOrPath);
    set_lower_limit(lowerLimit);
    set_upper_limit(upperLimit);
    set_lower_shape_parameters(shapeParametersLower);
    set_upper_shape_parameters(shapeParametersUpper);
}

void ExponentialCoordinateLimitForce::constructProperties() {
    constructProperty_coordinate("");
    constructProperty_lower_limit(0.0);
    constructProperty_upper_limit(0.0);
    constructProperty_lower_shape_parameters(SimTK::Vec2(0.0, 0.0));
    constructProperty_upper_shape_parameters(SimTK::Vec2(0.0, 0.0));
}

//=============================================================================
// METHODS
//=============================================================================
void ExponentialCoordinateLimitForce::extendConnectToModel(Model& aModel) {
    Super::extendConnectToModel(aModel);

    const auto& coordinateNameOrPath = get_coordinate();
    if (_model->getCoordinateSet().contains(coordinateNameOrPath)) {
        _coord = &_model->getCoordinateSet().get(coordinateNameOrPath);
    } else if (_model->hasComponent<Coordinate>(coordinateNameOrPath)) {
        _coord = &_model->getComponent<Coordinate>(coordinateNameOrPath);
    } else {
        OPENSIM_THROW_FRMOBJ(Exception,
            "Received '{}' from property 'coordinate', but no coordinate "
            "with this name or path was found in the model.",
            coordinateNameOrPath);
    }
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
double ExponentialCoordinateLimitForce::calcForce(const SimTK::State& s) const {
    double q = _coord->getValue(s);

    const SimTK::Vec2& s_lower = get_lower_shape_parameters();
    const SimTK::Vec2& s_upper = get_upper_shape_parameters();
    const double& q_lower = get_lower_limit();
    const double& q_upper = get_upper_limit();

    return s_lower[0]*std::exp(-s_lower[1]*(q - q_lower)) +
          -s_upper[0]*std::exp(s_upper[1]*(q - q_upper));
}

double ExponentialCoordinateLimitForce::computePotentialEnergy(
        const SimTK::State& s) const {
    double q = _coord->getValue(s);

    const SimTK::Vec2& s_lower = get_lower_shape_parameters();
    const SimTK::Vec2& s_upper = get_upper_shape_parameters();
    const double& q_lower = get_lower_limit();
    const double& q_upper = get_upper_limit();

    return -s_lower[0]*s_lower[1]*std::exp(-s_lower[1]*(q - q_lower)) +
           -s_upper[0]*s_upper[1]*std::exp(s_upper[1]*(q - q_upper));
}

//=============================================================================
// FORCE PRODUCER INTERFACE
//=============================================================================
void ExponentialCoordinateLimitForce::implProduceForces(const SimTK::State& s,
        ForceConsumer& forceConsumer) const {
    forceConsumer.consumeGeneralizedForce(s, *_coord, calcForce(s));
}

//=============================================================================
// REPORTING
//=============================================================================
Array<std::string> ExponentialCoordinateLimitForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(getName());
    labels.append("PotentialEnergy");
    return labels;
}

Array<double> ExponentialCoordinateLimitForce::getRecordValues(
        const SimTK::State& state) const {
    OpenSim::Array<double> values(0.0, 0, 2);
    values.append(calcForce(state));
    values.append(computePotentialEnergy(state));
    return values;
}
