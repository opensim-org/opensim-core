/* -------------------------------------------------------------------------- *
 *                      OpenSim:  FunctionBasedPath.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
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

#include "FunctionBasedPath.h"
#include "Model.h"

#include <OpenSim/Common/Assertion.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR
//=============================================================================
FunctionBasedPath::FunctionBasedPath() : AbstractGeometryPath()
{
    setAuthors("Nicholas Bianco");
    constructProperties();
}

//=============================================================================
// GET AND SET METHODS
//=============================================================================
void FunctionBasedPath::appendCoordinatePath(const std::string& coordinatePath)
{
    append_coordinate_paths(coordinatePath);
}

void FunctionBasedPath::setCoordinatePaths(
        const std::vector<std::string>& coordinatePaths)
{
    updProperty_coordinate_paths().clear();
    for (const auto& coordinatePath : coordinatePaths) {
        appendCoordinatePath(coordinatePath);
    }
}

std::vector<std::string> FunctionBasedPath::getCoordinatePaths() const
{
    std::vector<std::string> coordinates;
    coordinates.reserve(getProperty_coordinate_paths().size());
    for (int i = 0; i < getProperty_coordinate_paths().size(); ++i) {
        coordinates.push_back(get_coordinate_paths(i));
    }

    return coordinates;
}

void FunctionBasedPath::setLengthFunction(const Function& lengthFunction)
{
    set_length_function(lengthFunction);
}

const Function& FunctionBasedPath::getLengthFunction() const
{
    return get_length_function();
}

void FunctionBasedPath::setLengtheningSpeedFunction(
        const Function& speedFunction)
{
    set_lengthening_speed_function(speedFunction);
}

const Function& FunctionBasedPath::getLengtheningSpeedFunction() const {
    return get_lengthening_speed_function();
}

void FunctionBasedPath::appendMomentArmFunction(
        const Function& momentArmFunction)
{
    append_moment_arm_functions(momentArmFunction);
}

void FunctionBasedPath::setMomentArmFunctions(
        const std::vector<Function>& momentArmFunctions)
{
    for (const auto& momentArmFunction : momentArmFunctions) {
        append_moment_arm_functions(momentArmFunction);
    }
}

const Function& FunctionBasedPath::getMomentArmFunction(
        const std::string& coordinatePath) const
{
    auto it = _coordinateIndices.find(coordinatePath);
    OPENSIM_THROW_IF_FRMOBJ(it == _coordinateIndices.end(), Exception,
            fmt::format("Path does not depend on Coordinate '{}'.",
                    coordinatePath))
    return get_moment_arm_functions(it->second);
}

const SimTK::Vector& FunctionBasedPath::getMomentArms(
        const SimTK::State& s) const {
    computeMomentArms(s);
    return getCacheVariableValue<SimTK::Vector>(s, _momentArmsCV);
}

//=============================================================================
// ABSTRACT PATH INTERFACE
//=============================================================================
double FunctionBasedPath::getLength(const SimTK::State& s) const
{
    computeLength(s);
    return getCacheVariableValue<double>(s, _lengthCV);
}

double FunctionBasedPath::computeMomentArm(const SimTK::State& s,
        const Coordinate& coord) const
{
    auto it = _coordinateIndices.find(coord.getAbsolutePathString());
    if (it != _coordinateIndices.end()) {
        computeMomentArms(s);
        return getCacheVariableValue<SimTK::Vector>(s, _momentArmsCV)
                .get(it->second);
    } else {
        return 0.0;
    }
}

double FunctionBasedPath::getLengtheningSpeed(const SimTK::State& s) const
{
    computeLengtheningSpeed(s);
    return getCacheVariableValue<double>(s, _lengtheningSpeedCV);
}

void FunctionBasedPath::addInEquivalentForces(const SimTK::State& state,
        const double& tension,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& mobilityForces) const
{
    // Get the moment arms.
    computeMomentArms(state);
    const auto& momentArms =
            getCacheVariableValue<SimTK::Vector>(state, _momentArmsCV);
    OPENSIM_ASSERT_ALWAYS(momentArms.size() == (int)_coordinates.size());

    // Apply the mobility forces.
    const SimTK::SimbodyMatterSubsystem& matter =
            getModel().getMatterSubsystem();
    for (int i = 0; i < (int)_coordinates.size(); ++i) {
        const SimTK::MobilizedBody& mobod =
                matter.getMobilizedBody(_coordinates[i]->getBodyIndex());
        mobod.applyOneMobilityForce(state,
                _coordinates[i]->getMobilizerQIndex(),
                momentArms[i] * tension,
                mobilityForces);
    }
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void FunctionBasedPath::constructProperties()
{
    constructProperty_coordinate_paths();
    constructProperty_length_function();
    constructProperty_moment_arm_functions();
    constructProperty_lengthening_speed_function();
}

SimTK::Vector FunctionBasedPath::computeCoordinateValues(
        const SimTK::State& s) const
{
    SimTK::Vector coordinateValues((int)_coordinates.size(), 0.0);
    for (int i = 0; i < (int)_coordinates.size(); ++i) {
        coordinateValues[i] = _coordinates[i]->getValue(s);
    }

    return coordinateValues;
}

SimTK::Vector FunctionBasedPath::computeCoordinateDerivatives(
        const SimTK::State& s) const
{
    SimTK::Vector coordinateSpeeds((int)_coordinates.size(), 0.0);
    for (int i = 0; i < (int)_coordinates.size(); ++i) {
        coordinateSpeeds[i] = _coordinates[i]->getQDotValue(s);
    }

    return coordinateSpeeds;
}

void FunctionBasedPath::computeLength(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _lengthCV)) {
        return;
    }

    setCacheVariableValue(s, _lengthCV,
            getLengthFunction().calcValue(computeCoordinateValues(s)));
}

void FunctionBasedPath::computeMomentArms(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _momentArmsCV)) {
        return;
    }

    const auto& values = computeCoordinateValues(s);
    SimTK::Vector momentArms((int)_coordinates.size(), 0.0);
    if (_computeMomentArms) {
        // If we do not have moment arm functions, then compute the moment arms
        // based on the derivative of the length function with respect to each
        // coordinate.
        for (int i = 0; i < (int)_coordinates.size(); ++i) {
            // Negative sign to obey the OpenSim convention.
            momentArms[i] = -getLengthFunction().calcDerivative({i}, values);
        }
    } else {
        const auto& momentArmFunctions = getProperty_moment_arm_functions();
        for (int i = 0; i < momentArmFunctions.size(); ++i) {
            const auto& momentArmFunction = get_moment_arm_functions(i);
            momentArms[i] = momentArmFunction.calcValue(values);
        }
    }
    setCacheVariableValue(s, _momentArmsCV, momentArms);
}

void FunctionBasedPath::computeLengtheningSpeed(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _lengtheningSpeedCV)) {
        return;
    }

    if (_computeLengtheningSpeed) {
        // If we do not have a speed function, then compute the lengthening
        // speed based on the scalar product between the moment arms and the
        // coordinate derivatives.
        computeMomentArms(s);
        const auto& momentArms =
                getCacheVariableValue<SimTK::Vector>(s, _momentArmsCV);
        const SimTK::Vector& qdot = computeCoordinateDerivatives(s);
        // Negate the moment arms to cancel out the negative sign from the
        // OpenSim convention.
        const double lengtheningSpeed = ~qdot * momentArms.negate();
        setCacheVariableValue(s, _lengtheningSpeedCV, lengtheningSpeed);
    } else {
        SimTK::Vector coordinatesState(2*(int)_coordinates.size(), 0.0);
        for (int i = 0; i < (int)_coordinates.size(); ++i) {
            coordinatesState[i] = _coordinates[i]->getValue(s);
            coordinatesState[i + (int)_coordinates.size()] =
                    _coordinates[i]->getSpeedValue(s);
        }
        setCacheVariableValue(s, _lengtheningSpeedCV,
                getLengtheningSpeedFunction().calcValue(coordinatesState));
    }
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void FunctionBasedPath::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    // Check the properties.
    OPENSIM_THROW_IF_FRMOBJ(getProperty_length_function().empty(),
            Exception, "Expected 'length_function' to be provided, but the "
                       "property was empty.")

    OPENSIM_THROW_IF_FRMOBJ(getProperty_coordinate_paths().empty(), Exception,
            "This path should be dependent on at least one coordinate, but"
            "no coordinates were provided.")

    // Check that the `coordinate_paths` are unique.
    std::vector<std::string> uniqueCoordinatePaths;
    uniqueCoordinatePaths.reserve(getProperty_coordinate_paths().size());
    for (int i = 0; i < getProperty_coordinate_paths().size(); ++i) {
        const auto& coordinatePath = get_coordinate_paths(i);
        if (std::find(uniqueCoordinatePaths.begin(),
                      uniqueCoordinatePaths.end(),
                    coordinatePath) != uniqueCoordinatePaths.end()) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    fmt::format("Coordinate '{}' was provided more than once.",
                            coordinatePath))
        }
        uniqueCoordinatePaths.push_back(coordinatePath);
    }

    OPENSIM_THROW_IF_FRMOBJ(getLengthFunction().getArgumentSize() !=
            getProperty_coordinate_paths().size(), Exception,
            fmt::format("Expected the number of arguments in 'length_function' "
                        "({}) to equal the number of coordinates ({}).",
                        getLengthFunction().getArgumentSize(),
                        getProperty_coordinate_paths().size()))

    if (getProperty_moment_arm_functions().empty()) {
        _computeMomentArms = true;
        OPENSIM_THROW_IF_FRMOBJ(getLengthFunction().getMaxDerivativeOrder() < 1,
                Exception, "Since moment arm functions were not provided, "
                           "expected the length function to be at least "
                           "first-order differentiable with respect to "
                           "the coordinate values, but it is not.")
    } else {
        OPENSIM_THROW_IF_FRMOBJ(getProperty_moment_arm_functions().size() !=
                                getProperty_coordinate_paths().size(), Exception,
                fmt::format("Expected the number of moment arm functions ({}) "
                            "to equal the number of coordinates ({}).",
                        getProperty_moment_arm_functions().size(),
                        getProperty_coordinate_paths().size()))

        for (int i = 0; i < getProperty_moment_arm_functions().size(); ++i) {
            OPENSIM_THROW_IF_FRMOBJ(
                    get_moment_arm_functions(i).getArgumentSize() !=
                            getProperty_coordinate_paths().size(), Exception,
                    fmt::format("Expected the number of arguments in "
                                "'moment_arm_functions[{}]' ({}) to equal the "
                                "number of coordinates ({}).",
                            i, get_moment_arm_functions(i).getArgumentSize(),
                            getProperty_coordinate_paths().size()))
        }
    }

    if (getProperty_lengthening_speed_function().empty()) {
        _computeLengtheningSpeed = true;
    } else {
        OPENSIM_THROW_IF_FRMOBJ(
                getLengtheningSpeedFunction().getArgumentSize() !=
                2*getProperty_coordinate_paths().size(), Exception,
                fmt::format("Expected the number of arguments in "
                            "'speed_function' ({}) to equal to twice the "
                            "number of coordinates ({}).",
                        getLengtheningSpeedFunction().getArgumentSize(),
                        getProperty_coordinate_paths().size()))
    }

    // Populate the coordinate index map. In case "finalizeFromProperties()" is
    // called multiple times, clear the map first.
    _coordinateIndices.clear();
    for (int i = 0; i < getProperty_coordinate_paths().size(); ++i) {
        _coordinateIndices[get_coordinate_paths(i)] = i;
    }
}
void FunctionBasedPath::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // Check that the coordinates are in the model. If so, grab references
    // pointers to them. In case "connectToModel()" is called multiple times,
    // clear any existing references first.
    _coordinates.clear();
    for (int i = 0; i < getProperty_coordinate_paths().size(); ++i) {
        const auto& coordinatePath = get_coordinate_paths(i);
        OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<Coordinate>(coordinatePath),
                Exception,
                fmt::format("Coordinate '{}' does not exist in the model.",
                        coordinatePath))

        _coordinates.emplace_back(
                &model.getComponent<const Coordinate>(coordinatePath));
    }
}
void FunctionBasedPath::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    _lengthCV = addCacheVariable<double>("length", 0.0, SimTK::Stage::Position);
    _momentArmsCV = addCacheVariable<SimTK::Vector>("moment_arms",
            SimTK::Vector(getProperty_coordinate_paths().size(), 0.0),
            SimTK::Stage::Position);
    _lengtheningSpeedCV = addCacheVariable<double>("lengthening_speed", 0.0,
            SimTK::Stage::Velocity);
}
