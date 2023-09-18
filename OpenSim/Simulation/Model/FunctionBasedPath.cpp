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

using namespace OpenSim;

const std::string FunctionBasedPath::LENGTH_NAME("length");
const std::string FunctionBasedPath::MOMENT_ARMS_NAME("moment_arms");
const std::string FunctionBasedPath::LENGTHENING_SPEED_NAME("lengthening_speed");

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
FunctionBasedPath::FunctionBasedPath() : AbstractPath() 
{
    setAuthors("Nicholas Bianco");
    constructProperties();
}

FunctionBasedPath::~FunctionBasedPath() = default;

//=============================================================================
// GET AND SET METHODS
//=============================================================================
void FunctionBasedPath::appendCoordinate(const std::string& coordinateName) 
{
    append_coordinates(coordinateName);
}

void FunctionBasedPath::setCoordinates(
        const std::vector<std::string>& coordinateNames) 
{
    for (const auto& coordName : coordinateNames) {
        appendCoordinate(coordName);
    }
}

std::vector<std::string> FunctionBasedPath::getCoordinates() const 
{
    std::vector<std::string> coordinates;
    for (int i = 0; i < getProperty_coordinates().size(); ++i) {
        coordinates.push_back(get_coordinates(i));
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
        const std::string& coordinateName) const
{
    return get_moment_arm_functions(_coordinateIndices.at(coordinateName));
}

//=============================================================================
// ABSTRACT PATH INTERFACE
//=============================================================================
double FunctionBasedPath::getLength(const SimTK::State& s) const 
{
    computeLength(s);
    return getCacheVariableValue<double>(s, LENGTH_NAME); 
}

double FunctionBasedPath::computeMomentArm(const SimTK::State& s, 
        const Coordinate& coord) const 
{
    computeMomentArms(s);
    return getCacheVariableValue<SimTK::Vector>(s, MOMENT_ARMS_NAME)
            .get(_coordinateIndices.at(coord.getAbsolutePathString()));
}

double FunctionBasedPath::getLengtheningSpeed(const SimTK::State& s) const 
{
    computeLengtheningSpeed(s);
    return getCacheVariableValue<double>(s, LENGTHENING_SPEED_NAME);
}

void FunctionBasedPath::addInEquivalentForces(const SimTK::State& state,
        const double& tension,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& mobilityForces) const 
{
    // Get the moment arms.
    computeMomentArms(state);
    const auto& momentArms = 
            getCacheVariableValue<SimTK::Vector>(state, MOMENT_ARMS_NAME);
    
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
    constructProperty_coordinates();
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
    if (isCacheVariableValid(s, LENGTH_NAME)) {
        return;
    }
    
    setCacheVariableValue(s, LENGTH_NAME, 
            getLengthFunction().calcValue(computeCoordinateValues(s)));
}

void FunctionBasedPath::computeMomentArms(const SimTK::State& s) const 
{
    if (isCacheVariableValid(s, MOMENT_ARMS_NAME)) {
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
    setCacheVariableValue(s, MOMENT_ARMS_NAME, momentArms);
}

void FunctionBasedPath::computeLengtheningSpeed(const SimTK::State& s) const 
{
    if (isCacheVariableValid(s, LENGTHENING_SPEED_NAME)) {
        return;
    }
    
    if (_computeLengtheningSpeed) {
        // If we do not have a speed function, then compute the lengthening
        // speed based on the scalar product between the moment arms and the 
        // coordinate derivatives.
        computeMomentArms(s);
        const auto& momentArms = 
                getCacheVariableValue<SimTK::Vector>(s, MOMENT_ARMS_NAME);
        const SimTK::Vector& qdot = computeCoordinateDerivatives(s);
        // Negate the moment arms to cancel out the negative sign from the
        // OpenSim convention.
        const double lengtheningSpeed = ~qdot * momentArms.negate();
        setCacheVariableValue(s, LENGTHENING_SPEED_NAME, lengtheningSpeed);
    } else {
        SimTK::Vector coordinatesState(2*(int)_coordinates.size(), 0.0);
        for (int i = 0; i < (int)_coordinates.size(); ++i) {
            coordinatesState[i] = _coordinates[i]->getValue(s);
            coordinatesState[i + (int)_coordinates.size()] = 
                    _coordinates[i]->getSpeedValue(s);
        }
        setCacheVariableValue(s, LENGTHENING_SPEED_NAME, 
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
                       "property was empty.");
    
    OPENSIM_THROW_IF_FRMOBJ(getProperty_coordinates().empty(), Exception, 
            "This path should be dependent on at least one coordinate, but"
            "no coordinates were provided.");
    
    OPENSIM_THROW_IF_FRMOBJ(getLengthFunction().getArgumentSize() !=
            getProperty_coordinates().size(), Exception,
            fmt::format("Expected the number of arguments in 'length_function' "
                        "({}) to equal the number of coordinates ({}).",
                        getLengthFunction().getArgumentSize(),
                        getProperty_coordinates().size()));
    
    if (getProperty_moment_arm_functions().empty()) {
        _computeMomentArms = true;
        OPENSIM_THROW_IF_FRMOBJ(getLengthFunction().getMaxDerivativeOrder() < 1, 
                Exception, "Since moment arm functions were not provided, "
                           "expected the length function to be at least "
                           "first-order differentiable with respect to "
                           "the coordinate values, but it is not.");
    } else {
        OPENSIM_THROW_IF_FRMOBJ(getProperty_moment_arm_functions().size() !=
                                getProperty_coordinates().size(), Exception,
                fmt::format("Expected the number of moment arm functions ({}) "
                            "to equal the number of coordinates ({}).",
                        getProperty_moment_arm_functions().size(),
                        getProperty_coordinates().size()));
        
        for (int i = 0; i < getProperty_moment_arm_functions().size(); ++i) {
            OPENSIM_THROW_IF_FRMOBJ(
                    get_moment_arm_functions(i).getArgumentSize() != 
                            getProperty_coordinates().size(), Exception,
                    fmt::format("Expected the number of arguments in "
                                "'moment_arm_functions[{}]' ({}) to equal the "
                                "number of coordinates ({}).",
                            i, get_moment_arm_functions(i).getArgumentSize(),
                            getProperty_coordinates().size()));
        }
    }
    
    if (getProperty_lengthening_speed_function().empty()) {
        _computeLengtheningSpeed = true;
    } else {
        OPENSIM_THROW_IF_FRMOBJ(
                getLengtheningSpeedFunction().getArgumentSize() !=
                2*getProperty_coordinates().size(), Exception,
                fmt::format("Expected the number of arguments in "
                            "'speed_function' ({}) to equal to twice the "
                            "number of coordinates ({}).",
                        getLengtheningSpeedFunction().getArgumentSize(),
                        getProperty_coordinates().size()));
    }
    
    // Populate the coordinate index map. In case "finalizeFromProperties()" is
    // called multiple times, clear the map first.
    _coordinateIndices.clear();
    for (int i = 0; i < getProperty_coordinates().size(); ++i) {
        _coordinateIndices[get_coordinates(i)] = i;
    }
}
void FunctionBasedPath::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    
    // Check that the coordinates are in the model. If so, grab references
    // pointers to them. In case "connectToModel()" is called multiple times,
    // clear any existing references first.
    _coordinates.clear();
    for (int i = 0; i < getProperty_coordinates().size(); ++i) {
        const auto& coordName = get_coordinates(i);
        OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<Coordinate>(coordName),
                Exception,
                fmt::format("Coordinate '{}' does not exist in the model.",
                            coordName));
        
        _coordinates.emplace_back(
                &model.getComponent<const Coordinate>(coordName));
    }
}
void FunctionBasedPath::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    addCacheVariable<double>(LENGTH_NAME, 0.0, SimTK::Stage::Position);
    addCacheVariable<SimTK::Vector>(MOMENT_ARMS_NAME, 
            SimTK::Vector(getProperty_coordinates().size(), 0.0),
            SimTK::Stage::Velocity);
    addCacheVariable<double>(LENGTHENING_SPEED_NAME, 0.0,
                SimTK::Stage::Velocity);
}
