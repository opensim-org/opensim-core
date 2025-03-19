/* -------------------------------------------------------------------------- *
 *                 OpenSim:  ExpressionBasedPathForce.cpp                    *
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

#include "ExpressionBasedPathForce.h"

#include <lepton/Parser.h>
#include <lepton/ParsedExpression.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
ExpressionBasedPathForce::ExpressionBasedPathForce() {
    constructProperties();
}

ExpressionBasedPathForce::ExpressionBasedPathForce(const std::string& name, 
        double restLength, const std::string& expression, bool clampStretch) {
    constructProperties();
    setName(name);
    set_resting_length(restLength);
    set_expression(expression);
    set_clamp_stretch(clampStretch);
}

void ExpressionBasedPathForce::constructProperties() {
    constructProperty_path(GeometryPath());
    constructProperty_resting_length(0);
    constructProperty_expression("0.0");
    constructProperty_clamp_stretch(true);

    // override default GeometryPath color (at time of writing, grey) with
    // green for backwards-compatibility
    upd_path().upd_Appearance().set_color({0, 1, 0});
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void ExpressionBasedPathForce::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    OPENSIM_THROW_IF_FRMOBJ(
        (SimTK::isNaN(get_resting_length()) || get_resting_length() < 0),
        Exception,
        "Expected the 'resting_length' property to be greater than zero, but "
        "received {}.", get_resting_length());

    // Remove whitespace from the expression.
    std::string& expression = upd_expression();
    expression.erase(remove_if(expression.begin(), expression.end(), ::isspace), 
            expression.end());
    
    // Parse the expression and create the program.
    _tensionProg = Lepton::Parser::parse(expression).optimize().createProgram();
}

void ExpressionBasedPathForce::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    this->_tensionCV = addCacheVariable("tension", 0.0, SimTK::Stage::Velocity);
}

//==============================================================================
// METHODS
//==============================================================================
double ExpressionBasedPathForce::getLength(const SimTK::State& s) const {
    return getPath().getLength(s);
}

double ExpressionBasedPathForce::getStretch(const SimTK::State& s) const {
    const double& length = getLength(s);
    const double& restingLength = get_resting_length();
    return length < restingLength && getClampStretch() ? 0.0 
                                                       : length - restingLength;
}

double ExpressionBasedPathForce::getLengtheningSpeed(
        const SimTK::State& s) const {
    return getPath().getLengtheningSpeed(s);
}

double ExpressionBasedPathForce::getTension(const SimTK::State& s) const {
    return getCacheVariableValue(s, _tensionCV);
}

//==============================================================================
// SCALING
//==============================================================================
void ExpressionBasedPathForce::extendPostScale(const SimTK::State& s, 
        const ScaleSet& scaleSet) {
    Super::extendPostScale(s, scaleSet);

    AbstractGeometryPath& path = updPath();
    if (path.getPreScaleLength(s) > 0.0) {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        upd_resting_length() *= scaleFactor;

        // Clear the pre-scale length that was stored in the 
        // AbstractGeometryPath.
        path.setPreScaleLength(s, 0.0);
    }
}

//=============================================================================
// COMPUTATION
//=============================================================================
double ExpressionBasedPathForce::computeMomentArm(const SimTK::State& s,
        const Coordinate& coordinate) const {
    return getPath().computeMomentArm(s, coordinate);
}

OpenSim::Array<std::string> ExpressionBasedPathForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(fmt::format("{}_tension", getName()));
    return labels;
}

OpenSim::Array<double> ExpressionBasedPathForce::getRecordValues(
        const SimTK::State& state) const {
    OpenSim::Array<double> values(1);
    values.append(getTension(state));
    return values;
}

//=============================================================================
// FORCE PRODUCER INTERFACE
//=============================================================================
void ExpressionBasedPathForce::implProduceForces(const SimTK::State& s, 
        ForceConsumer& forceConsumer) const {

    std::map<std::string, double> vars;
    vars["s"] = getStretch(s);
    vars["ldot"] = getLengtheningSpeed(s);

    double tension = _tensionProg.evaluate(vars);
    setCacheVariableValue(s, _tensionCV, tension);

    getPath().produceForces(s, tension, forceConsumer);
}
