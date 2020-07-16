/* -------------------------------------------------------------------------- *
 *              OpenSim:  ExpressionBasedCoordinateForce.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Nabeel Allana                                                   *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "ExpressionBasedCoordinateForce.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <lepton/Parser.h>
#include <lepton/ParsedExpression.h>

using namespace OpenSim;
using namespace std;


//_____________________________________________________________________________
//Default constructor.
ExpressionBasedCoordinateForce::ExpressionBasedCoordinateForce()
{
    setNull();
    constructProperties();
}
//_____________________________________________________________________________
// Convenience constructor for API users.
ExpressionBasedCoordinateForce::ExpressionBasedCoordinateForce(
                const string& coordinate, const string& expression)
{
    setNull();
    constructProperties();

    // Set properties to the passed-in values.
    setCoordinateName(coordinate);
    setExpression(expression);
}

// Set the expression for the force function and create it's lepton program 
void ExpressionBasedCoordinateForce::setExpression(const string& expression) 
{
    set_expression(expression);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this force to their null values.
 */
void ExpressionBasedCoordinateForce::setNull()
{
    setAuthors("Nabeel Allana"); 
}

//_____________________________________________________________________________
/**
 * Construct properties and initialize to their default values.
 */
void ExpressionBasedCoordinateForce::constructProperties()
{
    constructProperty_coordinate("UNASSIGNED");
    std::string zero = "0.0";
    constructProperty_expression( zero );
}

//=============================================================================
// Connect this force element to the rest of the model.
//=============================================================================
void ExpressionBasedCoordinateForce::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    string errorMessage;
    const string& coordName = get_coordinate();

    string& expression = upd_expression();
    expression.erase(
            remove_if(expression.begin(), expression.end(), ::isspace), 
                      expression.end() );
    
    _forceProg = Lepton::Parser::parse(expression).optimize().createProgram();

    // Look up the coordinate
    if (!_model->updCoordinateSet().contains(coordName)) {
        errorMessage = "ExpressionBasedCoordinateForce: Invalid coordinate (" + coordName + ") specified in " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    _coord = &_model->updCoordinateSet().get(coordName);
    
    if(getName() == "")
        setName("expressionCoordForce_"+coordName);
}

//=============================================================================
// Create the underlying system component(s)
//=============================================================================
void ExpressionBasedCoordinateForce::
    extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);    // Base class first.
    this->_forceMagnitudeCV = addCacheVariable("force_magnitude", 0.0, SimTK::Stage::Velocity);
}

//=============================================================================
// Computing
//=============================================================================
// Compute and apply the force
void ExpressionBasedCoordinateForce::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    applyGeneralizedForce(s, *_coord, calcExpressionForce(s), generalizedForces);
}

// Compute the force
double ExpressionBasedCoordinateForce::calcExpressionForce(const SimTK::State& s ) const
{
    using namespace SimTK;
    double q = _coord->getValue(s);
    double qdot = _coord->getSpeedValue(s);
    std::map<std::string, double> forceVars;
    forceVars["q"] = q;
    forceVars["qdot"] = qdot;
    double forceMag = _forceProg.evaluate(forceVars);
    setCacheVariableValue(s, _forceMagnitudeCV, forceMag);
    return forceMag;
}

// get the force magnitude that has already been computed
const double& ExpressionBasedCoordinateForce::
    getForceMagnitude(const SimTK::State& s)
{
    return getCacheVariableValue(s, _forceMagnitudeCV);
}


//=============================================================================
// Reporting
//=============================================================================
// Provide names of the quantities (column labels) of the force value(s) 
// reported.
Array<std::string> ExpressionBasedCoordinateForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(getName());
    return labels;
}
// Provide the value(s) to be reported that correspond to the labels.
Array<double> ExpressionBasedCoordinateForce::getRecordValues(const SimTK::State& state) const {
    OpenSim::Array<double> values(0.0, 0, 1);
    values.append(calcExpressionForce(state));
    return values;
}
