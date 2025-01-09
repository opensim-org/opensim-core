/* -------------------------------------------------------------------------- *
 *                     OpenSim:  CoordinateReference.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "CoordinateReference.h"

using namespace std;
using namespace SimTK;

namespace OpenSim {

CoordinateReference::CoordinateReference()
        : Reference_<double>(),
    _coordinateValueFunction(_coordinateValueFunctionProp.getValueObjPtrRef()),
    _defaultWeight(_defaultWeightProp.getValueDbl())
{
    setAuthors("Ajay Seth");
    _names.resize(getNumRefs());
    _names[0] = getName();
}

//______________________________________________________________________________
/**
 * An implementation of the CoordinateReference
 *
 * @param name of the reference to be found in the model and
 * @param referenceFunction is its function returning its value
 */
CoordinateReference::CoordinateReference(const std::string name, 
    const Function& referenceFunction)
        : Reference_<double>(name),
    _coordinateValueFunction(_coordinateValueFunctionProp.getValueObjPtrRef()),
    _defaultWeight(_defaultWeightProp.getValueDbl())
{
    setAuthors("Ajay Seth");
    setValueFunction(referenceFunction);
    _names.resize(getNumRefs());
    _names[0] = getName();
}

// Copy constructor
CoordinateReference::CoordinateReference(const CoordinateReference& source)
:   Super(source), 
    _coordinateValueFunction(_coordinateValueFunctionProp.getValueObjPtrRef()),
    _defaultWeight(_defaultWeightProp.getValueDbl()) 
{
    copyData(source);
}


CoordinateReference& CoordinateReference::operator=(const CoordinateReference& source)
{
    if (&source != this) {
        Super::operator=(source);
        copyData(source);
    }
    return *this;
}

void CoordinateReference::copyData(const CoordinateReference& source)
{
    _coordinateValueFunction = source._coordinateValueFunction->clone();
    _defaultWeight = source._defaultWeight;
}


/** get the names of the referettes */
const SimTK::Array_<std::string>& CoordinateReference::getNames() const
{
    return _names;
}


/** get the values of the CoordinateReference */
void CoordinateReference::getValuesAtTime(double time, SimTK::Array_<double> &values) const
{
    SimTK::Vector t(1, time);
    values.resize(getNumRefs());
    values[0] = _coordinateValueFunction->calcValue(t);
}


/** get the weighting (importance) of meeting this Reference */
void CoordinateReference::getWeights(const SimTK::State &s, SimTK::Array_<double> &weights) const
{
    weights.resize(getNumRefs());
    weights[0] = _defaultWeight;
}


/** get the value of the CoordinateReference */
double CoordinateReference::getValue(const SimTK::State &s) const
{
    SimTK::Vector t(1, s.getTime());
    return _coordinateValueFunction->calcValue(t);
}

/** get the speed value of the CoordinateReference */
double CoordinateReference::getSpeedValue(const SimTK::State &s) const
{
    SimTK::Vector t(1, s.getTime());
    std::vector<int> order(1, 0);
    return _coordinateValueFunction->calcDerivative(order, t);
}

/** get the acceleration value of the CoordinateReference */
double CoordinateReference::getAccelerationValue(const SimTK::State &s) const
{
    SimTK::Vector t(1, s.getTime());
    std::vector<int> order(2, 0);
    return _coordinateValueFunction->calcDerivative(order, t);
}

/** get the weight of the CoordinateReference */
double CoordinateReference::getWeight(const SimTK::State &s) const
{
    return Reference_<double>::getWeights(s)[0];
}

/** set the weight of the CoordinateReference */
void CoordinateReference::setWeight(double weight)
{
    _defaultWeight = weight;
}


void CoordinateReference::setValueFunction(const OpenSim::Function& function)
{
    delete _coordinateValueFunction;
    _coordinateValueFunction = function.clone();
}

} // end of namespace OpenSim
