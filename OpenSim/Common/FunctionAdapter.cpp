/* -------------------------------------------------------------------------- *
 *                       OpenSim:  FunctionAdapter.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "FunctionAdapter.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
/**
 * Constructor from an OpenSim::Function.
 */
FunctionAdapter::FunctionAdapter(const OpenSim::Function& aFunction) : _function(aFunction)
{
}

//=============================================================================
// SimTK::Function METHODS
//=============================================================================
double FunctionAdapter::calcValue(const Vector& x) const {
    return _function.calcValue(x);
}
double FunctionAdapter::calcDerivative(const std::vector<int>& derivComponents, const Vector& x) const {
    return _function.calcDerivative(derivComponents, x);
}

double FunctionAdapter::calcDerivative(const SimTK::Array_<int>& derivComponents, const SimTK::Vector& x) const{
    std::vector<int> dcs(derivComponents.begin(), derivComponents.end());
    return _function.calcDerivative(dcs, x);
}

int FunctionAdapter::getArgumentSize() const {
    return _function.getArgumentSize();
}
int FunctionAdapter::getMaxDerivativeOrder() const {
    return _function.getMaxDerivativeOrder();
}

