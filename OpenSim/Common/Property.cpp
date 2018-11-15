/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Property.cpp                           *
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

//============================================================================
// INCLUDES
//============================================================================
#include "Property.h"
#include "Object.h"
#include <string>


using namespace OpenSim;
using namespace SimTK;
using namespace std;

//=============================================================================
// TYPE HELPER SPECIALIZATIONS
//=============================================================================

// Doubles compare equal if they are close enough, and they compare equal
// if they are both NaN. Vec3, Vector, and Array<double> properties must all
// be implemented in terms of this method.
bool Property<double>::TypeHelper::
isEqual(double a, double b) {
    if (a == b)
        return true; // catch exact match and Infinities

    if (SimTK::isNaN(a) && SimTK::isNaN(b))
        return true; // we define NaN==NaN to be true here

    // Floating point need only match to a tolerance.
    // TODO: why is this the right number??
    return std::abs(a - b) <= 1e-7;
}


bool Property<SimTK::Vec3>::TypeHelper::
isEqual(const SimTK::Vec3& a, const SimTK::Vec3& b) {
    for (int i=0; i < 3; ++i)
        if (!Property<double>::TypeHelper::isEqual(a[i],b[i]))
            return false;
    return true;
}

bool Property<SimTK::Vec6>::TypeHelper::
isEqual(const SimTK::Vec6& a, const SimTK::Vec6& b) {
    for (int i = 0; i < 6; ++i)
        if (!Property<double>::TypeHelper::isEqual(a[i], b[i]))
            return false;
    return true;
}

// SimTK::Vector
bool Property<SimTK::Vector>::TypeHelper::
isEqual(const SimTK::Vector& a, const SimTK::Vector& b) {
    if (a.size() != b.size())
        return false;
    for (int i=0; i < a.size(); ++i)
        if (!Property<double>::TypeHelper::isEqual(a[i],b[i]))
            return false;
    return true;
}

// SimTK::Transform
bool Property<SimTK::Transform>::TypeHelper::
isEqual(const SimTK::Transform& a, const SimTK::Transform& b) {
    // Check position vectors for equality, elementwise to a tolerance.
    if (!Property<SimTK::Vec3>::TypeHelper::isEqual(a.p(), b.p()))
        return false;
    // Check rotation matrix to equality within angle tolerance.
    // TODO: is this angle reasonable?
    if (!a.R().isSameRotationToWithinAngle(b.R(), 1e-7))
        return false;

    return true;
}

//=============================================================================
// PROPERTY 
//=============================================================================

// Try a few instantiations so we catch bugs now.
namespace OpenSim {
template class SimpleProperty<int>;
template class SimpleProperty<double>;
template class SimpleProperty<std::string>;
template class ObjectProperty<Object>;
}
