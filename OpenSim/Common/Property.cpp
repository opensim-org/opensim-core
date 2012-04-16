// AbstractProperty.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2012, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//============================================================================
// INCLUDES
//============================================================================
#include "AbstractProperty.h"
#include "Property.h"
#include "Object.h"

#include <string>
#include <limits>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//=============================================================================
// TYPE HELPER SPECIALIZATIONS
//=============================================================================

// Doubles compare equal if they are close enough, and they compare equal
// if they are both NaN. Vec3, Vector, and Array<double> properties must all
// be implemented in terms of this method.
template <>
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


template <>
bool Property<SimTK::Vec3>::TypeHelper::
isEqual(const SimTK::Vec3& a, const SimTK::Vec3& b) {
    for (int i=0; i < 3; ++i)
        if (!Property<double>::TypeHelper::isEqual(a[i],b[i]))
            return false;
    return true;
}


// SimTK::Vector
template <>
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
template <>
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

template SimpleProperty<int>;
template SimpleProperty<double>;
template SimpleProperty<std::string>;
template ObjectProperty<Object>;