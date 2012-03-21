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
#include "Object.h"
#include "Function.h"

#include <string>

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractProperty::AbstractProperty()
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
AbstractProperty::AbstractProperty(const std::string& name, 
                                   const std::string& typeAsString, 
                                   const std::string& comment)
{
    setNull();
	_name = name;
	_typeAsString = typeAsString;
	_comment = comment;
}


//_____________________________________________________________________________
/**
 * Set member variables to their null values.
 */
void AbstractProperty::setNull()
{
	_name = "unknown";
	_typeAsString = "none";
    _comment = "";
	_useDefault = false;
	_matchName = false;
	_minArraySize = 0;
	_maxArraySize = INT_MAX;
}

//=============================================================================
// TYPE HELPER SPECIALIZATIONS
//=============================================================================
string AbstractProperty::TypeHelper<bool>::
formatForDisplay(bool v) {
    return v ? "true" : "false";
}

string AbstractProperty::TypeHelper<int>::
formatForDisplay(int v) {
    char intString[32];
	sprintf(intString, "%d", v);
	return intString;
}

// Strings are compared exactly here but could conceivably by made fancier
// so make sure that Array<string>::isEqual() is defined in terms of this one.
bool AbstractProperty::TypeHelper<string>::
isEqual(const string& a, const std::string& b) {
    return a == b;
}
string AbstractProperty::TypeHelper<string>::
formatForDisplay(const string& s) {
    return s;
}

// Doubles compare equal if they are close enough, and they compare equal
// if they are both NaN. Vec3, Vector, and Array<double> properties must all
// be implemented in terms of this method.
bool AbstractProperty::TypeHelper<double>::
isEqual(double a, double b) {
    if (a == b)
        return true; // catch exact match and Infinities

    if (SimTK::isNaN(a) && SimTK::isNaN(b))
        return true; // we define NaN==NaN to be true here

    // Floating point need only match to a tolerance.
    // TODO: why is this the right number??
    return std::abs(a - b) <= 1e-7;
}

// All formatForDisplay() methods involving doubles must be implemented using
// this method.
string AbstractProperty::TypeHelper<double>::
formatForDisplay(double v) {
    if (SimTK::isFinite(v)) {
	    char dbl[256];
        sprintf(dbl, "%g", v);
        return dbl;
    }

    if (SimTK::isNaN(v)) return "NaN";
	if (v ==  SimTK::Infinity) return "infinity";
	if (v == -SimTK::Infinity) return "-infinity";
	return "UnrecognizedNonFinite???";
}

bool AbstractProperty::TypeHelper<SimTK::Vec3>::
isEqual(const SimTK::Vec3& a, const SimTK::Vec3& b) {
    for (int i=0; i < 3; ++i)
        if (!TypeHelper<double>::isEqual(a[i],b[i]))
            return false;
    return true;
}

string AbstractProperty::TypeHelper<SimTK::Vec3>::
formatForDisplay(const SimTK::Vec3& v) {
	string str = "(";
	for(int i=0; i < 3; ++i) {
        if (i>0) str += " ";
        str += TypeHelper<double>::formatForDisplay(v[i]);
    }
	str += ")";
	return str;
}

// SimTK::Vector
bool AbstractProperty::TypeHelper<SimTK::Vector>::
isEqual(const SimTK::Vector& a, const SimTK::Vector& b) {
    if (a.size() != b.size())
        return false;
    for (int i=0; i < a.size(); ++i)
        if (!TypeHelper<double>::isEqual(a[i],b[i]))
            return false;
    return true;
}
string AbstractProperty::TypeHelper<SimTK::Vector>::
formatForDisplay(const SimTK::Vector& v) {
	string str = "(";
	for(int i=0; i < v.size(); ++i) {
        if (i>0) str += " ";
        str += TypeHelper<double>::formatForDisplay(v[i]);
    }
	str += ")";
	return str;
}

// SimTK::Transform
bool AbstractProperty::TypeHelper<SimTK::Transform>::
isEqual(const SimTK::Transform& a, const SimTK::Transform& b) {
    // Check position vectors for equality, elementwise to a tolerance.
    if (!TypeHelper<SimTK::Vec3>::isEqual(a.p(), b.p()))
        return false;
    // Check rotation matrix to equality within angle tolerance.
    // TODO: is this angle reasonable?
    if (!a.R().isSameRotationToWithinAngle(b.R(), 1e-7))
        return false;

    return true;
}
string AbstractProperty::TypeHelper<SimTK::Transform>::
formatForDisplay(const SimTK::Transform& X) {
    SimTK::Vector rotTrans(6);
    rotTrans(0,3) = SimTK::Vector(X.R().convertRotationToBodyFixedXYZ());
    rotTrans(3,3) = SimTK::Vector(X.p()); // translations
    return TypeHelper<SimTK::Vector>::formatForDisplay(rotTrans);
}

// OpenSim::Array<bool>
bool AbstractProperty::TypeHelper< OpenSim::Array<bool> >::
isEqual(const OpenSim::Array<bool>& a, const OpenSim::Array<bool>& b) {
    if (a.getSize() != b.getSize())
        return false;
    for (int i=0; i < a.getSize(); ++i)
        if (a[i] != b[i])
            return false;
    return true;
}
string AbstractProperty::TypeHelper< OpenSim::Array<bool> >::
formatForDisplay(const OpenSim::Array<bool>& a) {
	string str = "(";
	for(int i=0; i < a.getSize(); ++i) {
        if (i>0) str += " ";
        str += TypeHelper<bool>::formatForDisplay(a[i]);
    }
	str += ")";
	return str;
}

bool AbstractProperty::TypeHelper< OpenSim::Array<int> >::
isEqual(const OpenSim::Array<int>& a, const OpenSim::Array<int>& b) {
    if (a.getSize() != b.getSize())
        return false;
    for (int i=0; i < a.getSize(); ++i)
        if (a[i] != b[i])
            return false;
    return true;
}
string AbstractProperty::TypeHelper< OpenSim::Array<int> >::
formatForDisplay(const OpenSim::Array<int>& a) {
	string str = "(";
	for(int i=0; i < a.getSize(); ++i) {
        if (i>0) str += " ";
        str += TypeHelper<int>::formatForDisplay(a[i]);
    }
	str += ")";
	return str;
}

bool AbstractProperty::TypeHelper< OpenSim::Array<double> >::
isEqual(const OpenSim::Array<double>& a, const OpenSim::Array<double>& b) {
    if (a.getSize() != b.getSize())
        return false;
    for (int i=0; i < a.getSize(); ++i)
        if (!TypeHelper<double>::isEqual(a[i],b[i]))
            return false;
    return true;
}
string AbstractProperty::TypeHelper< OpenSim::Array<double> >::
formatForDisplay(const OpenSim::Array<double>& a) {
	string str = "(";
	for(int i=0; i < a.getSize(); ++i) {
        if (i>0) str += " ";
        str += TypeHelper<double>::formatForDisplay(a[i]);
    }
	str += ")";
	return str;
}

bool AbstractProperty::TypeHelper< OpenSim::Array<string> >::
isEqual(const OpenSim::Array<string>& a, const OpenSim::Array<string>& b) {
    if (a.getSize() != b.getSize())
        return false;
    for (int i=0; i < a.getSize(); ++i)
        if (!TypeHelper<string>::isEqual(a[i],b[i]))
            return false;
    return true;
}
string AbstractProperty::TypeHelper< OpenSim::Array<string> >::
formatForDisplay(const OpenSim::Array<string>& a) {
	string str = "(";
	for(int i=0; i < a.getSize(); ++i) {
        if (i>0) str += " ";
        str += TypeHelper<string>::formatForDisplay(a[i]);
    }
	str += ")";
	return str;
}
