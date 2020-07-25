/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PropertySet.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//============================================================================
// INCLUDES
//============================================================================
#include "PropertySet.h"
#include "PropertyInt.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertySet::PropertySet()
{
    _array.setMemoryOwner(false);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSet Set of properties to be copied.
 */
PropertySet::PropertySet(const PropertySet &aSet)
{
    _array = aSet._array;
    _array.setMemoryOwner(false);

}

PropertySet& PropertySet::operator=(const PropertySet& aSet)
{
   if (this != &aSet) {
       _array = aSet._array;
       _array.setMemoryOwner(false);
   }
   return *this;
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// OUTPUT (<<)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Implementation of the output operator.
 *
 * @param aOut Output stream.
 * @param aArray Array to be output.
 * @return Reference to the output stream.
ostream&
operator<<(ostream &aOut,const PropertySet &aSet)
{
    aOut << "\nProperty Set:\n";

    int i;
    for(i=0;i<aSet.getSize();i++)  {
        aOut << aSet.get(i) << "\n";
    }

    return(aOut);
}
 */


//=============================================================================
// EMPTY?
//=============================================================================
//_____________________________________________________________________________
/**
 * Determine whether or not this property set is empty.
 *
 * @returns True if empty; false otherwise.
 */
bool PropertySet::
isEmpty() const
{
    if(_array.getSize()<=0) return(true);
    return(false);
}


//=============================================================================
// SIZE
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the number of properties in the set, or, equivalently, the size of
 * the property set.
 *
 * @returns Size or number of properties in this set.
 */
int PropertySet::
getSize() const
{
    return(_array.getSize());
}


//=============================================================================
// GET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get a reference to a property by index.
 *
 * @param aIndex Index of the property to get.
 * @throws Exception if the index is out of bounds.
 */
Property_Deprecated* PropertySet::
get(int aIndex)
{
    // NO SUCH PROPERTY - THROW EXCEPTION
    if((aIndex<0)||(aIndex>=_array.getSize())) {
        string msg = "PropertySet.get(int): Index is out of bounds.";
        throw Exception(msg,__FILE__,__LINE__);
    }

    return(_array[aIndex]);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to a property by index.
 *
 * @param aIndex Index of the property to get.
 * @throws Exception if the index is out of bounds.
 */
const Property_Deprecated* PropertySet::
get(int aIndex) const
{
    // NO SUCH PROPERTY - THROW EXCEPTION
    if((aIndex<0)||(aIndex>=_array.getSize())) {
        string msg = "PropertySet.get(int): Index is out of bounds.";
        throw Exception(msg,__FILE__,__LINE__);
    }

    return(_array[aIndex]);
}
//_____________________________________________________________________________
/**
 * Get the first property in the set that has a specified name.
 *
 * @param aName Name of the property to get.
 * @throws Exception if there is no such property.
 */
Property_Deprecated* PropertySet::
get(const string &aName)
{
    int i;
    PropertyInt prop(aName,0);
    for(i=0;i<_array.getSize();i++) {
        if((*_array[i]) == prop) return(_array[i]);
    }
        
    // NO SUCH PROPERTY - THROW EXCEPTION
    string msg = "PropertySet.get: No property named " + aName;
    throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get the first property in the set that has a specified name.
 *
 * @param aName Name of the property to get.
 * @throws Exception if there is no such property.
 */
const Property_Deprecated* PropertySet::
get(const string &aName) const
{
    int i;
    PropertyInt prop(aName,0);
    for(i=0;i<_array.getSize();i++) {
        if((*_array[i]) == prop) return(_array[i]);
    }
        
    // NO SUCH PROPERTY - THROW EXCEPTION
    string msg = "PropertySet.get: No property named " + aName;
    throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * If the set contains a property with the specified name, return it.
 *
 * @param aName Name of the property to get.
 * @return Pointer to the property.
 */
const Property_Deprecated* PropertySet::
contains(const string& name) const
{
    int i = _array.getIndex(name);
    //PropertyInt prop(aName,0);
    //for(i=0;i<_array.getSize();i++) {
    //    if((*_array[i]) == prop) return(_array[i]);
    //}

    if (i >= 0) return _array.get(i);

    return nullptr;
}
//_____________________________________________________________________________
/**
 * If the set contains a property with the specified name, return it.
 *
 * @param aName Name of the property to get.
 * @return Pointer to the property.
 */
Property_Deprecated* PropertySet::
contains(const string& name)
{
    int i  = _array.getIndex(name);
    //PropertyInt prop(aName,0);
    //for(i=0;i<_array.getSize();i++) {
    //    if((*_array[i]) == prop) return(_array[i]);
    //}

    if (i >= 0) return _array.get(i);

    return nullptr;
}


//=============================================================================
// APPEND
//=============================================================================
//_____________________________________________________________________________
/**
 * Append a property to the set.
 *
 * @param aProperty Property_Deprecated to be appended.  Note that a copy is NOT made.
 */
void PropertySet::
append(Property_Deprecated *aProperty)
{
    _array.append(aProperty);
}

//_____________________________________________________________________________
/**
 * Append a property to the set, and put it in the specified group.
 *
 * @param aProperty Property_Deprecated to be appended.  Note that a copy is NOT made.
 */
void PropertySet::
append(Property_Deprecated *aProperty, const string& aName)
{
    _array.append(aProperty);
}

//=============================================================================
// REMOVE
//=============================================================================
//_____________________________________________________________________________
/**
 * Remove the first property in the set that has a specified name.
 *
 * @param aName Name of the property to remove.
 * @throws Exception if there is no such property.
 */
void PropertySet::
remove(const string &aName)
{
    int i;
    PropertyInt prop(aName,0);
    for(i=0;i<_array.getSize();i++) {
        if((*_array[i]) == prop) {
            _array.remove(i);
            return;
        }
    }
        
    // NO SUCH PROPERTY - THROW EXCEPTION
    string msg = "PropertySet.get: No property named " + aName;
    throw Exception(msg,__FILE__,__LINE__);
}


//=============================================================================
// CLEAR
//=============================================================================
//_____________________________________________________________________________
/**
 * Clear this property set of all properties and groups.
 */
void PropertySet::
clear()
{
    // Remove all the properties.
    _array.setSize(0);
    _array.trim();
}
