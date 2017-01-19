/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PropertyTable.cpp                         *
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
#include "PropertyTable.h"


using namespace OpenSim;
using namespace SimTK;
using namespace std;

//_____________________________________________________________________________
// Default constructor is inline

//_____________________________________________________________________________
// Copy constructor has to clone the source properties.
PropertyTable::PropertyTable(const PropertyTable& source)
{
    replaceProperties(source.properties);
}

//_____________________________________________________________________________
// Destructor must deep-delete the properties since they are owned by the
// PropertyTable.
PropertyTable::~PropertyTable()
{
    deleteProperties();
}

//_____________________________________________________________________________
// Copy assignment has to clone the source properties.
PropertyTable& PropertyTable::operator=(const PropertyTable& source)
{
    if (&source != this)
        replaceProperties(source.properties);
    return *this;
}

//_____________________________________________________________________________
// Reinitialize the table to be completely empty.
void PropertyTable::clear() {
    deleteProperties();
}

//_____________________________________________________________________________
// Equality operator compares each property with the one at the same position.
bool PropertyTable::equals(const PropertyTable& other) const {
    if (getNumProperties() != other.getNumProperties())
        return false;
    for (int i=0; i < getNumProperties(); ++i) {
        if (!(getAbstractPropertyByIndex(i) == other.getAbstractPropertyByIndex(i)))
            return false;
    }
    return true;
}

int PropertyTable::adoptProperty(AbstractProperty* prop)
{
    assert(prop);
    const int          nxtIndex = properties.size();
    const std::string& name     = prop->getName();

    // Unnamed property should have had its Object class name used as its name.
    assert(!name.empty());

    if (hasProperty(name))
        throw OpenSim::Exception
            ("PropertyTable::adoptProperty(): Property " 
            + name + " already in table.");

    propertyIndex[name] = nxtIndex;
    properties.push_back(prop);
    return nxtIndex;
}

const AbstractProperty& PropertyTable::
getAbstractPropertyByIndex(int index) const {
    if (index == SimTK::InvalidIndex)
        throw OpenSim::Exception
           ("PropertyTable::getAbstractPropertyByIndex(): uninitialized " 
            "property index -- did you forget a constructProperty() call?");
    if (!(0 <= index && index < getNumProperties()))
        throw OpenSim::Exception
           ("PropertyTable::getAbstractPropertyByIndex(): index " 
            + String(index) + " out of range (" 
            + String(getNumProperties()) + " properties in table).");        
    return *properties[index]; 
}

AbstractProperty& PropertyTable::
updAbstractPropertyByIndex(int index) {
    if (index == SimTK::InvalidIndex)
        throw OpenSim::Exception
           ("PropertyTable::updAbstractPropertyByIndex(): uninitialized " 
            "property index -- did you forget a constructProperty() call?");
    if (!(0 <= index && index < getNumProperties()))
        throw OpenSim::Exception
           ("PropertyTable::updAbstractPropertyByIndex(): index " 
            + String(index) + " out of range (" 
            + String(getNumProperties()) + " properties in table).");        
    return *properties[index]; 
}

const AbstractProperty& PropertyTable::
getAbstractPropertyByName(const std::string& name) const {
    const AbstractProperty* p = getPropertyPtr(name);
    if (p == NULL) throw OpenSim::Exception
        ("PropertyTable::getAbstractPropertyByName(): Property " 
        + name + " not found.");
    return *p;
}

AbstractProperty& PropertyTable::
updAbstractPropertyByName(const std::string& name) {
    AbstractProperty* p = updPropertyPtr(name);
    if (p == NULL) throw OpenSim::Exception
        ("PropertyTable::updAbstractPropertyByName(): Property " 
        + name + " not found.");
    return *p;
}

// Look up the property by name in the map to find its index
// in the property array and return that. If the name isn't there, return -1.
// This method is reused in the implementation of any method that
// takes a property by name.
int PropertyTable::findPropertyIndex(const std::string& name) const {
    const std::map<std::string, int>::const_iterator 
        it = propertyIndex.find(name);
    return it == propertyIndex.end() ? -1 : it->second;
}

// Private method to replace the existing properties with a deep copy of 
// the source, and update the index map to match.
void PropertyTable::replaceProperties
   (const SimTK::Array_<AbstractProperty*>& source) {
    deleteProperties();
    for (unsigned i=0; i < source.size(); ++i) {
        properties.push_back(source[i]->clone());
        propertyIndex[source[i]->getName()] = i;
    }
}

// Private method to delete all the properties and clear the index.
void PropertyTable::deleteProperties() {
    for (unsigned i=0; i < properties.size(); ++i)
        delete properties[i];
    properties.clear();
    propertyIndex.clear();
}

