/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PropertyTable.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
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

#include "PropertyTable.h"

#include "Assertion.h"

using namespace OpenSim;

void PropertyTable::clear()
{
    _properties.clear();
    _namelookup.clear();
}

bool PropertyTable::equals(const PropertyTable& other) const
{
    return _properties == other._properties;
}

int PropertyTable::adoptProperty(AbstractProperty* prop)
{
    OPENSIM_ASSERT(prop != nullptr);
    OPENSIM_ASSERT(!prop->getName().empty());  // Unnamed property should have had its Object class name used as its name.

    auto idx = static_cast<int>(_properties.size());
    const bool inserted = _namelookup.emplace(prop->getName(), idx).second;

    if (!inserted) {
        throw OpenSim::Exception("PropertyTable::adoptProperty(): Property " + prop->getName() + " already in table.");
    }

    _properties.emplace_back(prop);
    return idx;
}

const AbstractProperty* PropertyTable::getPropertyPtr(const std::string& name) const
{
    const auto it = _namelookup.find(name);
    return it != _namelookup.end() ? _properties[it->second].get() : nullptr;
}

AbstractProperty* PropertyTable::updPropertyPtr(const std::string& name)
{
    const auto it = _namelookup.find(name);
    return it != _namelookup.end() ? _properties[it->second].get() : nullptr;
}

const AbstractProperty& PropertyTable::getAbstractPropertyByIndex(int index) const
{
    if (index == SimTK::InvalidIndex) {
        throw OpenSim::Exception
           ("PropertyTable::getAbstractPropertyByIndex(): uninitialized "
            "property index -- did you forget a constructProperty() call?");
    }
    if (!(0 <= index && index < getNumProperties())) {
        throw OpenSim::Exception
           ("PropertyTable::getAbstractPropertyByIndex(): index "
            + SimTK::String(index) + " out of range ("
            + SimTK::String(getNumProperties()) + " properties in table).");
    }
    return *_properties[index];
}

AbstractProperty& PropertyTable::updAbstractPropertyByIndex(int index) {
    if (index == SimTK::InvalidIndex) {
        throw OpenSim::Exception
           ("PropertyTable::updAbstractPropertyByIndex(): uninitialized "
            "property index -- did you forget a constructProperty() call?");
    }
    if (!(0 <= index && index < getNumProperties())) {
        throw OpenSim::Exception
           ("PropertyTable::updAbstractPropertyByIndex(): index "
            + SimTK::String(index) + " out of range ("
            + SimTK::String(getNumProperties()) + " properties in table).");
    }
    return *_properties[index];
}

const AbstractProperty& PropertyTable::getAbstractPropertyByName(const std::string& name) const {
    if (const AbstractProperty* p = getPropertyPtr(name)) {
        return *p;
    }
    throw OpenSim::Exception("PropertyTable::getAbstractPropertyByName(): Property " + name + " not found.");
}

AbstractProperty& PropertyTable::updAbstractPropertyByName(const std::string& name) {
    if (AbstractProperty* p = updPropertyPtr(name)) {
        return *p;
    }
    throw OpenSim::Exception("PropertyTable::updAbstractPropertyByName(): Property " + name + " not found.");
}

int PropertyTable::findPropertyIndex(const std::string& name) const
{
    const auto it = _namelookup.find(name);
    return it != _namelookup.end() ? it->second : -1;
}
