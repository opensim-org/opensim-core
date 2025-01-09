#ifndef OPENSIM_PROPERTY_TABLE_H_
#define OPENSIM_PROPERTY_TABLE_H_

/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PropertyTable.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Cassidy Kelly, Michael A. Sherman, Adam Kewley                  *
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

#include <OpenSim/Common/osimCommonDLL.h>
#include <OpenSim/Common/Property.h>

#include <SimTKcommon/internal/ClonePtr.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace OpenSim {

/**
 * A property table is the container that an OpenSim Object uses to hold its
 * properties (each derived from base class AbstractProperty). It provides 
 * methods to add properties to the table, and preserves the order with which 
 * they are defined. Methods for retrieving properties by name or by ordinal
 * are provided. The table is the owner of the individual property objects and
 * those are deleted when the table is deleted.
 *
 * Duplicate property names are not allowed in the same property table. Some
 * properties are unnamed, however; that is only allowed when the property
 * holds a single object, and we use the object class name as though it were
 * the property's name (and they can still be looked up by ordinal also). That
 * means no Object can contain two unnamed properties holding the same type of
 * object since that would appear as a duplicate name.
 *
 * @author Cassidy Kelly, Michael Sherman, Adam Kewley
 * @see AbstractProperty, Property, Object
 */
class OSIMCOMMON_API PropertyTable {
public:
    /**
     * Delete all the properties currently in this table, restoring the table to
     * its default-constructed state.
     */
    void clear();

    /**
     * Compare this table to another one for equality. Two tables are defined
     * to be equal if they have the same number of properties, and each property
     * tests equal to the one with the same index.
     */
    bool equals(const PropertyTable& other) const;

#ifndef SWIG
    /**
     * See equals() for the meaning of this operator.
     */
    bool operator==(const PropertyTable& other) const { return equals(other); }
#endif

    /**
     * Add a new property to this table, taking over ownership of the
     * supplied heap-allocated property. Throws an exception if there is already
     * a property with the same name in the table. Returns an index (ordinal)
     * that can be used to retrieve this property quickly.
     */
    int adoptProperty(AbstractProperty* prop);

    /**
     * Return a const reference to a property of known type T from the
     * table by name. This will throw an exception if no property with this name
     * is present, or if the property is present but not of type T.
     */
    template <class T> const Property<T>& getProperty(const std::string& name) const
    {
        const AbstractProperty& prop = getAbstractPropertyByName(name);
        return Property<T>::getAs(prop);
    }

    /**
     * Return a const reference to a property of known type T from the table
     * by its index (numbered in order of addition to the table). This will
     * throw an exception if the index is out of range, or if the property is
     * present but not of type T.
     */
    template <class T> const Property<T>& getProperty(int index) const
    {
        const AbstractProperty& prop = getAbstractPropertyByIndex(index);
        return Property<T>::getAs(prop);
    }

    /**
     * Return a writable reference to a property of known type T from the
     * table by name. This will throw an exception if no property with this name
     * is present, or if the property is present but not of type T.
     */
    template <class T> Property<T>& updProperty(const std::string& name)
    {
        AbstractProperty& prop = updAbstractPropertyByName(name);
        return Property<T>::updAs(prop);
    }

    /**
     * Return a writable reference to a property of known type T from the
     * table by its index (numbered in order of addition to the table). This will
     * throw an exception if the index is out of range, or if the property is
     * present but not of type T.
     */
    template <class T> Property<T>& updProperty(int index)
    {
        AbstractProperty& prop = updAbstractPropertyByIndex(index);
        return Property<T>::updAs(prop);
    }

    /**
     * Return true if there is a property with the given name currently stored
     * in this table.
     */
    bool hasProperty(const std::string& name) const
    {
        return findPropertyIndex(name) >= 0;
    }

    /**
     * Look up a property by name and return a pointer providing const access
     * to the stored AbstractProperty object if present, otherwise null.
     */
    const AbstractProperty* getPropertyPtr(const std::string& name) const;

    /**
     * Look up a property by name and return a pointer providing writable
     * access to the stored AbstractProperty object if present, otherwise null.
     */
    AbstractProperty* updPropertyPtr(const std::string& name);

    /**
     * Return the number of properties currently in this table. If this returns n,
     * the properties are indexed 0 to n-1, in the order they were added to the table.
     */
    int getNumProperties() const { return (int)_properties.size(); }

    /**
     * Retrieve a property by its index, which must be in the range
     * 0..getNumProperties()-1. The property index is assigned in the order that
     * the properties were added to this table.
     */
    const AbstractProperty& getAbstractPropertyByIndex(int index) const;

    /**
     * Retrieve a writable reference to a property by its index, which must be
     * in the range 0..getNumProperties()-1. The property index is assigned in the
     * order that the properties were added to this table.
     */
    AbstractProperty& updAbstractPropertyByIndex(int index);

    /**
     * Retrieve a property by name if it exists, otherwise throw an exception. The
     * property is returned as an AbstractProperty; see getProperty<T>() if you
     * know the property type.
     */
    const AbstractProperty& getAbstractPropertyByName(const std::string& name) const;

    /**
     * Retrieve a writable reference to a property by name if it exists, otherwise
     * throw an exception. The property is returned as an AbstractProperty; see
     * updProperty<T>() if you know the property type.
     */
    AbstractProperty& updAbstractPropertyByName(const std::string& name);

    /**
     * Return the property's index if it is present, else -1.
     */
    int findPropertyIndex(const std::string& name) const;

private:

    // internal class that defines how the property is stored in this table
    class PropertyHolder final {
    public:
        explicit PropertyHolder(AbstractProperty* p) : _ptr{p} {}

        friend bool operator==(const PropertyHolder& lhs, const PropertyHolder& rhs)
        {
            return *lhs._ptr == *rhs._ptr;
        }

        const AbstractProperty* get() const { return _ptr.get(); }
        AbstractProperty* get() { return _ptr.upd(); }
        const AbstractProperty& operator*() const { return *_ptr; }
        AbstractProperty& operator*() { return *_ptr; }
        const AbstractProperty* operator->() const { return _ptr.get(); }
        AbstractProperty* operator->() { return _ptr.upd(); }
    private:
        SimTK::ClonePtr<AbstractProperty> _ptr;
    };

    std::vector<PropertyHolder> _properties;
    std::unordered_map<std::string, int> _namelookup;

};  // END of class PropertyTable

}; //namespace

#endif // OPENSIM_PROPERTY_TABLE_H_
