#ifndef OPENSIM_PROPERTY_TABLE_H_
#define OPENSIM_PROPERTY_TABLE_H_
// PropertyTable.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2011-12, Stanford University. All rights reserved. 
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

// INCLUDES
#include "osimCommonDLL.h"
#include "ArrayPtrs.h"
#include "Property.h"
#include "PropertyGroup.h"

#include <map>
#include <iterator>

namespace OpenSim {

//==============================================================================
//                              PROPERTY TABLE
//==============================================================================
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
 * @author Cassidy Kelly, Michael Sherman
 * @see AbstractProperty, Property, Object
 */
class OSIMCOMMON_API PropertyTable {
//==============================================================================
// METHODS
//==============================================================================
public:
    /** Create an empty table. **/
    PropertyTable() {}
    /** The destructor deletes all the property object in this table; any
    existing references to them or to their values become invalid. **/
	~PropertyTable();
    /** Copy constructor performs a deep copy; that is, this table contains
    new property objects initialized from those in the source. **/
	PropertyTable(const PropertyTable& source);
    /** Copy assignment performs a deep copy; this table will be as though
    it had been copy constructed from the source. **/
	PropertyTable& operator=(const PropertyTable& source);

    /** Delete all the properties currently in this table, restoring the
    table to its default-constructed state. **/
    void clear();

    /** Compare this table to another one for equality. Two tables are defined
    to be equal if they have the same number of properties, and each property
    tests equal to the one with the same index. **/
    bool equals(const PropertyTable& other) const;

    #ifndef SWIG
    /** See equals() for the meaning of this operator. **/
    bool operator==(const PropertyTable& other) const {return equals(other);}
    #endif

    /** Add a new property to this table, taking over ownership of the
    supplied heap-allocated property. Throws an exception if there is already 
    a property with the same name in the table. Returns an index (ordinal) 
    that can be used to retrieve this property quickly. **/ 
    int adoptProperty(AbstractProperty* prop);

    /** Return a const reference to a property of known type T from the 
    table by name. This will throw an exception if no property with this name
    is present, or if the property is present but not of type T. **/
	template <class T> const Property<T>& 
    getProperty(const std::string& name) const;

    /** Return a const reference to a property of known type T from the 
    table by its index (numbered in order of addition to the table). This will
    throw an exception if the index is out of range, or if the property is 
    present but not of type T. **/
	template <class T> const Property<T>& getProperty(int index) const;

    /** Return a writable reference to a property of known type T from the 
    table by name. This will throw an exception if no property with this name
    is present, or if the property is present but not of type T. **/
	template <class T> Property<T>& updProperty(const std::string& name);

    /** Return a writable reference to a property of known type T from the 
    table by its index (numbered in order of addition to the table). This will 
    throw an exception if the index is out of range, or if the property is
    present but not of type T. **/
	template <class T> Property<T>& updProperty(int index);

    /** Return true if there is a property with the given name currently 
    stored in this table. **/
    bool hasProperty(const std::string& name) const
    {   return findPropertyIndex(name) >= 0; }

    /** Look up a property by name and return a pointer providing const access
    to the stored AbstractProperty object if present, otherwise null. **/
	const AbstractProperty* getPropertyPtr(const std::string& name) const {
        const int ix = findPropertyIndex(name);
        return ix < 0 ? NULL : &getAbstractPropertyByIndex(ix); 
    }
    /** Look up a property by name and return a pointer providing writable
    access to the stored AbstractProperty object if present, otherwise null. **/
	AbstractProperty* updPropertyPtr(const std::string& name) {
        const int ix = findPropertyIndex(name);
        return ix < 0 ? NULL : &updAbstractPropertyByIndex(ix); 
    }

    /** Return the number of properties currently in this table. If this
    returns n, the properties are indexed 0 to n-1, in the order they were
    added to the table. **/
    int getNumProperties() const {return (int)properties.size();}

    /** Retrieve a property by its index, which must be in the range 
    0..getNumProperties()-1. The property index is assigned in the order that
    the properties were added to this table. **/
    const AbstractProperty& getAbstractPropertyByIndex(int index) const;

    /** Retrieve a writable reference to a property by its index, which must be
    in the range 0..getNumProperties()-1. The property index is assigned in the
    order that the properties were added to this table. **/
    AbstractProperty& updAbstractPropertyByIndex(int index);

    /** Retrieve a property by name if it exists, otherwise throw an 
    exception. The property is returned as an AbstractProperty; see
    getProperty<T>() if you know the property type. **/
    const AbstractProperty& 
    getAbstractPropertyByName(const std::string& name) const;

    /** Retrieve a writable reference to a property by name if it exists, 
    otherwise throw an exception. The property is returned as an 
    AbstractProperty; see updProperty<T>() if you know the property type. **/
    AbstractProperty& updAbstractPropertyByName(const std::string& name);

    /** Return the property's index if it is present, else -1. **/
    int findPropertyIndex(const std::string& name) const;

 
//==============================================================================
// DATA
//==============================================================================
private:
    // Make this properties array a deep copy of the source. Any existing
    // properties are deleted first. The index map is updated appropriately.
    void replaceProperties(const SimTK::Array_<AbstractProperty*>& source);
    // Delete all properties and clear the index map.
    void deleteProperties();

	// The properties, in the order they were added.
    SimTK::Array_<AbstractProperty*>    properties;
    // A mapping from property name to its index in the properties array.
	std::map<std::string, int>          propertyIndex;

//==============================================================================
};	// END of class PropertyTable


//==============================================================================
// IMPLEMENTATION OF TEMPLATIZED METHODS
//==============================================================================

template <class T> inline
const Property<T>& PropertyTable::getProperty(const std::string &name) const {
    const AbstractProperty& prop = getAbstractPropertyByName(name);
    return Property<T>::getAs(prop);
}

template <class T> inline
const Property<T>& PropertyTable::getProperty(int index) const {
    const AbstractProperty& prop = getAbstractPropertyByIndex(index);
    return Property<T>::getAs(prop);
}

template <class T> inline
Property<T>& PropertyTable::updProperty(const std::string &name) {
    AbstractProperty& prop = updAbstractPropertyByName(name);
    return Property<T>::updAs(prop);
}

template <class T> inline
Property<T>& PropertyTable::updProperty(int index) {
    AbstractProperty& prop = updAbstractPropertyByIndex(index);
    return Property<T>::updAs(prop);
}

}; //namespace

#endif // OPENSIM_PROPERTY_TABLE_H_