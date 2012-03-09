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
#include "Property2.h"
#include "PropertyGroup.h"

#include <map>
#include <iterator>

namespace OpenSim {

//==============================================================================
//==============================================================================
/**
 * A property table is the container that an OpenSim Object uses to hold its
 * properties (each derived from base class AbstractProperty). It provides 
 * methods to add properties to the table, and preserves the order with which 
 * they are defined. Methods for retrieving properties by name or by ordinal
 * are provided. The table is the owner of the individual property objects and
 * those are deleted when the table is deleted.
 *
 * @author Cassidy Kelly, Michael Sherman
 * @see AbstractProperty, Property2
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

    /** Compare this table to another one for equality. Two tables are defined
    to be equal if they have the same number of properties, and each property
    tests equal to the one with the same index. **/
    bool equals(const PropertyTable& other) const;

    #ifndef SWIG
    /** See equals() for the meaning of this operator. **/
    bool operator==(const PropertyTable& other) const {return equals(other);}
    #endif

    /** Add a new property of type T to this table, provide it an initial
    value of that type, and optionally assign the property to a property group.
    This will throw an exception if there is already a property with this name
    in the table. 
    TODO: groups not yet implemented. **/
	template <class T> void 
    addProperty(const std::string& name, 
                const std::string& comment, 
                const T&           value,
                const std::string& group = "Misc");

    /** Return a const reference to a property of known type T from the 
    table by name. This will throw an exception if no property with this name
    is present, or if the property is present but not of type T. **/
	template <class T> const Property2<T>& 
    getProperty(const std::string& name) const;

    /** Return a writable reference to a property of known type T from the 
    table by name. This will throw an exception if no property with this name
    is present, or if the property is present but not of type T. **/
	template <class T> Property2<T>& 
    updProperty(const std::string& name);

    /** Obtain a const reference to the current value of a property of known 
    type T, given the property name. This will throw an exception if no 
    property with this name is present, or if the property is present but not 
    of type T. **/
	template <class T> const T& 
    getPropertyValue(const std::string& name) const
    {   return getProperty<T>(name).getValue(); }

    /** Obtain a writable reference to the current value of a property of known 
    type T, given the property name. This will throw an exception if no 
    property with this name is present, or if the property is present but not 
    of type T. **/
	template <class T> T& 
    updPropertyValue(const std::string& name)
    {   return updProperty<T>(name).updValue(); }

    /** Assign a new value to a property of type T, given the property name
    and a new value. This will throw an exception if no property with this name
    is present, or if the property is present but not of type T. **/
	template <class T> void 
    setPropertyValue(const std::string& name, const T& value)
    {   updPropertyValue<T>(name) = value; }


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
    /** Look up a property by name and return a pointer providing writeable
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
    const AbstractProperty& getAbstractPropertyByIndex(int index) const
    {   return *properties[index]; }

    /** Retrieve a writable reference to a property by its index, which must be
    in the range 0..getNumProperties()-1. The property index is assigned in the
    order that the properties were added to this table. **/
    AbstractProperty& updAbstractPropertyByIndex(int index)
    {   return *properties[index]; }

    /** Retrieve a property by name if it exists, otherwise throw an 
    exception. The property is returned as an AbstractProperty; see
    getProperty<T>() if you know the property type. **/
    const AbstractProperty& 
    getAbstractProperty(const std::string& name) const {
        const AbstractProperty* p = getPropertyPtr(name);
        if (p == NULL) throw Exception("Property " + name + " not found.");
        return *p;
    }

    /** Retrieve a writable reference to a property by name if it exists, 
    otherwise throw an exception. The property is returned as an 
    AbstractProperty; see getProperty<T>() if you know the property type. **/
    AbstractProperty& updAbstractProperty(const std::string& name) {
        AbstractProperty* p = updPropertyPtr(name);
        if (p == NULL) throw Exception("Property " + name + " not found.");
        return *p;
    }

    /** Convenience method combining property lookup by name and returning
    the string name for the property's type. Will throw an exception if no
    property of this name is in the table. **/
	const std::string& getPropertyTypeAsString(const std::string& name) const
    {   return getAbstractProperty(name).getTypeAsString(); }
    /** Convenience method combining property lookup by name and returning
    the property's comment. Will throw an exception if no property of this name
    is in the table. **/
	const std::string& getPropertyComment(const std::string& name) const
    {   return getAbstractProperty(name).getComment(); }

//==============================================================================
// DATA
//==============================================================================
private:
    // Return the property's index if it is present, else -1.
    int findPropertyIndex(const std::string& name) const;
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
void PropertyTable::addProperty(const std::string& name, 
                                const std::string& comment, 
                                const T&           value,
                                const std::string& group)
{
    if (hasProperty(name))
        throw Exception("Property " + name + " already in table.");

    const int nxtIndex = properties.size();
    assert(propertyIndex.size() == nxtIndex); // tables must be the same size

    properties.push_back(new Property2<T>(name, comment, value));
    propertyIndex[name] = nxtIndex;

    //TODO: deal with group
}

template <class T> inline
const Property2<T>& PropertyTable::getProperty(const std::string &name) const {
    const AbstractProperty& prop = getAbstractProperty(name);
    const Property2<T>* propT = dynamic_cast<const Property2<T>*>(&prop);
    if (propT == NULL)
        throw Exception("Property " + name + " was not of type "
                        + std::string(PropertyTypeName<T>::name()));
    return *propT;
}

template <class T> inline
Property2<T>& PropertyTable::updProperty(const std::string &name) {
    AbstractProperty& prop = updAbstractProperty(name);
    Property2<T>* propT = dynamic_cast<Property2<T>*>(&prop);
    if (propT == NULL)
        throw Exception("Property " + name + " was not of type "
                        + std::string(PropertyTypeName<T>::name()));
    return *propT;
}



}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY_TABLE_H_