#ifndef _PropertyTable_h_
#define _PropertyTable_h_
// PropertyTable.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson, Ajay Seth 
 */

// INCLUDES
#include "osimCommonDLL.h"
#include "ArrayPtrs.h"
#include "Property2.h"
#include "PropertyGroup.h"
#include "map"
#include "iterator"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A property table is simply a table of properties.  It provides methods for
 * adding, removing, and retrieving properties from itself.
 *
 * @version 1.0
 * @author Cassidy Kelly
 * @see Property
 */
class PropertyTable
{

//=============================================================================
// DATA
//=============================================================================
public:
	/** Set of properties. */
	std::map<std::string, AbstractProperty*> _properties;

protected:
	/** Array of property groups. */
	ArrayPtrs<PropertyGroup> _propertyGroups;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyTable();
	PropertyTable(const PropertyTable &aPropertyTable);
	virtual ~PropertyTable() {};
	PropertyTable& operator=(const PropertyTable &aPropertyTable);
	bool operator==(const PropertyTable &aPropertyTable) const;
	AbstractProperty* getPropertyPtr(const std::string &name) const;
	void addProperty(AbstractProperty &aProperty);
	template <class T> const Property2<T>& getProperty(const std::string &name) const;
	template <class T> Property2<T>& updateProperty(const std::string &name);
	template <class T> void addProperty(const std::string &name, const std::string &type, const std::string &comment, const T &value);
	template <class T> const T& getPropertyValue(const std::string &name) const;
	template <class T> T& updatePropertyValue(const std::string &name) const;
	template <class T> void setPropertyValue(const std::string &name, const T &value);
	std::string getPropertyType(const std::string &name) const;
	std::string getPropertyComment(const std::string &name) const;
	std::map<std::string, AbstractProperty*>::iterator begin();
	std::map<std::string, AbstractProperty*>::iterator end();
	int getSize() const;


//=============================================================================
};	// END of class PropertyTable

template <class T>
const Property2<T>& PropertyTable::getProperty(const std::string &name) const
{
	std::map<std::string, AbstractProperty*>::const_iterator it = _properties.find(name);
	if (it != _properties.end()) {
		const AbstractProperty& prop = *(*it).second;
		return dynamic_cast<const Property2<T>&>(prop);
	}
	throw Exception("Property " + name + " not found!");
}

template <class T>
Property2<T>& PropertyTable::updateProperty(const std::string &name)
{
	std::map<std::string, AbstractProperty*>::iterator it = _properties.find(name);
	if (it != _properties.end()) {
		AbstractProperty& prop = *(*it).second;
		return dynamic_cast<Property2<T>&>(prop);
	}
	throw Exception("Property " + name + " not found!");
}

template <class T>
void PropertyTable::addProperty(const std::string &name, const std::string &type, const std::string &comment, const T &value)
{
	Property2<T>* propPtr = new Property2<T>(name, type, comment, value);
	_properties.insert(std::pair<std::string, AbstractProperty*>(name, propPtr));
}

template <class T>
const T& PropertyTable::getPropertyValue(const std::string &name) const
{
	std::map<std::string, AbstractProperty*>::const_iterator it = _properties.find(name);
	if (it != _properties.end()) {
		const Property2<T>& prop = dynamic_cast<const Property2<T>&>(*(*it).second);
		return prop.getValue();
	}
	throw Exception("Property " + name + " not found!");
}

template <class T>
T& PropertyTable::updatePropertyValue(const std::string &name) const
{
	std::map<std::string, AbstractProperty*>::const_iterator it = _properties.find(name);
	if (it != _properties.end()) {
		Property2<T>& prop = dynamic_cast<Property2<T>&>(*(*it).second);
		return prop.updateValue();
	}
	throw Exception("Property " + name + " not found!");
}

template <class T>
void PropertyTable::setPropertyValue(const std::string &name, const T &value)
{
	std::map<std::string, AbstractProperty*>::iterator it = _properties.find(name);
	if (it != _properties.end()) {
		Property2<T>& prop = dynamic_cast<Property2<T>&>(*(it->second));
		prop.setValue(value);
	}
	else{
		throw Exception("Property " + name + " not found!");
	}
}

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyTable_h__
