// PropertyTable.cpp
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
 * Author: Frank C. Anderson 
 */


//============================================================================
// INCLUDES
//============================================================================
#include "PropertyTable.h"


using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyTable::PropertyTable()
{
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTable Table of properties to be copied.
 */
PropertyTable::PropertyTable(const PropertyTable &aPropertyTable)
{
	_properties = aPropertyTable._properties;
}

PropertyTable::~PropertyTable()
{

}

PropertyTable& PropertyTable::operator=(const PropertyTable &aPropertyTable)
{
	_properties = aPropertyTable._properties;
	return *this;
}

bool PropertyTable::operator==(const PropertyTable &aPropertyTable) const
{
	map<string, AbstractProperty*>::const_iterator it;
	for (it = _properties.begin(); it != _properties.end(); ++it) {
		string name = (*it).first;
		AbstractProperty& prop1 = *(*it).second;
		if (!prop1.equals(aPropertyTable.getPropertyPtr(name)))
			return false;
	}
	return true;
}

AbstractProperty* PropertyTable::getPropertyPtr(const std::string &name) const
{
	return (*_properties.find(name)).second;
}

Array<AbstractProperty *> PropertyTable::getArray()
{
	Array<AbstractProperty *> propertyArray(NULL, _properties.size());
	for(map<string, AbstractProperty*>::iterator it = _properties.begin(); it != _properties.end(); ++it) {
		AbstractProperty *abstractProperty = (*it).second;
		propertyArray[abstractProperty->getIndex()] = abstractProperty;
	}
	return propertyArray;
}

void PropertyTable::addProperty(AbstractProperty &aProperty)
{
	aProperty.setIndex(_properties.size());
	_properties.insert(std::pair<std::string, AbstractProperty*>(aProperty.getName(), &aProperty));
}

std::string PropertyTable::getPropertyType(const std::string &name) const
{
	return (*(*_properties.find(name)).second).getType();
}

std::string PropertyTable::getPropertyComment(const std::string &name) const
{
	return (*(*_properties.find(name)).second).getComment();
}

int PropertyTable::getSize() const
{
	return _properties.size();
}