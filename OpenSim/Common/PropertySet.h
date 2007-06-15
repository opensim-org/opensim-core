#ifndef _PropertySet_h_
#define _PropertySet_h_
// PropertySet.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include "ArrayPtrs.h"
#include "Property.h"
#include "PropertyGroup.h"


#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
#endif

#ifndef SWIG
#ifdef WIN32
template class OSIMCOMMON_API OpenSim::ArrayPtrs<OpenSim::Property>;
#endif
#endif

//=============================================================================
//=============================================================================
/**
 * A property set is simply a set of properties.  It provides methods for
 * adding, removing, and retrieving properties from itself.
 *
 * @version 1.0
 * @author Frank C. Anderson
 * @see Property
 */
namespace OpenSim { 

class OSIMCOMMON_API PropertySet  
{

//=============================================================================
// DATA
//=============================================================================
public:
	/** Set of properties. */
	ArrayPtrs<Property> _array;

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
	PropertySet();
	PropertySet(const PropertySet &aSet);
	virtual ~PropertySet() { _array.setSize(0); };

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	friend std::ostream& operator<<(std::ostream &aOut,
															const PropertySet &aSet) {
		aOut << "\nProperty Set:\n";
		for(int i=0;i<aSet.getSize();i++) aOut << *aSet.get(i) << "\n";
		return(aOut);
	}
#endif
	//--------------------------------------------------------------------------
	// ACCESS
	//--------------------------------------------------------------------------
public:
	// Empty?
	bool isEmpty() const;
	// Number of properties
	int getSize() const;
	// Get
	virtual Property* get(int i) throw (Exception);
#ifndef SWIG
	virtual const Property* get(int i) const;
#endif
	virtual Property* get(const std::string &aName) throw (Exception);
#ifndef SWIG
	virtual const Property* get(const std::string &aName) const;
#endif
	virtual const Property* contains(const std::string& aName) const;
	virtual Property* contains(const std::string& aName);
	// Append
	virtual void append(Property *aProperty);
	virtual void append(Property *aProperty, const std::string& aName);
	// Remove
	virtual void remove(const std::string &aName);
	// Clear
	virtual void clear();

   PropertyGroup* addGroup(std::string aGroupName);
   void addPropertyToGroup(std::string aGroupName, std::string aPropertyName);
   void addPropertyToGroup(PropertyGroup* aGroup, std::string aPropertyName);
	void addPropertyToGroup(PropertyGroup* aGroup, Property* aProperty);
	void addPropertyToGroup(std::string aGroupName, Property* aProperty);
	ArrayPtrs<PropertyGroup>& getGroups() { return _propertyGroups; }
	PropertyGroup* getGroupContaining(Property* aProperty);
	int getGroupIndexContaining(Property* aProperty);

//=============================================================================
};	// END of class PropertySet

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertySet_h__
