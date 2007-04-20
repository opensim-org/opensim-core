#ifndef __PropertyGroup_h__
#define __PropertyGroup_h__

// PropertyGroup.h
// Author: Peter Loan
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDE
#include "osimCommonDLL.h"
#include "Property.h"
#include "Array.h"

namespace OpenSim {

#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * A class implementing a property group.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API PropertyGroup
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Name of the group. */
	std::string _name;

protected:
	/** Pointers to the properties in the group. */
	Array<Property*> _properties;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyGroup();
	PropertyGroup(std::string& aName);
	PropertyGroup(const PropertyGroup &aGroup);
	virtual ~PropertyGroup();
	virtual PropertyGroup* copy() const;

#ifndef SWIG
	PropertyGroup& operator=(const PropertyGroup &aGroup);
	bool operator<(const PropertyGroup &aGroup) const;
	bool operator==(const PropertyGroup& aGroup) const;
#endif
   void copyData(const PropertyGroup &aGroup);
	void clear();

	bool contains(const std::string& aName) const;
	void add(Property* aProperty);
	void remove(Property* aProperty);
	const Array<Property*>& getProperties() const { return _properties; }
	Property* get(int aIndex);
	int getPropertyIndex(Property* aProperty) const;

	// NAME
	void setName(const std::string &aName) { _name = aName; }
	const std::string& getName() const { return _name; }

private:
	void setNull();
//=============================================================================
};	// END of class PropertyGroup
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PropertyGroup_h__


