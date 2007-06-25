#ifndef __ObjectGroup_h__
#define __ObjectGroup_h__

// ObjectGroup.h
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
#include <iostream>
#include <string>
#include <math.h>
#include "osimCommonDLL.h"
#include "PropertyStrArray.h"
#include "ArrayPtrs.h"
#include "Object.h"

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
 * A class implementing an object group. For most uses, object groups are
 * owned and managed by the Set that contains the object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API ObjectGroup : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:
	PropertyStrArray _memberNamesProp;
	Array<std::string>& _memberNames;

	ArrayPtrs<Object> _memberObjects;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ObjectGroup();
	ObjectGroup(const std::string& aName);
	ObjectGroup(const ObjectGroup &aGroup);
	virtual ~ObjectGroup();
	virtual Object* copy() const;

#ifndef SWIG
	ObjectGroup& operator=(const ObjectGroup &aGroup);
#endif
   void copyData(const ObjectGroup &aGroup);

	bool contains(const std::string& aName) const;
	void add(Object* aObject);
	void remove(const Object* aObject);
	void replace(const Object* aOldObject, Object* aNewObject);
	void setup(ArrayPtrs<Object>& aObjects);
	const ArrayPtrs<Object>& getMembers() const
	{
		return _memberObjects;
	}

	OPENSIM_DECLARE_DERIVED(ObjectGroup, Object);

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class ObjectGroup
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ObjectGroup_h__


