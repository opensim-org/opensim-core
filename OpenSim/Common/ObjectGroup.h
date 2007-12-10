#ifndef __ObjectGroup_h__
#define __ObjectGroup_h__

// ObjectGroup.h
// Author: Peter Loan
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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

	Array<Object*> _memberObjects;

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
	const Array<Object*>& getMembers() const { return _memberObjects; }

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


