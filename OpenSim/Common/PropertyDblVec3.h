#ifndef _PropertyDblVec3_h_
#define _PropertyDblVec3_h_
// PropertyDblVec3.h
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

#ifdef WIN32
#pragma warning( disable : 4251 )
#endif

#include "osimCommonDLL.h"
#include <string>
#include "Property.h"
#include "SimTKcommon.h"

//=============================================================================
//=============================================================================
/**
 * Class PropertyDblVec3 extends class Property.  It consists of an
 * array of doubles (i.e., SimTK::Vec3) and the methods for accessing
 * and modifying this Vec3.
 *
 * @version 1.0
 * @author Ayman HAbib
 */
namespace OpenSim { 

class OSIMCOMMON_API PropertyDblVec3 : public Property
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Array of doubles. */
	SimTK::Vec3 _vec;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyDblVec3();
	PropertyDblVec3(const std::string &aName,
		const SimTK::Vec3& aVec3);
	PropertyDblVec3(const std::string &aName,
		const Array<double> &aArray);
	PropertyDblVec3(const PropertyDblVec3 &aProperty);
	virtual Property* copy() const;
	virtual ~PropertyDblVec3() { };

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyDblVec3& operator=(const PropertyDblVec3 &aProperty);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// TYPE
	virtual const char* getTypeAsString() const;
	// VALUE
	virtual void setValue(const SimTK::Vec3 &aVec3);
	virtual SimTK::Vec3& getValueDblVec3();
	virtual const SimTK::Vec3& getValueDblVec3() const;
	virtual void setValue(int aSize,const double aArray[]){ // to be used by the serialization code
		setValue(SimTK::Vec3::getAs(aArray));
	};	
	// VALUE as String
	virtual const std::string &toString();
	// SIZE
	virtual int getArraySize() const { return 3; }


//=============================================================================
};	// END of class PropertyDblVec3

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyDblVec3_h__
