#ifndef _PropertyObj_h_
#define _PropertyObj_h_
// PropertyObj.h
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
#include "rdTools.h"
#include <string>
#include "Object.h"
#include "Property.h"


//=============================================================================
//=============================================================================
/**
 * Class PropertyObj extends class Property.  It consists of a pointer to
 * an object and the methods for accessing and modifying this object.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class RDTOOLS_API PropertyObj : public Property
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Value. */
	Object *_value;

	/** Whether to only consider XML elements with matching name attribute. */
	bool _matchName;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyObj();
	PropertyObj(const std::string &aName,const Object &aValue);
	PropertyObj(const PropertyObj &aProperty);
	virtual Property* copy() const;
	virtual ~PropertyObj();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyObj& operator=(const PropertyObj &aProperty);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// TYPE
	virtual const char* getTypeAsString() const;
	// VALUE
	// Got rid of setValue(Obj) since it would be dangerous to do so given that users of
	// PropertyObj would continue to hold a reference to the (deleted) object - Eran.
	virtual Object& getValueObj();
	virtual const Object& getValueObj() const;
	// VALUE as String
	virtual const std::string &toString();
	// MATCH NAME
	void setMatchName(bool aMatchName) { _matchName = aMatchName; }
	bool getMatchName() const { return _matchName; }

//=============================================================================
};	// END of class PropertyObj

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyObj_h__
