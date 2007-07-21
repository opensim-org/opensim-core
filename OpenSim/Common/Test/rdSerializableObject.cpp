// rdSerializableObject.cpp
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
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyObjArray.h>
#include "rdSerializableObject.h"
#include "rdSerializableObject2.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// STATIC CONSTANTS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
rdSerializableObject::
rdSerializableObject()
{
	setNull();
	setupSerializedMembers();
}
//_____________________________________________________________________________
/**
 * File constructor.
 */
rdSerializableObject::
rdSerializableObject(const string &aFileName) :
	Object(aFileName,false)
{
	setNull();
	setupSerializedMembers();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aControl Control to copy.
 */
rdSerializableObject::rdSerializableObject(const rdSerializableObject &aControl)
{
	setNull();
	setupSerializedMembers();
	*this = aControl;
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this object.
 *
 * The object is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this object.
 */
Object* rdSerializableObject::
copy() const
{
	rdSerializableObject *object = new rdSerializableObject(*this);
	return(object);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL value for the member variables.
 */
void rdSerializableObject::
setNull()
{
	setType("rdSerializableObject");
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers of the serialized
 * member variables.
 */
void rdSerializableObject::
setupSerializedMembers()
{
	int i;

	// Bool
	PropertyBool pBool("Test_Bool",true);
	pBool.setComment("Comment on a boolean");
	_propertySet.append(pBool.copy());

	// Int
	PropertyInt pInt("Test_Int",0);
	pInt.setComment("Comment on a Int");
	_propertySet.append(pInt.copy());

	// Dbl
	PropertyDbl pDbl("Test_Dbl",0.0);
	pDbl.setComment("Comment on a Double");
	_propertySet.append(pDbl.copy());

	// Str
	PropertyStr pStr("Test_Str","ABC");
	pStr.setComment("Comment on a String");
	_propertySet.append(pStr.copy());

	// Obj
	rdSerializableObject2 obj;
	PropertyObj pObj("Test_Obj",obj);
	pObj.setComment("Comment on an Object");
	_propertySet.append(pObj.copy());

	// IntArray
	Array<int> arrayInt(2);
	arrayInt.setSize(4);
	for(i=0;i<arrayInt.getSize();i++) arrayInt[i] = i;
	PropertyIntArray pIntArray("Test_IntArray",arrayInt);
	pIntArray.setComment("Comment on an int-array");
	_propertySet.append(pIntArray.copy());

	// DblArray
	Array<double> arrayDbl(0.0);
	arrayDbl.setSize(4);
	for(i=0;i<arrayDbl.getSize();i++) arrayDbl[i] = (double)i;
	PropertyDblArray pDblArray("Test_DblArray",arrayDbl);
	pDblArray.setComment("Comment on a Dbl-array");
	_propertySet.append(pDblArray.copy());

	// StrArray
	Array<string> arrayStr("");
	arrayStr.setSize(4);
	arrayStr[0] = "abc";
	arrayStr[1] = "def";
	arrayStr[2] = "ghi";
	arrayStr[3] = "jkl";
	PropertyStrArray pStrArray("Test_StrArray",arrayStr);
	pStrArray.setComment("Comment on a str-array");
	_propertySet.append(pStrArray.copy());

	// ObjArray
	ArrayPtrs<Object> arrayObj;
	rdSerializableObject2 object;
	object.setName("Obj1");
	arrayObj.append(object.copy());
	object.setName("Obj2");
	arrayObj.append(object.copy());
	object.setName("Obj3");
	arrayObj.append(object.copy());
	PropertyObjArray<Object> pObjArray("Test_ObjArray",arrayObj);
	pObjArray.setComment("Comment on Object Array");
	_propertySet.append(pObjArray.copy());
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to the altered object.
 */
rdSerializableObject& rdSerializableObject::
operator=(const rdSerializableObject &aObject)
{
	Object::operator=(aObject);
	return(*this);
}


//=============================================================================
// SERIALIZATION
//=============================================================================
//-----------------------------------------------------------------------------
// DEFAULT OBJECTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine whether a specified object is a valid type for this
 * object.  This method returns true only for objects of type
 * rdSerializableObject2.
 *
 * @param aObject Object to be tested as valid or invalid.
 */
bool rdSerializableObject::
isValidDefaultType(const Object *aObject) const
{
	if(aObject==NULL) return(false);

	string type1 = "rdSerializableObject2";
	if(type1 == aObject->getType()) return(true);

	return(false);
}

