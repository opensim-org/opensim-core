// rdSerializableObject2.cpp
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
rdSerializableObject2::
rdSerializableObject2()
{
	setNull();
	setupSerializedMembers();
}
//_____________________________________________________________________________
/**
 * File constructor.
 */
rdSerializableObject2::
rdSerializableObject2(const string &aFileName) :
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
 * @param aObject Object to copy.
 */
rdSerializableObject2::
rdSerializableObject2(const rdSerializableObject2 &aObject)
{
	setNull();
	setupSerializedMembers();
	*this = aObject;
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
Object* rdSerializableObject2::
copy() const
{
	rdSerializableObject2 *object = new rdSerializableObject2(*this);
	return(object);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL value for the member variables.
 */
void rdSerializableObject2::
setNull()
{
	setType("rdSerializableObject2");
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void rdSerializableObject2::
setupSerializedMembers()
{
	// Bool
	PropertyBool pBool("Test_Bool2",false);
	_propertySet.append(pBool.copy());

	// DblArray
	Array<double> dblArray(0.1);
	dblArray.setSize(3);
	PropertyDblArray pDblArray("Test_DblArray2",dblArray);
	_propertySet.append(pDblArray.copy());
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
rdSerializableObject2& rdSerializableObject2::
operator=(const rdSerializableObject2 &aObject)
{
	Object::operator=(aObject);
	return(*this);
}


//=============================================================================
// SET / GET
//=============================================================================


