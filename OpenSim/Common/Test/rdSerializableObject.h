#ifndef _rdSerializableObject_h_
#define _rdSerializableObject_h_
// rdSerializableObject.h:
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
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Common/Object.h>

//extern template class OSIMCOMMON_API Array<double>;

//=============================================================================
//=============================================================================
/**
 * An object for mainly for testing XML serialization.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class rdSerializableObject : public Object
{

//=============================================================================
// MEMBER DATA
//=============================================================================

//=============================================================================
// METHODS
//=============================================================================
public:
	rdSerializableObject();
	rdSerializableObject(const std::string &aFileName);
	rdSerializableObject(const rdSerializableObject &aNode);
	virtual Object* copy() const;
private:
	void setNull();
	void setupSerializedMembers();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	rdSerializableObject& operator=(const rdSerializableObject &aObject);

	//--------------------------------------------------------------------------
	// XML SERIALIZATION
	//--------------------------------------------------------------------------
	virtual bool isValidDefaultType(const Object *aObject) const;

//=============================================================================
};

}; //namespace

//=============================================================================
//=============================================================================

#endif // __rdSerializableObject_h__
