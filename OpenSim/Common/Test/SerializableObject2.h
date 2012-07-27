#ifndef _SerializableObject2_h_
#define _SerializableObject2_h_
// SerializableObject2.h:
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property_Deprecated.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyBool.h>

//=============================================================================
//=============================================================================
/**
* An object for mainly for testing XML serialization.
*
* @author Frank C. Anderson
* @version 1.0
*/
namespace OpenSim { 

class SerializableObject2 : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(SerializableObject2, Object);

	//=============================================================================
	// MEMBER DATA
	//=============================================================================

	//=============================================================================
	// METHODS
	//=============================================================================
public:
	SerializableObject2(){
		setNull();
		setupSerializedMembers();
	}
	SerializableObject2(const std::string &aFileName) :
	Object(aFileName,false)
	{
		setNull();
		setupSerializedMembers();
		updateFromXMLDocument();
	}
	//SerializableObject2(const SerializableObject2 &aObject){
	//	setNull();
	//	setupSerializedMembers();
	//	*this = aObject;
	//}

    OpenSim_DECLARE_PROPERTY(Test_Bool2, bool, "obj2's bool prop");
    OpenSim_DECLARE_LIST_PROPERTY(Test_DblArray2, double, 
        "obj2's double array prop");

private:
	void setNull(){
	};
	void setupSerializedMembers(){
		// Bool
		//PropertyBool pBool("Test_Bool2",false);
		//_propertySet.append(pBool.clone());
        constructProperty_Test_Bool2(false);

		// DblArray
		Array<double> dblArray(0.1);
		dblArray.setSize(3);
		//PropertyDblArray pDblArray("Test_DblArray2",dblArray);
		//_propertySet.append(pDblArray.clone());
        constructProperty_Test_DblArray2(dblArray);
	};

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	//SerializableObject2& operator=(const SerializableObject2 &aObject){
	//	Object::operator=(aObject);
 //       updPropertyByIndex(0).updValue<bool>()=
 //           aObject.getPropertyByIndex(0).getValue<bool>();
 //       AbstractProperty& prop = updPropertyByIndex(1);
 //       for (int i=0; i < prop.size(); ++i)
 //           prop.updValue<double>(i)= 
 //               aObject.getPropertyByIndex(1).getValue<double>(i);
	//	return(*this);
	//}

	//=============================================================================
};

class SerializableObject3 : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(SerializableObject3, Object);

	//=============================================================================
	// MEMBER DATA
	//=============================================================================

	//=============================================================================
	// METHODS
	//=============================================================================
public:
	SerializableObject3(){
		setNull();
		setupSerializedMembers();
	}
	SerializableObject3(const std::string &aFileName) :
	Object(aFileName,false)
	{
		setNull();
		setupSerializedMembers();
		updateFromXMLDocument();
	}
	SerializableObject3(const SerializableObject3 &aObject){
		setNull();
		setupSerializedMembers();
		*this = aObject;
	}

private:
	void setNull(){
	}
	void setupSerializedMembers(){
		// Bool
		PropertyBool pBool("Test_Bool2",false);
		_propertySet.append(pBool.clone());

		// DblArray
		Array<double> dblArray(0.1);
		dblArray.setSize(3);
		PropertyDblArray pDblArray("Test_DblArray2",dblArray);
		_propertySet.append(pDblArray.clone());
	}

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	SerializableObject3& operator=(const SerializableObject3 &aObject){
		Object::operator=(aObject);
        updPropertyByIndex(0).updValue<bool>()=
            aObject.getPropertyByIndex(0).getValue<bool>();
        AbstractProperty& prop = updPropertyByIndex(1);
        for (int i=0; i < prop.size(); ++i)
            prop.updValue<double>(i)= 
                aObject.getPropertyByIndex(1).getValue<double>(i);
		return(*this);
	}

	//=============================================================================
};

} //namespace

//=============================================================================
//=============================================================================

#endif // __SerializableObject2_h__
