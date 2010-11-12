#ifndef _rdSerializableObject_h_
#define _rdSerializableObject_h_
// rdSerializableObject.h:
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


// INCLUDES
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyTransform.h>
#include <OpenSim/Common/PropertyObjArray.h>

#include "rdSerializableObject2.h"
//extern template class OSIMCOMMON_API Array<double>;

//=============================================================================
//=============================================================================
/**
* An object for mainly for testing XML serialization.
*
* @author Frank C. Anderson
* @version 1.0
*/
using std::string;
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
		rdSerializableObject(){
			setNull();
			setupSerializedMembers();
		};
		rdSerializableObject(const std::string &aFileName) :
		Object(aFileName,false)
		{
			setNull();
			setupSerializedMembers();
			updateFromXMLNode();
		};
		rdSerializableObject(const rdSerializableObject &aControl)
		{
			setNull();
			setupSerializedMembers();
			*this = aControl;
		};
		virtual Object* copy() const
		{
			rdSerializableObject *object = new rdSerializableObject(*this);
			return(object);
		};

	private:
		void setNull() {setType("rdSerializableObject");};
		void setupSerializedMembers(){
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
			PropertyDbl pDbl1("Test_Infinity",SimTK::Infinity);
			pDbl1.setComment("Comment on a Double Infinity");
			_propertySet.append(pDbl1.copy());

			// Dbl
			PropertyDbl pDbl2("Test_MinusInfinity",-SimTK::Infinity);
			pDbl2.setComment("Comment on a Double");
			_propertySet.append(pDbl2.copy());

			// Dbl
			PropertyDbl pDbl3("Test_Dbl",1.23456);
			pDbl3.setComment("Comment on a Double");
			_propertySet.append(pDbl3.copy());

			// Dbl
			PropertyDbl pDbl4("Test_NaN",SimTK::NaN);
			pDbl4.setComment("Comment on a Double");
			_propertySet.append(pDbl4.copy());

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

			// Transform
			SimTK::Transform xform;
			xform.updP() = SimTK::Vec3(3., 2., 1.);
			xform.updR().setRotationToBodyFixedXYZ(SimTK::Vec3(2, 1, 0.5));
			PropertyTransform* transformP=new PropertyTransform("MyTransformProperty", xform);
			_propertySet.append(transformP);
			/*
			PropertyObj pObj("Test_Obj",obj);
			pObj.setComment("Comment on an Object");
			_propertySet.append(pObj.copy());*/

		}
		//--------------------------------------------------------------------------
		// OPERATORS
		//--------------------------------------------------------------------------
	public:
		rdSerializableObject& operator=(const rdSerializableObject &aObject){
			Object::operator=(aObject);
			return(*this);
		};


		//--------------------------------------------------------------------------
		// XML SERIALIZATION
		//--------------------------------------------------------------------------
		virtual bool isValidDefaultType(const Object *aObject) const{
			if(aObject==NULL) return(false);

			string type1 = "rdSerializableObject2";
			if(type1 == aObject->getType()) return(true);

			return(false);
		}
		//=============================================================================
	};

}; //namespace

//=============================================================================
//=============================================================================

#endif // __rdSerializableObject_h__
