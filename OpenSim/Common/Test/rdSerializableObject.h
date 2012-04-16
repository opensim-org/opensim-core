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
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyTransform.h>
#include <OpenSim/Common/PropertyObjArray.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/NaturalCubicSpline.h>
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

class rdSerializableObject : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(rdSerializableObject, Object);

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
	}
	rdSerializableObject(const std::string &aFileName) :
	Object(aFileName,false)
	{
		setNull();
		setupSerializedMembers();
		SimTK::Xml::Element e = updDocument()->getRootDataElement(); 
		updateFromXMLNode(e, getDocument()->getDocumentVersion());
	}
	rdSerializableObject(const rdSerializableObject &aControl)
	{
		setNull();
		setupSerializedMembers();
		*this = aControl;
	}

private:
	void setNull() {}
	void setupSerializedMembers(){
		int i;

		// Bool
		PropertyBool pBool("Test_Bool",true);
		pBool.setComment("Comment on deprecated boolean");
		_propertySet.append(pBool.clone());

		// Int
		PropertyInt pInt("Test_Int",0);
		pInt.setComment("Comment on deprecated Int");
		_propertySet.append(pInt.clone());

		// Dbl
		PropertyDbl pDbl1("Test_Infinity",SimTK::Infinity);
		pDbl1.setComment("Comment on deprecated Double Infinity");
		_propertySet.append(pDbl1.clone());

		// Dbl
		PropertyDbl pDbl2("Test_MinusInfinity",-SimTK::Infinity);
		pDbl2.setComment("Comment on deprecated Double");
		_propertySet.append(pDbl2.clone());

		// Dbl
		PropertyDbl pDbl3("Test_Dbl",1.23456);
		pDbl3.setComment("Comment on deprecated Double");
		_propertySet.append(pDbl3.clone());

		// Dbl
		PropertyDbl pDbl4("Test_NaN",SimTK::NaN);
		pDbl4.setComment("Comment on deprecated Double");
		_propertySet.append(pDbl4.clone());

		// Str
		PropertyStr pStr("Test_Str","ABC");
		pStr.setComment("Comment on deprecated String");
		_propertySet.append(pStr.clone());

		// Obj
		rdSerializableObject2 obj;
		PropertyObj pObj("Test_Obj",obj);
		pObj.setComment("Comment on deprecated Object");
		_propertySet.append(pObj.clone());

		// IntArray
		Array<int> arrayInt(2);
		arrayInt.setSize(4);
		for(i=0;i<arrayInt.getSize();i++) arrayInt[i] = i;
		PropertyIntArray pIntArray("Test_IntArray",arrayInt);
		pIntArray.setComment("Comment on deprecated int-array");
		_propertySet.append(pIntArray.clone());

		// DblArray
		Array<double> arrayDbl(0.0);
		arrayDbl.setSize(4);
		for(i=0;i<arrayDbl.getSize();i++) arrayDbl[i] = (double)i;
		PropertyDblArray pDblArray("Test_DblArray",arrayDbl);
		pDblArray.setComment("Comment on deprecated Dbl-array");
		_propertySet.append(pDblArray.clone());

		// StrArray
		Array<string> arrayStr("");
		arrayStr.setSize(4);
		arrayStr[0] = "abc";
		arrayStr[1] = "def";
		arrayStr[2] = "ghi";
		arrayStr[3] = "jkl";
		PropertyStrArray pStrArray("Test_StrArray",arrayStr);
		pStrArray.setComment("Comment on deprecated str-array");
		_propertySet.append(pStrArray.clone());

		// ObjArray
		ArrayPtrs<Object> arrayObj;
		rdSerializableObject2 object;
		object.setName("Obj1");
		arrayObj.append(object.clone());
		object.setName("Obj2");
		arrayObj.append(object.clone());
		object.setName("Obj3");
		arrayObj.append(object.clone());
		PropertyObjArray<Object> pObjArray("Test_ObjArray",arrayObj);
		pObjArray.setComment("Comment on deprecated Object Array");
		_propertySet.append(pObjArray.clone());

		// Transform
		SimTK::Transform xform;
		xform.updP() = SimTK::Vec3(3., 2., 1.);
		xform.updR().setRotationToBodyFixedXYZ(SimTK::Vec3(2, 1, 0.5));
		PropertyTransform* transformP =
            new PropertyTransform("MyTransformProperty", xform);
        transformP->setComment("Comment on deprecated Transform");
		_propertySet.append(transformP);
			
        // Vec3
		PropertyDblVec3* propPoint = 
            new PropertyDblVec3("Test_DblVec3",SimTK::Vec3(3., 5., 7.));
		propPoint->setComment("deprecated Point at 3,5,7");
		_propertySet.append(propPoint);

		// Nameless Obj
		PropertyObj pNamelessObj("",obj);
		pNamelessObj.setComment("Comment on deprecated nameless Object");
		_propertySet.append(pNamelessObj.clone());

		// Bool
		addProperty<bool>("Test_Bool_2",
			"Comment on a boolean",
			true);

		// Int
		addProperty<int>("Test_Int_2",
			"Comment on a int",
			0);

		// Dbl
		addProperty<double>("Test_Infinity_2",
			"Comment on a double infinity",
			SimTK::Infinity);

		// Dbl
		addProperty<double>("Test_MinusInfinity_2",
			"Comment on a double minus infinity",
			-SimTK::Infinity);

		// Dbl
		addProperty<double>("Test_Dbl_2",
			"Comment on a double",
			1.23456);

		// Dbl
		addProperty<double>("Test_NaN_2",
			"Comment on a double not a number",
			SimTK::NaN);

		// Str
		addProperty<string>("Test_Str_2",
			"Comment on a string",
			"ABC");
			
		// Obj
		rdSerializableObject3 obj2;
		obj2.setName("Test_Obj_2");
		addProperty<rdSerializableObject3>("Test_Obj_2",
			"Comment on an Object",
			obj2);
			
		// IntArray
		Array<int> arrayInt2(2);
		arrayInt2.setSize(4);
		for(i=0;i<arrayInt.getSize();i++) arrayInt2[i] = i;

		addListProperty<int>("Test_IntArray_2",
			"Comment on an int-array",
			arrayInt2);

		// DblArray
		Array<double> arrayDbl2(0.0);
		arrayDbl2.setSize(4);
		for(i=0;i<arrayDbl.getSize();i++) arrayDbl2[i] = (double)i;

		addListProperty<double>("Test_DblArray_2",
			"Comment on a double-array",
			arrayDbl2);

		// StrArray
		Array<string> arrayStr2("");
		arrayStr2.setSize(4);
		arrayStr2[0] = "abc";
		arrayStr2[1] = "def";
		arrayStr2[2] = "ghi";
		arrayStr2[3] = "jkl";

		addListProperty<string>("Test_StrArray_2",
			"Comment on a string-array",
			arrayStr2);

		// ObjArray
		ArrayPtrs<Object> arrayObj2;
		rdSerializableObject2 object2;
		object2.setName("Obj1");
		arrayObj2.append(object2.clone());
		object2.setName("Obj2");
		arrayObj2.append(object2.clone());
		object2.setName("Obj3");
		arrayObj2.append(object2.clone());

		addListProperty<Object>("Test_ObjArray_2",
			"Comment on Object Array",
			arrayObj2);

		// Transform
		SimTK::Transform xform2;
		xform2.updP() = SimTK::Vec3(3., 2., 1.);
		xform2.updR().setRotationToBodyFixedXYZ(SimTK::Vec3(2, 1, 0.5));
        addProperty<SimTK::Transform>("MyTransformProperty",
            "Comment on Transform",
            xform2);

        // Vec3
		addProperty<SimTK::Vec3>("Test_DblVec3_2",
			"Point at 3,5,7",
			SimTK::Vec3(3., 5., 7.));

		// Nameless Obj
		addProperty<rdSerializableObject3>("",
			"Comment on nameless Object",
			obj2);
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
		if(type1 == aObject->getConcreteClassName()) return(true);

		return(false);
	}
	//=============================================================================
};

} //namespace

//=============================================================================
//=============================================================================

#endif // __rdSerializableObject_h__
