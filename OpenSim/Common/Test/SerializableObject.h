#ifndef _SerializableObject_h_
#define _SerializableObject_h_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  SerializableObject.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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
#include <OpenSim/Common/SimmSpline.h>
#include "SerializableObject2.h"

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

class SerializableObject : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(SerializableObject, Object);
public:
    //=============================================================================
    // PROPERTIES
    //=============================================================================
    OpenSim_DECLARE_PROPERTY(Test_Bool_2, bool, "Comment on a bool"); 
    // Int
    OpenSim_DECLARE_PROPERTY(Test_Int_2, int, "Comment on a int");
    // Dbl
    OpenSim_DECLARE_PROPERTY(Test_Infinity_2, double,
        "Comment on a double infinity");
    // Dbl
    OpenSim_DECLARE_PROPERTY(Test_MinusInfinity_2, double,
        "Comment on a double minus infinity");
    // Dbl
    OpenSim_DECLARE_PROPERTY(Test_Dbl_2, double, "Comment on a double");
    // Dbl
    OpenSim_DECLARE_PROPERTY(Test_NaN_2, double,
        "Comment on a double not a number");
    // Str
    OpenSim_DECLARE_PROPERTY(Test_Str_2, std::string,
        "Comment on a string");         
    // Obj
    OpenSim_DECLARE_PROPERTY(Test_Obj_2, SerializableObject3,
        "Comment on an Object");            
    // IntArray
    OpenSim_DECLARE_LIST_PROPERTY(Test_IntArray_2, int,
        "Comment on an int-array");
    // DblArray
    OpenSim_DECLARE_LIST_PROPERTY(Test_DblArray_2, double,
        "Comment on a double-array");
    OpenSim_DECLARE_LIST_PROPERTY(Test_StrArray_2, std::string,
        "Comment on a string-array");
    // ObjArray
    OpenSim_DECLARE_LIST_PROPERTY(Test_ObjArray_2, Object,
        "Comment on Object Array");
    // Transform
    OpenSim_DECLARE_PROPERTY(MyTransformProperty_2, SimTK::Transform,
        "Comment on Transform");
    // Vec3
    OpenSim_DECLARE_PROPERTY(Test_DblVec3_2, SimTK::Vec3,
        "Point at 3,5,7");
    // Nameless Obj
    OpenSim_DECLARE_UNNAMED_PROPERTY(SerializableObject3,
        "Comment on nameless Object");
    //=============================================================================
    // METHODS
    //=============================================================================
public:
    SerializableObject(){
        setNull();
        setupSerializedMembers();
    }
    SerializableObject(const std::string &aFileName) 
    :   Object(aFileName,false)
    {
        setNull();
        setupSerializedMembers();
        SimTK::Xml::Element e = updDocument()->getRootDataElement(); 
        updateFromXMLNode(e, getDocument()->getDocumentVersion());
    }
    SerializableObject(const SerializableObject& source)
    :   Object(source)
    {
        setNull();
        setupSerializedMembers(&source);
        *this = source;
    }

private:
    void setNull() {
        _propertySet._array.setMemoryOwner(true);
    }
    // If sourcep is set we are doing a copy construction in which case old
    // and new properties must be handled differently. The old ones must be
    // recreated from scratch; the new ones are copied automatically in the
    // base class but the local index must be reset here using copyProperty_()
    // methods.
    void setupSerializedMembers(const SerializableObject* sourcep=0){
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
        SerializableObject2 obj;
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
        SerializableObject2 object;
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

        if (sourcep) {
            // Just reset local indices for copy construction of new props.
            const SerializableObject& source = *sourcep;
            copyProperty_Test_Bool_2(source);
            copyProperty_Test_Int_2(source);
            copyProperty_Test_Infinity_2(source);
            copyProperty_Test_MinusInfinity_2(source);
            copyProperty_Test_Dbl_2(source);
            copyProperty_Test_NaN_2(source);
            copyProperty_Test_Str_2(source);
            copyProperty_Test_Obj_2(source);
            copyProperty_Test_IntArray_2(source);
            copyProperty_Test_DblArray_2(source);
            copyProperty_Test_StrArray_2(source);
            copyProperty_Test_ObjArray_2(source);
            copyProperty_MyTransformProperty_2(source);
            copyProperty_Test_DblVec3_2(source);
            copyProperty_SerializableObject3(source);
            return;
        }

        // This is a first-time construction.
        constructProperty_Test_Bool_2(true);
        constructProperty_Test_Int_2(0);
        constructProperty_Test_Infinity_2(SimTK::Infinity);
        constructProperty_Test_MinusInfinity_2(-SimTK::Infinity);
        constructProperty_Test_Dbl_2(1.23456);
        constructProperty_Test_NaN_2(SimTK::NaN);
        constructProperty_Test_Str_2("ABC");

        // Obj
        SerializableObject3 obj2;
        obj2.setName("Test_Obj_2");
        constructProperty_Test_Obj_2(obj2);

        // IntArray
        Array<int> arrayInt2(2);
        arrayInt2.setSize(4);
        for(i=0;i<arrayInt.getSize();i++) arrayInt2[i] = i;
        constructProperty_Test_IntArray_2(arrayInt2);

        // DblArray
        Array<double> arrayDbl2(0.0);
        arrayDbl2.setSize(4);
        for(i=0;i<arrayDbl.getSize();i++) arrayDbl2[i] = (double)i;
        constructProperty_Test_DblArray_2(arrayDbl2);

        // StrArray
        Array<string> arrayStr2("");
        arrayStr2.setSize(4);
        arrayStr2[0] = "abc";
        arrayStr2[1] = "def";
        arrayStr2[2] = "ghi";
        arrayStr2[3] = "jkl";
        constructProperty_Test_StrArray_2(arrayStr2);

        // ObjArray
        ArrayPtrs<Object> arrayObj2;
        SerializableObject2 object2;
        object2.setName("Obj1");
        arrayObj2.append(object2.clone());
        object2.setName("Obj2");
        arrayObj2.append(object2.clone());
        object2.setName("Obj3");
        arrayObj2.append(object2.clone());
        constructProperty_Test_ObjArray_2(arrayObj2);

        // Transform
        SimTK::Transform xform2;
        xform2.updP() = SimTK::Vec3(3., 2., 1.);
        xform2.updR().setRotationToBodyFixedXYZ(SimTK::Vec3(2, 1, 0.5));
        constructProperty_MyTransformProperty_2(xform2);

        // Vec3
        constructProperty_Test_DblVec3_2(SimTK::Vec3(3., 5., 7.));

        // Nameless Obj
        constructProperty_SerializableObject3(obj2);
    }
    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
    SerializableObject& operator=(const SerializableObject &aObject){
        Object::operator=(aObject);
        return(*this);
    };


    //--------------------------------------------------------------------------
    // XML SERIALIZATION
    //--------------------------------------------------------------------------
    virtual bool isValidDefaultType(const Object *aObject) const{
        if(aObject==NULL) return(false);

        string type1 = "SerializableObject2";
        if(type1 == aObject->getConcreteClassName()) return(true);

        return(false);
    }
    //=============================================================================
};

} //namespace

//=============================================================================
//=============================================================================

#endif // __SerializableObject_h__
