#ifndef _SerializableObject2_h_
#define _SerializableObject2_h_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  SerializableObject2.h                       *
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
    //  setNull();
    //  setupSerializedMembers();
    //  *this = aObject;
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
    //  Object::operator=(aObject);
 //       updPropertyByIndex(0).updValue<bool>()=
 //           aObject.getPropertyByIndex(0).getValue<bool>();
 //       AbstractProperty& prop = updPropertyByIndex(1);
 //       for (int i=0; i < prop.size(); ++i)
 //           prop.updValue<double>(i)= 
 //               aObject.getPropertyByIndex(1).getValue<double>(i);
    //  return(*this);
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
    SerializableObject3(const SerializableObject3 &aObject) :
        Object(aObject) {
        setNull();
        setupSerializedMembers();
        *this = aObject;
    }

private:
    void setNull(){
        _propertySet._array.setMemoryOwner(true);
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
