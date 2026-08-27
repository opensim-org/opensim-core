/* -------------------------------------------------------------------------- *
 *                      OpenSim:  testSerialization.cpp                       *
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

#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertySet.h>
#include <OpenSim/Common/PropertyTransform.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/XMLDocument.h>

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <catch2/catch_all.hpp>
#include "SimTKcommon.h"

#include <iostream>
#include <string>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

namespace {

    // An object for mainly for testing XML serialization.
    class SerializableObject2 : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(SerializableObject2, Object);
    public:
        // PROPERTIES
        OpenSim_DECLARE_PROPERTY(Test_Bool2, bool, "obj2's bool prop");
        OpenSim_DECLARE_LIST_PROPERTY(Test_DblArray2, double,
            "obj2's double array prop");

        // CONSTRUCTION
        SerializableObject2() {
            setupSerializedMembers();
        }
        SerializableObject2(const std::string &aFileName) :
                Object(aFileName, false) {
            setupSerializedMembers();
            updateFromXMLDocument();
        }


    private:
        void setupSerializedMembers(){
            constructProperty_Test_Bool2(false);

            Array<double> dblArray(0.1);
            dblArray.setSize(3);
            constructProperty_Test_DblArray2(dblArray);
        };
    };

    // An object for mainly for testing XML serialization.
    class SerializableObject3 : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(SerializableObject3, Object);
    public:
        // CONSTRUCTION
        SerializableObject3() {
            setNull();
            setupSerializedMembers();
        }
        SerializableObject3(const std::string &aFileName) :
                Object(aFileName,false) {
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

        // OPERATORS
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
    };

    // An object for mainly for testing XML serialization.
    class SerializableObject : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(SerializableObject, Object);
    public:
        // PROPERTIES
        OpenSim_DECLARE_PROPERTY(Test_Bool_2, bool, "Comment on a bool");
        OpenSim_DECLARE_PROPERTY(Test_Int_2, int, "Comment on a int");
        OpenSim_DECLARE_PROPERTY(Test_Infinity_2, double,
            "Comment on a double infinity");
        OpenSim_DECLARE_PROPERTY(Test_MinusInfinity_2, double,
            "Comment on a double minus infinity");
        OpenSim_DECLARE_PROPERTY(Test_Dbl_2, double, "Comment on a double");
        OpenSim_DECLARE_PROPERTY(Test_NaN_2, double,
            "Comment on a double not a number");
        OpenSim_DECLARE_PROPERTY(Test_Str_2, std::string,
            "Comment on a string");
        OpenSim_DECLARE_PROPERTY(Test_Obj_2, SerializableObject3,
            "Comment on an Object");
        OpenSim_DECLARE_LIST_PROPERTY(Test_IntArray_2, int,
            "Comment on an int-array");
        OpenSim_DECLARE_LIST_PROPERTY(Test_DblArray_2, double,
            "Comment on a double-array");
        OpenSim_DECLARE_LIST_PROPERTY(Test_StrArray_2, std::string,
            "Comment on a string-array");
        OpenSim_DECLARE_LIST_PROPERTY(Test_ObjArray_2, Object,
            "Comment on Object Array");
        OpenSim_DECLARE_PROPERTY(MyTransformProperty_2, SimTK::Transform,
            "Comment on Transform");
        OpenSim_DECLARE_PROPERTY(Test_DblVec3_2, SimTK::Vec3,
            "Point at 3,5,7");
        OpenSim_DECLARE_UNNAMED_PROPERTY(SerializableObject3,
            "Comment on nameless Object");

        // CONSTRUCTION
        SerializableObject(){
            setNull();
            setupSerializedMembers();
        }
        SerializableObject(const std::string &aFileName) :
                Object(aFileName,false) {
            setNull();
            setupSerializedMembers();
            SimTK::Xml::Element e = updDocument()->getRootDataElement();
            updateFromXMLNode(e, getDocument()->getDocumentVersion());
        }
        SerializableObject(const SerializableObject& source) :
                Object(source) {
            setNull();
            setupSerializedMembers(&source);
            *this = source;
        }

        // OPERATORS
        SerializableObject& operator=(const SerializableObject &aObject){
            Object::operator=(aObject);
            return(*this);
        };

        // XML SERIALIZATION
        virtual bool isValidDefaultType(const Object *aObject) const{
            if(aObject==NULL) return(false);

            string type1 = "SerializableObject2";
            if(type1 == aObject->getConcreteClassName()) return(true);

            return(false);
        }

    private:
        void setNull() {
            _propertySet._array.setMemoryOwner(true);
        }
        // If sourcep is set we are doing a copy construction in which case old
        // and new properties must be handled differently. The old ones must be
        // recreated from scratch; the new ones are copied automatically in the
        // base class but the local index must be reset here using
        // copyProperty_() methods.
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
    };

    class ObjSet : public Set<SerializableObject> {
    OpenSim_DECLARE_CONCRETE_OBJECT(ObjSet, Set<SerializableObject>);
    };

    static void indent(int nSpaces) {
        for (int i=0; i<nSpaces; ++i) cout << " ";
    }

    class ObjectWithListProperty : public Object {
        OpenSim_DECLARE_CONCRETE_OBJECT(ObjectWithListProperty, Object);
    public:
        OpenSim_DECLARE_LIST_PROPERTY(list_SerializableObject, SerializableObject,
            "List of SerializableObject(s).");
        ObjectWithListProperty() {
            constructProperty_list_SerializableObject();
        }
    };
    // Recursively dump out contents of an object and its properties.
    static void dumpObj(const Object& obj, int nSpaces) {
        indent(nSpaces);
        cout << obj.getConcreteClassName() << " Object "
            << (obj.getName().empty()?"NONAME":obj.getName())
            << endl;
        for (int p=0; p < obj.getNumProperties(); ++p) {
            const AbstractProperty& ap = obj.getPropertyByIndex(p);
            indent(nSpaces+2);
            cout << ap.getName() << "=" << ap.toString()
                << " typeName=" << ap.getTypeName() << endl;
            if (ap.isObjectProperty()) {
                for (int i=0; i < ap.size(); ++i)
                    dumpObj(ap.getValueAsObject(i), nSpaces+4);
            }
        }
    }

    static void testPropertyOutputHelper(const double& val, const std::string& ans)
    {
        cout << "(double)" << val << ":  ";
        stringstream ss;

        Property<double>* propertyDouble =
            Property<double>::TypeHelper::create("double", true);

        propertyDouble->setValue(val);
        double valOut = propertyDouble->getValue();

        writeUnformatted(ss, valOut);
        cout << std::to_string(valOut) << " " << ss.str() << " ";
        cout << propertyDouble->toString() << endl;

        ASSERT(propertyDouble->toString() == ans);
    }

    static void testPropertyOutputHelper(const int& val, const std::string& ans)
    {
        cout << "(int)" << val << ":  ";
        stringstream ss;

        Property<int>* propertyInt =
            Property<int>::TypeHelper::create("int", true);

        propertyInt->setValue(val);
        double valOut = propertyInt->getValue();

        writeUnformatted(ss, valOut);
        cout << std::to_string(valOut) << " " << ss.str() << " ";
        cout << propertyInt->toString() << endl;

        ASSERT(propertyInt->toString() == ans);
    }

    template <int M> static void testPropertyOutputHelper(const SimTK::Vec<M>& val, const std::string& ans)
    {
        cout << "(Vec" << M << ")" << val << ":\n";
        stringstream ss;

        Property<SimTK::Vec<M>>* propertyVec =
            Property<SimTK::Vec<M>>::TypeHelper::create("VecM", true);

        propertyVec->setValue(val);
        Vec<M> valOut = propertyVec->getValue();

        writeUnformatted(ss, valOut);
        cout << ss.str() << "\n";
        cout << propertyVec->toString() << endl;

        ASSERT(propertyVec->toString() == ans);
    }

    static void testPropertyOutputHelper(const SimTK::Vector& val, const std::string& ans)
    {
        cout << "(Vector)" << val << ":\n";
        stringstream ss;

        Property<SimTK::Vector>* propertyVector =
            Property<SimTK::Vector>::TypeHelper::create("Vector", true);

        propertyVector->setValue(val);
        Vector valOut = propertyVector->getValue();

        writeUnformatted(ss, valOut);
        cout << ss.str() << "\n";
        cout << propertyVector->toString() << endl;

        ASSERT(propertyVector->toString() == ans);
    }

    static void testPropertyOutputHelper(const SimTK::Transform& val)
    {
        cout << "(Transform)" << val << ":\n";
        stringstream ss;

        Property<SimTK::Transform>* propertyTransform =
            Property<SimTK::Transform>::TypeHelper::create("Transform", true);

        propertyTransform->setValue(val);
        SimTK::Transform valOut = propertyTransform->getValue();

        writeUnformatted(ss, valOut);
        cout << ss.str() << "\n";
        cout << propertyTransform->toString() << endl;
    }

}

TEST_CASE("Object Serialization")
{
    // Test simple stringstream functionality with SimTK::writeUnformatted
    // and SimTK::readUnformatted
    stringstream ss(" hell there 1.234e5  -infinity");
    stringstream sout;
    while (true) {
        SimTK::String token;
        readUnformatted(ss, token);
        if (ss.fail()) break;
        cout << "'" << token << "'\n";
    }
    ss.clear();
    ss.seekg(0, ios::beg);
    SimTK::Array_<SimTK::String> arrTokens;
    readUnformatted(ss, arrTokens);
    cout << arrTokens << endl;

    writeUnformatted(sout, true);
    sout << " ";
    writeUnformatted(sout, false);
    cout << "'" << sout.str() << "'\n";
    sout.seekg(0, ios::beg);
    while (true) {
        bool res;
        if (!readUnformatted(sout, res))
            break;
        cout << res << "\n";
    }

    // Test Property's toString() and toStringForDisplay() functionality
    cout << "Testing toString() and toStringForDisplay()" << endl;
    cout << "Input:  std::to_string() SimTK::writeUnformatted() Property::toString()" << endl;

    testPropertyOutputHelper((double)0.12345, "0.12345");
    testPropertyOutputHelper((double)0.123456789012345, "0.123457");
    testPropertyOutputHelper((double)0.1, "0.1");
    testPropertyOutputHelper((double)1.0000, "1");
    testPropertyOutputHelper((double)123456789, "1.23457e+08");
    testPropertyOutputHelper((double)0.000000000123456789, "1.23457e-10");
    testPropertyOutputHelper((double)9999999999999999, "1e+16");
    testPropertyOutputHelper((double)1.234e-10, "1.234e-10");
    testPropertyOutputHelper((double)1.234e10, "1.234e+10");
    testPropertyOutputHelper((double)1.23456789e-10, "1.23457e-10");
    testPropertyOutputHelper((double)1.23456789e10, "1.23457e+10");
    testPropertyOutputHelper((double)0, "0");

    testPropertyOutputHelper((int)2, "2");
    testPropertyOutputHelper((int)123456789, "123456789");
    testPropertyOutputHelper((int)0, "0");

    // Don't test std::to_string for Vec, Vector, and Transforms
    cout << "Input:  SimTK::writeUnformatted() Property::toString()" << endl;

    testPropertyOutputHelper(SimTK::Vec3(0, 0, 0), "(0 0 0)");
    testPropertyOutputHelper(SimTK::Vec6(0.123456789, 123456789, 1.23456789e-12, 1.23456, 0.00001234, 1234),
        "(0.123457 1.23457e+08 1.23457e-12 1.23456 1.234e-05 1234)");
    testPropertyOutputHelper(SimTK::Vector(4, -1234567), 
        "(-1.23457e+06 -1.23457e+06 -1.23457e+06 -1.23457e+06)");

    Vec3 p = Vec3(0.1, 0.2, 0.12345678);
    SimTK::Rotation R = SimTK::Rotation(SimTK::Pi, SimTK::XAxis);
    // this does not check against a string because random "-0"s show up in place of "0"s
    testPropertyOutputHelper(SimTK::Transform(R, p));

    Property<double>* propertyDouble =
        Property<double>::TypeHelper::create("double", true);

    // Test precision argument with toStringForDisplay()
    propertyDouble->setValue((double)0.123456789);
    SimTK::Array_<std::string> ans;
    ans.push_back("0.1");
    ans.push_back("0.12");
    ans.push_back("0.123");
    ans.push_back("0.1235");
    ans.push_back("0.12346");
    ans.push_back("0.123457");
    ans.push_back("0.1234568");
    ans.push_back("0.12345679");
    ans.push_back("0.123456789");

    ASSERT_THROW(OpenSim::Exception, propertyDouble->toStringForDisplay(0));

    for (unsigned int i = 0; i < ans.size(); ++i) {
        std::string valStr = propertyDouble->toStringForDisplay(i+1);
        cout << valStr << " " << ans[i] << endl;
        ASSERT(valStr == ans[i]);
    }
    cout << endl;

    // TYPE REGISTRATION
    Object::registerType(SerializableObject());
    Object::registerType(SerializableObject2());
    Object::registerType(SerializableObject3());

    ObjSet objSet;
    const Set<SerializableObject>& baseSet = objSet;

    SimTK_TEST(objSet.getClassName() == "ObjSet");
    // Cannot serialize name containing "<T>" into valid XML code
    // template <T> have been replaced by "_T_" as serialized name
    SimTK_TEST(baseSet.getClassName() == "Set_SerializableObject_");
    SimTK_TEST(baseSet.getConcreteClassName() == "ObjSet");

    // OBJECT 1
    SerializableObject obj1;
    obj1.setName("TestObject");
    obj1.print("obj1.xml");
    SerializableObject obj1copy(obj1);
    obj1copy.setAllPropertiesUseDefault(true);
    obj1copy.set_Test_Bool_2(false);
    obj1copy.print("obj1copy.xml");

    //Xml xx("obj1.xml");
    //cerr << xx;

    // OBJECT 2
    SerializableObject obj2("obj1.xml");
    obj2.print("roundtrip.xml");

    // OBJECT 3
    SerializableObject obj3("obj1Defaults.xml");
    //Property_Deprecated* pObjArr = obj3.getPropertySet().get(11);
    //PropertyObjArray<OpenSim::Object>* objs = (PropertyObjArray<OpenSim::Object> *)pObjArr;
    //Object* dObj = objs->getValueObjPtr(1);
    //Property_Deprecated* p1 = dObj->getPropertySet().get(0);
    //Property_Deprecated* p2 = dObj->getPropertySet().get(1);
    //std::cout << (*p1) << std::endl;
    //std::cout << (*p2) << std::endl;
    Object::setSerializeAllDefaults(true);
    obj3.print("roundtripDefaults.xml");

    // Now compare object properties to make sure we're not reading and writing the file as just text!
    int numProperties1 = obj1.getPropertySet().getSize();
    ASSERT(numProperties1 == obj2.getPropertySet().getSize(), __FILE__, __LINE__, "num properties");

    ASSERT(obj1 == obj2, __FILE__, __LINE__, "equality");

    PropertySet &propSet1 = obj1.getPropertySet();
    PropertySet &propSet2 = obj2.getPropertySet();
    for (int i=0; i < numProperties1; i++){
        Property_Deprecated *prop1 = propSet1.get(i);
        Property_Deprecated *prop2 = propSet2.get(i);
        ASSERT(prop1->getName() == prop2->getName(), __FILE__, __LINE__, "property names");
    }

    ASSERT(((PropertyBool*) propSet1.get(0))->getValueBool() == ((PropertyBool*) propSet2.get(0))->getValueBool(), __FILE__, __LINE__, "bool property");

    ASSERT(((PropertyInt*) propSet1.get(1))->getValueInt() == ((PropertyInt*) propSet2.get(1))->getValueInt(), __FILE__, __LINE__, "int property");

    ASSERT(((PropertyDbl*) propSet1.get(2))->getValueDbl() == ((PropertyDbl*) propSet2.get(2))->getValueDbl(), __FILE__, __LINE__, "double property");

    /* The following actually fails due to extra spaces when we read back from file!.*/
    string& str1 = ((PropertyStr*) propSet1.get(6))->getValueStr();
    string& str2 = ((PropertyStr*) propSet2.get(6))->getValueStr();
    int cmp=str1.compare(str2);
    if (cmp!=0) {
        throw OpenSim::Exception("String property",__FILE__,__LINE__);
    }

    for (int i=0; i < obj1.getNumProperties(); ++i) {
        const AbstractProperty& ap = obj1.getPropertyByIndex(i);
        std::cout << ap.getName() << "=" << ap.toString() << std::endl;
    }

    // Should be able to find new and deprecated properties by name.
    SimTK_TEST(obj1.hasProperty("Test_Obj")); // deprecated
    SimTK_TEST(obj1.hasProperty("Test_Obj_2")); // new
    SimTK_TEST(!obj1.hasProperty("No_Such_Property"));

    // Now check new property system's handing of nameless, one-object
    // property (of type T=SerializableObject3).

    SimTK_TEST(obj1.hasProperty<SerializableObject3>());
    SimTK_TEST(obj1.hasProperty("SerializableObject3"));

    // Check for correct object return type.
    SimTK_TEST(obj1.getProperty_SerializableObject3()[0]
        .getConcreteClassName() == "SerializableObject3");

    cout << "\n------------------------------------------" << endl;
    cout << "DUMPOBJ(obj1)" << endl;
    dumpObj(obj1, 0);

    obj1.updProperty_Test_Str_2() = "DID THIS GET COPIED??";

    SerializableObject copyOfObj1(obj1);
    SerializableObject assignOfObj1;
    assignOfObj1 = obj1;

    cout << "\n------------------------------------------" << endl;
    cout << "DUMPOBJ(copyOfObj1)" << endl;
    dumpObj(copyOfObj1, 0);
    cout << "\n------------------------------------------" << endl;
    cout << "DUMPOBJ(assignOfObj1)" << endl;
    dumpObj(assignOfObj1, 0);

    ObjectWithListProperty objWithListProp;
    SerializableObject obj_l1;
    obj_l1.setName("First");
    SerializableObject obj_l2;
    obj_l2.setName("Second");
    objWithListProp.append_list_SerializableObject(obj_l1);
    objWithListProp.append_list_SerializableObject(obj_l2);
    int loc = objWithListProp.getProperty_list_SerializableObject().findIndexForName("Second");
    ASSERT(loc == 1);
    int notFound = objWithListProp.getProperty_list_SerializableObject().findIndexForName("Third");
    ASSERT(notFound == -1);
    SimTK_TEST_MUST_THROW(SerializableObject bad("obj1Bad.xml"));
}
