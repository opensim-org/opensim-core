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

#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertySet.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "SimTKcommon.h"

#include <iostream>
#include <string>

#include "SerializableObject.h"
#include "SerializableObject2.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

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

int main()
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
    

    try {
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
    catch(const std::exception& e) {
        cerr << "EXCEPTION: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
