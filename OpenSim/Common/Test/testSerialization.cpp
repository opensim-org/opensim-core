// testTools.cpp
// Author:  Frank C. Anderson
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
#include <iostream>
#include <string>
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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "rdSerializableObject.h"
#include "rdSerializableObject2.h"
#include "SimTKcommon.h"

using namespace OpenSim;
using namespace std;

int main()
{
	try {
		// TYPE REGISTRATION
		Object::RegisterType(rdSerializableObject());
		Object::RegisterType(rdSerializableObject2());

		// OBJECT 1
		rdSerializableObject obj1;
		obj1.setName("TestObject");
		obj1.print("obj1.xml");

		// OBJECT 2
		rdSerializableObject obj2("obj1.xml");
		obj2.print("roundtrip.xml");

		// OBJECT 3
		rdSerializableObject obj3("obj1Defaults.xml");
		Property* pObjArr = obj3.getPropertySet().get(11);
		PropertyObjArray<OpenSim::Object>* objs = (PropertyObjArray<OpenSim::Object> *)pObjArr;
		Object* dObj = objs->getValueObjPtr(1);
		Property* p1 = dObj->getPropertySet().get(0);
		Property* p2 = dObj->getPropertySet().get(1);
		std::cout << (*p1) << std::endl;
		std::cout << (*p2) << std::endl;
		Object::setSerializeAllDefaults(true);
		obj3.print("roundtripDefaults.xml");

		// Now compare object properties to make sure we're not reading and writing the file as just text!
		int numProperties1 = obj1.getPropertySet().getSize();
		ASSERT(numProperties1 == obj2.getPropertySet().getSize(), __FILE__, __LINE__, "num properties");

		PropertySet &propSet1 = obj1.getPropertySet();
		PropertySet &propSet2 = obj2.getPropertySet();
		for (int i=0; i < numProperties1; i++){
			Property *prop1 = propSet1.get(i);
			Property *prop2 = propSet2.get(i);
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
			throw Exception("String property",__FILE__,__LINE__);
		}
		
	}
    catch(const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}