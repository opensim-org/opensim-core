/* -------------------------------------------------------------------------- *
 *                 OpenSim:  testSerializeOpenSimObjects.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Analyses/osimAnalyses.h>

using namespace OpenSim;
using namespace std;

void testPropertiesDump(const OpenSim::Object& aObject);

static void indent(int nSpaces) {
    for (int i=0; i<nSpaces; ++i) cout << " ";
}

// Recursively dump out contents of an object and its properties.
static void dumpObj(const Object& obj, int nSpaces) {
    indent(nSpaces);
    cout << obj.getConcreteClassName() << " Object " 
         << (obj.getName().empty()?"NONAME":obj.getName())
         << endl;
    for (int p=0; p < obj.getNumProperties(); ++p) {
        const AbstractProperty& ap = obj.getPropertyByIndex(p); 
        indent(nSpaces+2);
        cout << ap.getName() << "=" << ap.toString() << endl;
        // Check return values from Property API for debugging purposes
        bool t1 = ap.isListProperty();
        bool t2 = ap.isObjectProperty();
        bool t3 = ap.isOneObjectProperty();
        bool t4 = ap.isOneValueProperty();
        string ts = ap.getTypeName();
        indent(nSpaces+2);
        cout << "isList, isObject, isOneObject, isOneValue, typeName = " <<
            t1 <<", "<< t2 <<", "<< t3 <<", "<< t4 <<", "<< ts << endl;
        if (ap.isObjectProperty()) {
            for (int i=0; i < ap.size(); ++i)
                dumpObj(ap.getValueAsObject(i), nSpaces+4);
        }
    }
}


int main()
{
    // Need to force Windows to load Actuators library if there are no
    // explicit uses.
    LoadOpenSimLibrary("osimActuators");

    try {
        Model testModel;
        srand((unsigned)time(0));

        //Test serialization for all ModelComponents
        ArrayPtrs<OpenSim::ModelComponent> availableComponentTypes;
        Object::getRegisteredObjectsOfGivenType<OpenSim::ModelComponent>(availableComponentTypes);
        for (int i=0; i< availableComponentTypes.getSize(); i++){
            Object* clone = availableComponentTypes[i]->clone();
            Object* randClone = randomize(clone);
            try {
                testModel.addModelComponent(ModelComponent::safeDownCast(randClone));
            } //Ignore the validity of the property values
            // TODO this should specifically handle "InvalidPropertyValue" exceptions
            // once we have that in place.
            catch (const std::exception&) {
                // const string& errMsg = err.getMessage();
                //std::cout << errMsg << std::endl;
            }
        }

        int nc = testModel.getMiscModelComponentSet().getSize();
        cout << nc << " model components were serialized in testModel." << endl;

        //Serialize all the components
        testModel.print("allComponents.osim");

        // The finalize flag is for testing purposes ONLY. This way we
        // can ignore invalid properties and focus the test on serialization.
        Model deserializedModel("allComponents.osim", false);

        try {
            deserializedModel.finalizeFromProperties();
        }
        //Ignore the validity of the property values
        // TODO this should specifically handle "InvalidPropertyValue" exceptions
        // once we have that in place.
        catch (const std::exception&) {
            // const string& errMsg = err.getMessage();
            //std::cout << errMsg << std::endl;
        }

        nc = deserializedModel.getMiscModelComponentSet().getSize();
        cout << nc << " model components were deserialized from file." << endl;

        ASSERT(testModel == deserializedModel,  
            "deserializedModel FAILED to match original model.");

        //Might as well test cloning and assignment
        Model* cloneModel = testModel.clone();

        ASSERT(testModel == *cloneModel,
            "cloneModel FAILED to match original model.");

        Model assignedModel = *cloneModel;

        delete cloneModel;

        ASSERT(testModel == assignedModel,
            "assignedModel FAILED to match original model.");

    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void testPropertiesDump(const Object& aObject)
{
    dumpObj(aObject, 4);
}
