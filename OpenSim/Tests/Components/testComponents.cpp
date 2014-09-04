/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testComponents.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Ajay Seth                                         *
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

#include <stdint.h>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Auxiliary/getRSS.h>

using namespace OpenSim;
using namespace std;

static Model dummyModel;
const double acceptableMemoryLeakPercent = 2.0;
const bool reportAllMemoryLeaks = true;

template <typename T>
void testComponent(const T& instanceToTest);

// NOTE: disabling randomizePropertyValues weakens the test substantially.
template <typename T>
void testModelComponent(const T& instanceToTest,
        bool randomizePropertyValues=true,
        Model& model=dummyModel);

int main()
{
    // Do not delete this line. It is used to allow users to optionally pass in their own model.
    dummyModel.setName("dummyModel");

    // Add a line here for each model component that we want to test.
    testModelComponent(ClutchedPathSpring());
    testModelComponent(Thelen2003Muscle(), false);
    testModelComponent(Millard2012EquilibriumMuscle(), false);
    //testModelComponent(Millard2012AccelerationMuscle(), false);
    // TODO randomized properties out of range; throws exception.
    // TODO testModelComponent(PathActuator());

    {
        ContactSphere contactSphere; contactSphere.set_body_name("ground");
        testModelComponent(contactSphere);
    }

    /*{ TODO memory leak with initSystem.
        Body* body1 = new Body(); body1->setName("body1"); body1->setMass(1.0);
        PinJoint pinJoint;
        pinJoint.updConnector<Body>("parent_body").
            set_connected_to_name("ground");
        pinJoint.updConnector<Body>("child_body").
            set_connected_to_name("body1");
        //TODO Model model; model.addBody(body1);
        Model model("gait10dof18musc_subject01.osim"); model.addBody(body1);
        testModelComponent(pinJoint, true, model);
    }*/

    testModelComponent(Bhargava2004MuscleMetabolicsProbe());
    testModelComponent(Umberger2010MuscleMetabolicsProbe());
}

class DummyComponent : public Component {
	OpenSim_DECLARE_CONCRETE_OBJECT(DummyComponent, Component);
};


template <typename T>
void testComponent(const T& instanceToTest)
{
    // TODO
    throw Exception("Not implemented.");
}

template <typename T>
void testModelComponent(const T& instanceToTest, bool randomizePropertyValues,
        Model& model)
{
    // Make a copy so that we can modify the instance.
    T* instance = new T(instanceToTest);
    string className = instance->getConcreteClassName();

    std::cout << "\nTesting " << className << std::endl;

    // 1. Set properties to random values.
    // -----------------------------------
    if (randomizePropertyValues)
    {
        std::cout << "Randomizing the component's properties." << std::endl;
        randomize(instance); 
    }

    // 2. Ensure that cloning produces an exact copy.
    // ----------------------------------------------
    // This will find missing calls to copyProperty_<name>().
    std::cout << "Cloning the component." << std::endl;
    T* copyInstance = instance->clone();
    if (!(*copyInstance == *instance))
    {
        std::cout << "XML serialization for the first instance:" << std::endl;
        std::cout << instance->dump() << std::endl;
        std::cout << "XML serialization for the clone:" << std::endl;
        std::cout << copyInstance->dump() << std::endl;
        throw Exception(
                "testComponents: for " + className +
                ", clone() did not produce an identical object.",
                __FILE__, __LINE__);
    }
    // TODO should try to delete even if exception is thrown.
    delete copyInstance;
    
    // 3. Serialize and de-serialize.
    // ------------------------------
    // This will find issues with serialization.
    std::cout << "Serializing and deserializing component." << std::endl;
    string serializationFilename =
        "testing_serialization_" + className + ".xml";
    instance->print(serializationFilename);
    T* deserializedInstance =
        static_cast<T*>(Object::makeObjectFromFile(serializationFilename));
    if (!(*deserializedInstance == *instance))
    {
        std::cout << "XML for serialized instance:" << std::endl;
        instance->dump();
        std::cout << "XML for seriaization of deseralized instance:" <<
            std::endl;
        deserializedInstance->dump();
        throw Exception(
                "testComponents: for " + className +
                ", deserialization did not produce an identical object.",
                __FILE__, __LINE__);
    }
    // TODO should try to delete even if exception is thrown.
    delete deserializedInstance;

    // 4. Set up the aggregate component.
    // -------------------------------------------------------------------
    std::cout << "Set up aggregate component." << std::endl;
    if (model.getName() == "dummyModel")
    {
        // User did not provide a model; create a fresh model.
        model = Model();
    }

    // 5. Add this component to an aggregate component.
    // ------------------------------------------------
    std::cout << "Add this ModelComponent to the model." << std::endl;
    if (Object::isObjectTypeDerivedFrom< Analysis >(className))
        model.addAnalysis(dynamic_cast<Analysis*>(instance));
    else if (Object::isObjectTypeDerivedFrom< Body >(className))
        model.addBody(dynamic_cast<Body*>(instance));
    else if (Object::isObjectTypeDerivedFrom< Constraint >(className))
        model.addConstraint(dynamic_cast<Constraint*>(instance));
    else if (Object::isObjectTypeDerivedFrom< ContactGeometry >(className))
        model.addContactGeometry(dynamic_cast<ContactGeometry*>(instance));
    else if (Object::isObjectTypeDerivedFrom< Controller >(className))
        model.addController(dynamic_cast<Controller*>(instance));
    else if (Object::isObjectTypeDerivedFrom< Force >(className))
        model.addForce(dynamic_cast<Force*>(instance));
    else if (Object::isObjectTypeDerivedFrom< Probe >(className))
        model.addProbe(dynamic_cast<Probe*>(instance));
    else if (Object::isObjectTypeDerivedFrom< Joint >(className))
        model.addJoint(dynamic_cast<Joint*>(instance));
    else if (Object::isObjectTypeDerivedFrom< ModelComponent >(className))
        model.addModelComponent(dynamic_cast<ModelComponent*>(instance));
    else
    {
        throw Exception(className + " is not a ModelComponent.",
                __FILE__, __LINE__);
    }

    // 6. Connect up the aggregate; check that connections are correct.
    // ----------------------------------------------------------------
    // This method calls connect().
    std::cout << "Call Model::setup()." << std::endl;
    model.setup();

    // 7. Build the system.
    // --------------------
    SimTK::State& initState = model.initSystem();

    // Outputs.
    // --------
    std::cout << "Testing Output's." << std::endl;
    for (auto it = instance->getOutputsBegin();
            it != instance->getOutputsEnd(); ++it)
    {
        const std::string thisName = it->first;
        const AbstractOutput* thisOutput = it->second.get();

        std::cout << "Testing Output " << thisName << ", dependent on " <<
            thisOutput->getDependsOnStage().getName() << std::endl;

        // Start fresh.
        SimTK::State state(initState);

        // 8. Check that each output throws an exception if we're below its
        // dependsOnStage. Model::initSystem() gives us a state that is already
        // realized to Model.
        if (thisOutput->getDependsOnStage() > SimTK::Stage::Model)
        {
            model.getSystem().realize(state,
                    thisOutput->getDependsOnStage().prev());
            ASSERT_THROW(Exception,
                    thisOutput->getValueAsString(state);
            );
        }

        // 9. Now realize to the dependsOnStage.
        // Doesn't matter what the value is; just want to make sure the output
        // is wired.
        model.getSystem().realize(state, thisOutput->getDependsOnStage());
        std::cout << "Component " << className <<
            ", output " <<thisName << ": " <<
            thisOutput->getValueAsString(state) << std::endl;
    }

    // 10. Test for memory leaks by copying and deleting.
    // --------------------------------------------------
    std::cout << "Testing for memory leaks from copying." << std::endl;
    {
        const size_t initMemory = getCurrentRSS();
        for (unsigned int ileak = 0; ileak < 1000; ++ileak)
        {
            T* copy = new T(*instance);
            delete copy;
        }
        const int64_t increaseInMemory = getCurrentRSS() - initMemory;
        const long double leakPercent = 100.0 * increaseInMemory / initMemory;

        ASSERT(leakPercent < acceptableMemoryLeakPercent, __FILE__, __LINE__,
                "testComponents: memory leak greater than " +
                to_string(acceptableMemoryLeakPercent) + "%. Initial memory: " +
                to_string(initMemory/1024) + " KB, increase in memory: " +
                to_string(increaseInMemory/1024) + " KB, " +
                to_string(leakPercent) + "%.");

        if (reportAllMemoryLeaks && increaseInMemory>0)
            std::cout << "\t[" << className
                      << "] copying increased memory use by "
                      << setprecision(3) << leakPercent << "%" << std::endl;
    }

    // 11. Test that repeated calls to initSystem do not change test results,
    // and that memory does not leak.
    // ------------------------------------------------------------------------
    std::cout << "Testing for memory leaks from initSystem." << std::endl;
    {
        SimTK::State& finalInitState(initState);
        const size_t initMemory = getCurrentRSS();
        for (unsigned int ileak = 0; ileak < 500; ++ileak)
        {
            finalInitState = model.initSystem();
        }
        const int64_t increaseInMemory = getCurrentRSS() - initMemory;
        const long double leakPercent = 100.0 * increaseInMemory / initMemory;

        ASSERT_EQUAL(0.0,
                (finalInitState.getY() - initState.getY()).norm(),
                1e-7, __FILE__, __LINE__, "testComponents: initial state "
                    "differs after repeated calls to initSystem().");

        ASSERT(leakPercent < acceptableMemoryLeakPercent, __FILE__, __LINE__,
                "testComponents: memory leak greater than " +
                to_string(acceptableMemoryLeakPercent) + "%. Initial memory: " +
                to_string(initMemory/1024) + " KB, increase in memory: " +
                to_string(increaseInMemory/1024) + " KB, " +
                to_string(leakPercent) + "%.");

        if (reportAllMemoryLeaks && increaseInMemory>0)
            std::cout << "\t[" << className
                      << "] initSystem increased memory use by "
                      << setprecision(3) << leakPercent << "%" << std::endl;
    }
}
