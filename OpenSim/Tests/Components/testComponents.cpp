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

template <typename T>
void testComponent(T& instance);

template <typename T>
void testModelComponent(T& instance);


int main()
{
    // Add a line here for each model component that we want to test.
    testModelComponent(ClutchedPathSpring());
}

class DummyComponent : public Component {
	OpenSim_DECLARE_CONCRETE_OBJECT(DummyComponent, Component);
};

template <typename T>
void testComponent(T& instance)
{
    // TODO
    throw Exception("Not implemented.");
}

template <typename T>
void testModelComponent(T& instance)
{
    // TODO instead of using a template method, can also use methods like
    // newInstanceOfType().
   
    std::cout << "Testing " << instance.getConcreteClassName() << std::endl;

    // 1. Set properties to random values.
    for (unsigned int iprop = 0;
            iprop < instance.getNumProperties();
            ++iprop)
    {
        AbstractProperty& prop = instance.updPropertyByIndex(iprop);
        // TODO
    }
    

    // 2. Ensure that cloning produces an exact copy. This will find missing
    // calls to copyProperty_<name>().
    T* copyInstance = instance.clone();
    ASSERT(&copyInstance == &instance, __FILE__, __LINE__,
            "testComponents: for " + instance.getConcreteClassName() +
            ", clone() did not produce an identical object.");
    delete copyInstance;
    
    // 3. Serialize and de-serialize. This will find any issues with the
    // serialization.
    instance.print(
            "testing_serialization_" + instance.getConcreteClassName() +
            ".xml");

    // 4. Make the aggregate component. Add in dependent components to the
    // aggregate, by looking at connectors.
    Model model; // TODO "gait10dof18musc_subject01.osim");
    // TODO

    // 5. Add this component to an aggregate component.
    model.addModelComponent(instance);

    // 6. Connect up the aggregate; check that connections are correct.
    // This method calls connect().
    model.setup();

    // 7. Build the system.
    SimTK::State& initState = model.initSystem();

    // Outputs.
    std::map<std::string, const AbstractOutput*>::const_iterator it;
    for (it = instance.getOutputsBegin();
            it != instance.getOutputsEnd(); ++it)
    {
        const std::string thisName = it->first;
        const AbstractOutput* thisOutput = it->second;

        // Start fresh.
        SimTK::State state(initState);

        // 8. Check that each output throws an exception if we're below its
        // dependsOnStage.
        model.getSystem().realize(state,
                thisOutput->getDependsOnStage().prev());
        ASSERT_THROW(Exception,
                thisOutput->getValueAsString(state);
        );

        // 9. Now realize to the dependsOnStage.
        // Doesn't matter what the value is; just want to make sure the output
        // is wired.
        model.getSystem().realize(state, thisOutput->getDependsOnStage());
        std::cout << "Component " << instance.getConcreteClassName() <<
            ", output " <<thisName << ": " <<
            thisOutput->getValueAsString(state) << std::endl;
    }

    // 10. Test for memory leaks by copying and deleting.
    {
        size_t initMemory = getCurrentRSS();
        for (unsigned int ileak = 0; ileak < 1000; ++ileak)
        {
            T* copy = new T(&instance);
            delete copy;
        }
        int64_t increaseInMemory = getCurrentRSS() - initMemory;
        long double leakPercent = 100.0 * increaseInMemory / initMemory;

        ASSERT(leakPercent < 2.0, __FILE__, __LINE__,
                "testComponents: memory leak greater than 2%. Initial memory: " +
                to_string(initMemory) + ", increase in memory: " +
                to_string(increaseInMemory) + ".");
    }

    // 11. Test that repeated calls to initSystem does not change test results,
    // and that memory does not leak.
    {
        SimTK::State& finalInitState(initState);
        size_t initMemory = getCurrentRSS();
        for (unsigned int ileak = 0; ileak < 1000; ++ileak)
        {
            finalInitState = model.initSystem();
        }
        int64_t increaseInMemory = getCurrentRSS() - initMemory;
        long double leakPercent = 100.0 * increaseInMemory / initMemory;

;
        ASSERT_EQUAL(0.0,
                (finalInitState.getY() - initState.getY()).norm(),
                1e-7, __FILE__, __LINE__, "testComponents: initial state "
                    "differs after repeated calls to initSystem().");

        ASSERT(leakPercent < 2.0, __FILE__, __LINE__,
                "testComponents: memory leak greater than 2%. Initial memory: " +
                to_string(initMemory) + ", increase in memory: " +
                to_string(increaseInMemory) + ".");
    }

}







