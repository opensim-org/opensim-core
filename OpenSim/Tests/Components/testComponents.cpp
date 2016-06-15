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
const double acceptableMemoryLeakPercent = 1.0;
const bool reportAllMemoryLeaks = true;

void testComponent(const Component& instanceToTest);
void testCloning(Component* instance);
void testSerialization(Component* instance);

void addObjectAsComponentToModel(Object* instance, Model& model);

int main()
{
    SimTK::Array_<std::string> failures;

    // get all registered Components
    SimTK::Array_<Component*> availableComponents;

    // starting with type Frame
    ArrayPtrs<Frame> availableFrames;
    Object::getRegisteredObjectsOfGivenType(availableFrames);
    for (int i = 0; i < availableFrames.size(); ++i) {
        availableComponents.push_back(availableFrames[i]);
    }

    // next with type Point
    ArrayPtrs<Point> availablePoints;
    Object::getRegisteredObjectsOfGivenType(availablePoints);
    for (int i = 0; i < availablePoints.size(); ++i) {
        availableComponents.push_back(availablePoints[i]);
    }

    // then type Joint
    ArrayPtrs<Joint> availableJoints;
    Object::getRegisteredObjectsOfGivenType(availableJoints);
    for (int i = 0; i < availableJoints.size(); ++i) {
        availableComponents.push_back(availableJoints[i]);
    }

    // then type TwoFrameLinker<Constraint>
    ArrayPtrs<TwoFrameLinker<Constraint, PhysicalFrame> > availableLink2Constraints;
    Object::getRegisteredObjectsOfGivenType(availableLink2Constraints);
    for (int i = 0; i < availableLink2Constraints.size(); ++i) {
        availableComponents.push_back(availableLink2Constraints[i]);
    }

    // then type TwoFrameLinker<Force> which are all the BushingForces
    ArrayPtrs<TwoFrameLinker<Force, PhysicalFrame> > availableBushingForces;
    Object::getRegisteredObjectsOfGivenType(availableBushingForces);
    for (int i = 0; i < availableBushingForces.size(); ++i) {
        availableComponents.push_back(availableBushingForces[i]);
    }

    // Test PrescribedForce
    std::unique_ptr<PrescribedForce> f(new PrescribedForce());
    availableComponents.push_back(f.get());
    // continue with other Constraints, Forces, Actuators, ...
    //Examples of updated forces that pass
    ArrayPtrs<PointToPointSpring> availablePointToPointSpring;
    Object::getRegisteredObjectsOfGivenType(availablePointToPointSpring);
    availableComponents.push_back(availablePointToPointSpring[0]);

    /** //Uncomment when dependencies of CoordinateCouplerConstraints are 
    // specified as Connectors 
    ArrayPtrs<Constraint> availableConstraints;
    Object::getRegisteredObjectsOfGivenType(availableConstraints);
    for (int i = 0; i < availableConstraints.size(); ++i) {
        availableComponents.push_back(availableConstraints[i]);
    }
    */

    for (unsigned int i = 0; i < availableComponents.size(); i++) {
        try {
            testComponent(*availableComponents[i]);
        }
        catch (const std::exception& e) {
            cout << "*******************************************************\n";
            cout<< "FAILURE: " << availableComponents[i]->getConcreteClassName() << endl;
            cout<< e.what() << endl;
            failures.push_back(availableComponents[i]->getConcreteClassName());
        }
    }
    
    if (!failures.empty()) {
        cout << "*******************************************************\n";
        cout << "Done, with failure(s): " << failures << endl;
        cout << failures.size() << "/" << availableComponents.size() 
            << " components failed test." << endl;
        cout << 100 * (availableComponents.size() - failures.size()) / availableComponents.size()
            << "% components passed." << endl;
        cout << "*******************************************************\n" << endl;
        return 1;
    }
    cout << "\ntestComponents PASSED. " << availableComponents.size() 
        << " components were tested." << endl;
}


//template <typename T>
void testComponent(const Component& instanceToTest)
{
    // Empty model used to serve as the aggregate Component
    Model model;
    model.setName("TheModel");

    // Make a copy so that we can modify the instance.
    Component* instance = instanceToTest.clone();
    const string& className = instance->getConcreteClassName();

    cout << "\n**********************************************************\n";
    cout << "* Testing " << className << endl;
    cout << "**********************************************************" << endl;

    // 1. Set properties to random values.
    // -----------------------------------
    cout << "Randomizing the component's properties." << endl;
    randomize(instance);

    // 2. Ensure that cloning produces an exact copy.
    // ----------------------------------------------
    // This will find missing calls to copyProperty_<name>().
    testCloning(instance);

    // 3. Serialize and de-serialize.
    // ------------------------------
    // This will find issues with de/serialization.
    cout << "Serializing and deserializing component." << endl;
    testSerialization(instance);

    const size_t instanceSize = getCurrentRSS();

    // 4. Verify Components structural attributes

    // -------------------------------------------------------------------
    cout << "Set up aggregate component." << endl;
    // 5. Add this component to an aggregate component.
    // ------------------------------------------------
    addObjectAsComponentToModel(instance, model);


    // 6. Connect up the aggregate; check that connections are correct.
    // ----------------------------------------------------------------
    // First make sure Connectors are satisfied.
    Component* sub = instance;
    ComponentList<Component> comps = instance->getComponentList<Component>();
    ComponentList<Component>::const_iterator it = comps.begin();

    while(sub) {
        int nc = sub->getNumConnectors();
        for (int i = 0; i < nc; ++i){
            AbstractConnector& connector = sub->updConnector(i);
            string dependencyTypeName = connector.getConnecteeTypeName();
            cout << "Connector '" << connector.getName() <<
                "' has dependency on: " << dependencyTypeName << endl;
            Object* dependency =
                Object::newInstanceOfType(dependencyTypeName);

            if (dependency == nullptr){
                // Get a concrete instance of a PhysicalFrame, which is a Body
                if (dependencyTypeName == "PhysicalFrame"){
                    dependency = Object::newInstanceOfType("Body");
                }
            }

            if (dependency) {
                //give it some random values including a name
                randomize(dependency);
                connector.setConnecteeName(dependency->getName());

                // add the dependency 
                addObjectAsComponentToModel(dependency, model);
            }
        }
        const Component& next = *it;
        //Now keep checking the subcomponents
        sub = const_cast<Component *>(&next);
        it++;
    }

    // This method calls connect().
    cout << "Call Model::setup()." << endl;
    try{
        model.setup();
    }
    catch (const std::exception &x) {
        cout << "testComponents::" << className << " unable to connect to model:" << endl;
        cout << " '" << x.what() << "'" <<endl;
        cout << "Error is likely due to " << className;
        cout << " having structural dependencies that are not specified as Connectors.";
        cout << endl;
    }


    // 7. Build the system.
    // --------------------
    SimTK::State initState;
    try{
        initState = model.initSystem();
    }
    catch (const std::exception &x) {
        cout << "testComponents::" << className << " unable to initialize the system:" << endl;
        cout << " '" << x.what() << "'" << endl;
        cout << "Skipping ... " << endl;
    }

    // Outputs.
    // --------
    cout << "Invoking Output's." << endl;
    for (auto it = instance->getOutputsBegin();
            it != instance->getOutputsEnd(); ++it)
    {
        const std::string thisName = it->first;
        const AbstractOutput* thisOutput = it->second.get();

        cout << "Testing Output " << thisName << ", dependent on " <<
            thisOutput->getDependsOnStage().getName() << endl;

        // Start fresh.
        SimTK::State state(initState);

        // 8. Check that each output throws an exception if we're below its
        // dependsOnStage. Model::initSystem() gives us a state that is already
        // realized to Model.
        if (thisOutput->getDependsOnStage() > SimTK::Stage::Model)
        {
            model.getSystem().realize(state,
                    thisOutput->getDependsOnStage().prev());
            ASSERT_THROW(SimTK::Exception::StageTooLow,
                    thisOutput->getValueAsString(state);
            );
        }

        // 9. Now realize to the dependsOnStage.
        // Doesn't matter what the value is; just want to make sure the output
        // is wired.
        model.getSystem().realize(state, thisOutput->getDependsOnStage());
        cout << "Component " << className <<", output " <<thisName << ": " <<
            thisOutput->getValueAsString(state) << endl;
    }

    // 10. Test for memory leaks by copying and deleting.
    // --------------------------------------------------
    cout << "Testing for memory leaks from copying." << endl;
    {
        unsigned int nCopies = 100;
        const size_t initMemory = getCurrentRSS();

        for (unsigned int ileak = 0; ileak < nCopies; ++ileak)
        {
            Component* copy = instance->clone();
            delete copy;
        }
        const size_t increaseInMemory = getCurrentRSS() - initMemory;
        const long double leakPercent = (100.0*increaseInMemory/instanceSize)/nCopies;

        stringstream msg;
        msg << className << ".clone() increased memory use by "
            << setprecision(3) << leakPercent << "%";

        ASSERT(leakPercent < acceptableMemoryLeakPercent, __FILE__, __LINE__,
            msg.str() + "exceeds acceptable tolerance (" + 
            to_string(acceptableMemoryLeakPercent) + ").\n Instance size: " +
            to_string(instanceSize / 1024) + "KB increased by " +
            to_string(increaseInMemory / 1024) + "KB over " + to_string(nCopies) +
            " iterations = " + to_string(leakPercent) + "%.\n"); // << endl;

        if (reportAllMemoryLeaks && increaseInMemory>0)
            cout << msg.str()  << endl;
    }

    // 11. Test that repeated calls to initSystem do not change test results,
    // and that memory does not leak.
    // ------------------------------------------------------------------------
    cout << "Testing for memory leaks from initSystem." << endl;
    {
        unsigned int nLoops = 100;
        SimTK::State& finalInitState(initState);
        const size_t initMemory = getCurrentRSS();

        for (unsigned int ileak = 0; ileak < nLoops; ++ileak)
        {
            finalInitState = model.initSystem();
        }
        const size_t increaseInMemory = getCurrentRSS() - initMemory;
        const long double leakPercent = (100.0*increaseInMemory/instanceSize)/nLoops;

        ASSERT_EQUAL(0.0,
                (finalInitState.getY() - initState.getY()).norm(),
                1e-7, __FILE__, __LINE__, "testComponents: " + 
                instanceToTest.getConcreteClassName() + " initial state " +
                "differs after repeated calls to initSystem().");

        stringstream msg;
        msg << className << ".initSystem() increased memory use by "
            << setprecision(3) << leakPercent << "%.";

        ASSERT(leakPercent < acceptableMemoryLeakPercent, __FILE__, __LINE__,
            msg.str() + "\nExceeds acceptable tolerance of " +
            to_string(acceptableMemoryLeakPercent) + "%.\n Instance size: " +
            to_string(instanceSize / 1024) + "KB increased by " +
            to_string(increaseInMemory / 1024) + "KB over " + to_string(nLoops) +
            " iterations = " + to_string(leakPercent) + "%.\n"); // << endl;

        if (reportAllMemoryLeaks && increaseInMemory>0)
            cout << msg.str() << endl;
    }
}

void testComponentEquivalence(const Component* a, const Component* b) 
{
    const string& className = a->getConcreteClassName();

    bool same = *a == *b;
    ASSERT(same, __FILE__, __LINE__,
        className + " components are not equivalent in properties.");

    int nc_a = a->getNumConnectors();
    int nc_b = b->getNumConnectors();
    cout << className << " getNumConnectors: " << nc_a << endl;
    ASSERT(nc_a==nc_b, __FILE__, __LINE__, 
        className + "components differ in number of connectors.");

    int nin_a = a->getNumInputs();
    int nin_b = b->getNumInputs();
    cout << className << " getNumInputs: " << nin_a << endl;
    ASSERT(nin_a == nin_b, __FILE__, __LINE__,
        className + " components differ in number of inputs.");

    int nout_a = a->getNumOutputs();
    int nout_b = b->getNumOutputs();
    cout << className << " getNumOutputs: " << nout_a << endl;
    ASSERT(nout_a == nout_b, __FILE__, __LINE__, 
        className + " components differ in number of outputs.");

    ComponentList<Component> aSubsList = a->getComponentList<Component>();
    ComponentList<Component> bSubsList = a->getComponentList<Component>();
    auto iter_a = aSubsList.begin();
    auto iter_b = bSubsList.begin();

    //Subcomponents must be equivalent too!
    while (iter_a != aSubsList.end() && iter_b != aSubsList.end()) {
        const Component& asub = *iter_a;
        const Component& bsub = *iter_b;
        testComponentEquivalence(&asub, &bsub);
        ++iter_a;
        ++iter_b;
    }
}

void testCloning(Component* instance)
{
    cout << "Cloning the component." << endl;
    Component* copyInstance = instance->clone();
    if (!(*copyInstance == *instance))
    {
        cout << "XML serialization for the first instance:" << endl;
        cout << instance->dump() << endl;
        cout << "XML serialization for the clone:" << endl;
        cout << copyInstance->dump() << endl;
        const string& className = instance->getConcreteClassName();
        throw Exception(
            "testComponents: for " + className +
            ", clone() did not produce an identical object.",
            __FILE__, __LINE__);
    }

    instance->finalizeFromProperties();
    copyInstance->finalizeFromProperties();

    testComponentEquivalence(instance, copyInstance);
    delete copyInstance;
}

void testSerialization(Component* instance)
{
    const string& className = instance->getConcreteClassName();
    string serializationFilename =
        "testing_serialization_" + className + ".xml";
    instance->print(serializationFilename);

    Object* deserializedInstance =
        static_cast<Object*>(Object::makeObjectFromFile(serializationFilename));

    if (!(*deserializedInstance == *instance))
    {
        cout << "XML for serialized instance:" << endl;
        cout << instance->dump() << endl;
        cout << "XML for serialization of deserialized instance:" << endl;
        cout << deserializedInstance->dump() << endl;
        throw Exception(
            "testComponents: for " + className +
            ", deserialization did not produce an identical object.",
            __FILE__, __LINE__);
    }

    Component* deserializedComp = dynamic_cast<Component *>(deserializedInstance);

    testComponentEquivalence(instance, deserializedComp);
    delete deserializedInstance;
}

void addObjectAsComponentToModel(Object* instance, Model& model)
{
    const string& className = instance->getConcreteClassName();
    cout << "Adding " << className << " to the model." << endl;

    try{
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
        else if (Object::isObjectTypeDerivedFrom< Frame >(className))
            model.addFrame(dynamic_cast<Frame*>(instance));
        else if (Object::isObjectTypeDerivedFrom< ModelComponent >(className))
            model.addModelComponent(dynamic_cast<ModelComponent*>(instance));
        else
        {
            throw Exception(className + " is not a ModelComponent.",
                __FILE__, __LINE__);
        }
    }
    // It is more than likely that connect() will fail, but the subcomponents tree
    // will be traversable, so we can continue to resolve dependencies by visiting
    // subcomponents' connectors
    catch (const std::exception& e) {
        cout << "testComponents: Model unable to connect after adding ";
        cout << instance->getName() << endl;
        cout << "ERROR: " << e.what() << "'" << endl;
        cout << "Possible that dependency was not added yet. Continuing...." << endl;
    }
}
