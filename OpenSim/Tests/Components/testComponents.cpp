/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testComponents.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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


#include <OpenSim/OpenSim.h>
#include <OpenSim/Common/Logger.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Auxiliary/getRSS.h>
#include <OpenSim/Simulation/Model/StationDefinedFrame.h>

#include <catch2/catch_all.hpp>

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

using namespace OpenSim;

namespace {

    // helper: adds `instance` to the model in the appropriate collection (`Body` goes
    // into `BodySet`, etc.)
    Component& addObjectAsComponentToModel(
        std::unique_ptr<Object> instance,
        Model& model)
    {
        const std::string& className = instance->getConcreteClassName();
        log_info("Adding {} to the model.", className);

        if (Object::isObjectTypeDerivedFrom<Analysis>(className)) {
            OPENSIM_THROW(Exception, "Analysis is not a Component.");
        }

        // else: it must be a component: figure out where to add it
        Component& rv = dynamic_cast<Component&>(*instance);
        if (Object::isObjectTypeDerivedFrom<Body>(className)) {
            model.addBody(dynamic_cast<Body*>(instance.release()));
        } else if (Object::isObjectTypeDerivedFrom<Constraint>(className)) {
            model.addConstraint(dynamic_cast<Constraint*>(instance.release()));
        } else if (Object::isObjectTypeDerivedFrom<ContactGeometry>(className)) {
            model.addContactGeometry(dynamic_cast<ContactGeometry*>(instance.release()));
        } else if (Object::isObjectTypeDerivedFrom<Controller>(className)) {
            model.addController(dynamic_cast<Controller*>(instance.release()));
        } else if (Object::isObjectTypeDerivedFrom<Force>(className)) {
            model.addForce(dynamic_cast<Force*>(instance.release()));
        } else if (Object::isObjectTypeDerivedFrom<Probe>(className)) {
            model.addProbe(dynamic_cast<Probe*>(instance.release()));
        } else if (Object::isObjectTypeDerivedFrom<Joint>(className)) {
            model.addJoint(dynamic_cast<Joint*>(instance.release()));
        } else {
            model.addComponent(dynamic_cast<Component*>(instance.release()));
        }
        return rv;
    }

    // helper class: used to satisfy the Inputs of the tested components
    class OutputGenerator : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(OutputGenerator, Component);
    public:
        OpenSim_DECLARE_OUTPUT(outdouble, double, calcDouble, SimTK::Stage::Model);
        OpenSim_DECLARE_OUTPUT(outvec3, SimTK::Vec3, calcVec3, SimTK::Stage::Model);
        OpenSim_DECLARE_OUTPUT(outxform, SimTK::Transform, calcXform, SimTK::Stage::Model);

        double calcDouble(const SimTK::State&) const { return 0.0; }
        SimTK::Vec3 calcVec3(const SimTK::State&) const { return SimTK::Vec3(0); }
        SimTK::Transform calcXform(const SimTK::State&) const
        {
            return SimTK::Transform(SimTK::Vec3(0));
        }
    };

    // helper: appends compnents of type `T` in the global registry to `appendOut`
    template<typename T>
    void appendComponentsOfType(std::vector<Component*>& appendOut)
    {
        ArrayPtrs<T> ptrs;
        Object::getRegisteredObjectsOfGivenType(ptrs);
        for (int i = 0; i < ptrs.size(); ++i) {
            if (dynamic_cast<StationDefinedFrame*>(ptrs[i])) {
                // edge-case: `StationDefinedFrame` has transitive sockets that
                // this test suite cannot build
                continue;
            }
            appendOut.push_back(ptrs[i]);
        }
    }

    // helper: gets all of the components that this test suite should test
    std::vector<Component*> getRegisteredComponents()
    {
        std::vector<Component*> rv;
        appendComponentsOfType<Frame>(rv);
        appendComponentsOfType<Geometry>(rv);
        appendComponentsOfType<Point>(rv);
        appendComponentsOfType<Joint>(rv);
        appendComponentsOfType<TwoFrameLinker<Constraint, PhysicalFrame>>(rv);
        appendComponentsOfType<TwoFrameLinker<Force, PhysicalFrame>>(rv);  // (all the BushingForces)
        appendComponentsOfType<PointToPointSpring>(rv);

        // uncomment this when dependencies of `CoordinateCouplerConstraints` are
        // specified as Sockets:
        //
        // appendComponentsOfType<Constraint>;
        return rv;
    }

    // constructs a range of components that this file tests for consistency
    class ComponentTestCases final {
    public:
        ComponentTestCases()
        {
            _components = getRegisteredComponents();
            _components.push_back(&_prescribedForce);
            _components.push_back(&_signalGenerator);
        }

        auto begin() { return _components.begin(); }
        auto end() { return _components.end(); }
    private:
        // non-registered components that should be tested
        PrescribedForce _prescribedForce;
        SignalGenerator _signalGenerator;

        std::vector<Component*> _components;
    };
}

void testComponentEquivalence(
    const Component& a,
    const Component& b,
    bool recurse = true)
{
    const std::string& className = a.getConcreteClassName();

    int ns_a = a.getNumSockets();
    int ns_b = b.getNumSockets();
    log_info("{} getNumSockets: a == {}, b == {}", className, ns_a, ns_b);

    int nin_a = a.getNumInputs();
    int nin_b = b.getNumInputs();
    log_info("{} getNumInputs: a == {}, b == {}", className, nin_a, nin_b);

    int nout_a = a.getNumOutputs();
    int nout_b = b.getNumOutputs();
    log_info("{} getNumOutputs: a == {}, b == {}", className, nout_a, nout_b);

    CHECK((a == b && "components must have same properties"));
    CHECK((ns_a == ns_b && "components must have same number of sockets"));
    CHECK((nin_a == nin_b && "components must have same number of inputs"));
    CHECK((nout_a == nout_b && "components must have same number of outputs"));

    if (recurse) {
        try {
            auto aSubsList = a.getComponentList<Component>();
            auto bSubsList = b.getComponentList<Component>();
            auto iter_a = aSubsList.begin();
            auto iter_b = bSubsList.begin();

            //Subcomponents must be equivalent too!
            while (iter_a != aSubsList.end() && iter_b != aSubsList.end()) {
                testComponentEquivalence(*iter_a, *iter_b, false);
                ++iter_a;
                ++iter_b;
            }
        }
        catch (const ComponentIsRootWithNoSubcomponents& ex) {
            // only trap the `ComponentIsRootWithNoSubcomponents`:
            //
            // just print the exception message but allow the test to
            // continue, because the test is blind to whether Components
            // should have any subcomponents as part of its generic
            // processing.
            log_warn("(ignored exception): {}", ex.what());
        }
    }
}

void testCloning(Component& instance)
{
    log_info("Cloning the component.");

    std::unique_ptr<Component> copyInstance{instance.clone()};
    if (!(*copyInstance == instance))
    {
        log_info("XML serialization for the first instance:");
        log_info("{}", instance.dump());
        log_info("XML serialization for the clone:");
        log_info("{}", copyInstance->dump());

        const std::string& className = instance.getConcreteClassName();
        OPENSIM_THROW(Exception, "testComponents: for " + className + ", clone() did not produce an identical object.");
    }

    instance.finalizeFromProperties();
    copyInstance->finalizeFromProperties();

    testComponentEquivalence(instance, *copyInstance);
}

void testSerialization(Component& instance)
{
    const std::string& className = instance.getConcreteClassName();
    std::string serializationFilename = "testing_serialization_" + className + ".xml";
    instance.print(serializationFilename);

    std::unique_ptr<Object> deserializedInstance{Object::makeObjectFromFile(serializationFilename)};

    if (!(*deserializedInstance == instance))
    {
        log_info("XML for serialized instance:");
        log_info("{}", instance.dump());
        log_info("XML for serialization of deserialized instance:");
        log_info("{}", deserializedInstance->dump());
        OPENSIM_THROW(Exception, "testComponents: for " + className + ", deserialization did not produce an identical object.");
    }

    Component& deserializedComp = dynamic_cast<Component&>(*deserializedInstance);

    instance.finalizeFromProperties();
    deserializedComp.finalizeFromProperties();

    testComponentEquivalence(instance, deserializedComp);
}

void testComponentInAggregate(std::unique_ptr<Component> p)
{
    log_info("Test if {} works in a higher-level aggregate Model", p->getConcreteClassName());

    Model model;
    model.setName("TheModel");

    // add it to the model
    Component& instance = addObjectAsComponentToModel(std::move(p), model);

    // ensure Sockets are satisfied
    Component* sub = &instance;
    auto components = instance.updComponentList<Component>();
    auto componentsCur = components.begin();
    auto componentsEnd = components.end();
    while (sub) {
        log_info("Traversing to {}", sub->getConcreteClassName());
        for (const auto& socketName : sub->getSocketNames()) {
            AbstractSocket& socket = sub->updSocket(socketName);
            std::string dependencyTypeName = socket.getConnecteeTypeName();

            log_info("Socket '{}' has dependency on: {}", socket.getName(), dependencyTypeName);

            // Dependency on a Coordinate needs special treatment.
            // A Coordinate is defined by a Joint and cannot stand on its own.
            // Here we see if there is a Coordinate already in the model, 
            // otherwise we add a Body and Joint so we can connect to its
            // Coordinate.
            if (dynamic_cast<Socket<Coordinate> *>(&socket)) {
                while (!socket.isConnected()) {
                    // Dependency on a coordinate, check if there is one in the model already
                    auto coordinates = model.getComponentList<Coordinate>();
                    if(coordinates.begin() != coordinates.end()) {
                        socket.connect(*coordinates.begin());
                        break;
                    }
                    // no luck finding a Coordinate already in the Model
                    Body* body = new Body();
                    randomize(body);
                    model.addBody(body);
                    model.addJoint(new PinJoint("pin", model.getGround(), *body));
                }
                continue;
            }

            std::unique_ptr<Object> dependency;
            if (dynamic_cast<Socket<Frame>*>(&socket) ||
                dynamic_cast<Socket<PhysicalFrame>*>(&socket)) {
                dependency.reset(Object::newInstanceOfType("Body"));
            } else {
                dependency.reset(Object::newInstanceOfType(dependencyTypeName));
            }

            if (dependency) {
                //give it some random values including a name
                randomize(dependency.get());

                // add the dependency 
                Component& c = addObjectAsComponentToModel(std::move(dependency), model);

                // Connect the socket. This should come after adding the
                // dependency to the model, otherwise the connectee path may
                // be incorrect.
                socket.connect(c);
            }
        }

        log_info("Traversed {}", sub->getConcreteClassName());

        // keep checking the remaining subcomponents
        if (componentsCur != componentsEnd) {
            Component& next = *componentsCur;
            sub = &next;
            ++componentsCur;
        } else {
            sub = nullptr;
        }
    }

    // Now make sure Inputs are satisfied.
    // We'll use the custom OutputGenerator class to satisfy the inputs.
    OutputGenerator* outputGen = new OutputGenerator();
    outputGen->setName("output_gen");
    model.addComponent(outputGen);
    for (auto& sub : model.updComponentList()) {
        for (const auto& inputName : sub.getInputNames()) {
            AbstractInput& input = sub.updInput(inputName);

            // Special case: Geometry cannot have both its input and socket
            // connected.
            if (dynamic_cast<Geometry*>(&sub) && inputName == "transform") {
                input.setConnecteePath("");
                continue;
            }

            std::string dependencyTypeName = input.getConnecteeTypeName();
            log_info("Input '{}' has dependency on: Output<{}>", input.getName(), dependencyTypeName);

            // Find an output of the correct type.
            bool foundAnOutput = false;
            for (const auto& ito : outputGen->getOutputs()) {
                const AbstractOutput* output = ito.second.get();
                if (dependencyTypeName == output->getTypeName()) {
                    input.setConnecteePath(output->getChannel("").getPathName());
                    foundAnOutput = true;
                }
            }
            if (!foundAnOutput) {
                OPENSIM_THROW(Exception, "OutputGenerator does not provide an output of type " + dependencyTypeName + ".");
            }
        }
    }

    // This method calls connect().
    log_info("Calling Model::setup().");
    try{
        model.setup();
    }
    catch (const std::exception& x) {
        log_info("testComponents::{} unable to connect to model:", instance.getConcreteClassName());
        log_info(" '{}'", x.what());
        log_info("Error is likely due to {} having structural dependencies that are not specified as Sockets.", instance.getConcreteClassName());
    }

    // 7. Build the system.
    // --------------------
    SimTK::State initState;
    try{
        initState = model.initSystem();
    }
    catch (const std::exception &x) {
        log_info("testComponents::{} unable to initialize the system:", instance.getConcreteClassName());
        log_info(" '{}'", x.what());
        log_info("Skipping ... ");
    }

    // Verify that the Model (and its System) remains up-to-date with its
    // properties after initSystem (or attempt). Throw if not.
    OPENSIM_THROW_IF(!model.isObjectUpToDateWithProperties(), Exception,
        "testComponents:: model.initSystem() caused Model to no longer be up-to-date with its properties.");

    // Outputs.
    // --------
    log_info("Invoking Output's.");
    for (const auto& entry : instance.getOutputs()) {
        const std::string thisName = entry.first;
        const AbstractOutput* thisOutput = entry.second.get();

        log_info("Testing Output {}, dependent on {}", thisName, thisOutput->getDependsOnStage().getName());

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
        log_info("Component {}, output {}: {}", instance.getConcreteClassName(), thisName, thisOutput->getValueAsString(state));
    }
}

void testComponent(const Component& instanceToTest)
{
    log_info("");
    log_info("**********************************************************");
    log_info("* Testing {}", instanceToTest.getConcreteClassName());
    log_info("**********************************************************");

    // Make a copy so that we can modify the instance.
    std::unique_ptr<Component> instance{instanceToTest.clone()};

    // 1. Set properties to random values.
    // -----------------------------------
    log_info("Randomizing the component's properties.");
    randomize(instance.get());

    // 2. Ensure that cloning produces an exact copy.
    // ----------------------------------------------
    // This will find missing calls to copyProperty_<name>().
    testCloning(*instance);

    // 3. Serialize and de-serialize.
    // ------------------------------
    // This will find issues with de/serialization, after giving the
    // component opportunity to validate the properties via finalizeFromProperties
    log_info("Serializing and deserializing component.");
    instance->finalizeFromProperties();
    testSerialization(*instance);

    // 4. Test the component in a `Model` aggregate
    testComponentInAggregate(std::move(instance));
}

TEST_CASE("testComponents")
{
    for (Component* c : ComponentTestCases{}) {
        DYNAMIC_SECTION("test " << c->getConcreteClassName())
        {
            REQUIRE_NOTHROW(testComponent(*c));
        }
    }
}
