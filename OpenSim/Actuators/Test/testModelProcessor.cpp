/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testModelProcessor.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#define CATCH_CONFIG_MAIN
#include "OpenSim/Moco/Test/Testing.h"

#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>

using namespace OpenSim;

TEST_CASE("ModelProcessor") {

    Object::registerType(ModelProcessor());

    class MyModelOperator : public ModelOperator {
        OpenSim_DECLARE_CONCRETE_OBJECT(MyModelOperator, ModelOperator);

    public:
        void operate(Model& model, const std::string&) const override {
            model.addAnalysis(new MuscleAnalysis());
        }
    };
    Object::registerType(MyModelOperator());
    Model model = ModelFactory::createPendulum();

    SECTION("Exceptions") {
        // Exception if no model was provided.
        CHECK_THROWS(ModelProcessor().process());
        // No exception if an empty model is provided.
        ModelProcessor(Model{}).process();
        {
            ModelProcessor proc(Model{});
            proc.set_filepath("file.osim");
            CHECK_THROWS_WITH(proc.process(),
                    Catch::Contains("Expected either a Model object or a "
                                    "filepath"));
        }
    }

    SECTION("Operators take effect") {
        ModelProcessor proc = ModelProcessor(model) | MyModelOperator();
        CHECK(proc.process().getAnalysisSet().getSize() == 1);
    }

    SECTION("Serialization") {
        model.print("testModelProcessor_model.osim");
        {
            ModelProcessor proc =
                    ModelProcessor("testModelProcessor_model.osim") |
                    MyModelOperator();
            proc.print("testModelProcessor_ModelProcessor.xml");
        }
        {
            std::unique_ptr<Object> obj(Object::makeObjectFromFile(
                    "testModelProcessor_ModelProcessor.xml"));
            auto* proc = dynamic_cast<ModelProcessor*>(obj.get());
            REQUIRE(proc);
            Model modelDeserialized = proc->process();
            CHECK(modelDeserialized.getAnalysisSet().getSize() == 1);
        }
    }
}

Model createElbowModel() {
    Model model;
    using SimTK::Vec3;
    using SimTK::Inertia;
    auto* body = new OpenSim::Body("body", 1, Vec3(0), Inertia(0));
    auto* joint = new PinJoint("joint",
            model.getGround(), Vec3(0), Vec3(0),
            *body, Vec3(0, 1, 0), Vec3(0));

    // Add a muscle that flexes the elbow.
    auto* biceps = new
            Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
    biceps->addNewPathPoint("origin", model.getGround(), Vec3(0, 0.8, 0));
    biceps->addNewPathPoint("insertion", *body,  Vec3(0, 0.7, 0));

    model.addBody(body);
    model.addJoint(joint);
    model.addForce(biceps);
    model.finalizeConnections();

    return model;
}

TEST_CASE("ModOpRemoveMuscles") {
    ModelProcessor proc(createElbowModel());
    proc.append(ModOpRemoveMuscles());
    Model processedModel = proc.process();
    processedModel.finalizeFromProperties();

    CHECK(processedModel.countNumComponents<Millard2012EquilibriumMuscle>() ==
            0);
}

TEST_CASE("ModOpReplaceMusclesWithPathActuators") {
    ModelProcessor proc(createElbowModel());
    proc.append(ModOpReplaceMusclesWithPathActuators());
    Model processedModel = proc.process();
    processedModel.finalizeFromProperties();

    CHECK(processedModel.countNumComponents<Millard2012EquilibriumMuscle>() ==
            0);
    CHECK(processedModel.countNumComponents<PathActuator>() == 1);
}
