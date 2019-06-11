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
#include "Testing.h"
#include <Moco/ModelProcessor.h>

#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Common/LogManager.h>

using namespace OpenSim;

TEST_CASE("ModelProcessor") {
    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cerr.rdbuf(LogManager::cerr.rdbuf());

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
            Model modelDeserialized = proc->process();
            CHECK(modelDeserialized.getAnalysisSet().getSize() == 1);
        }
    }
}
