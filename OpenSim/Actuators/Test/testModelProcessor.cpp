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

#include "OpenSim/Common/Sine.h"
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>

#include <catch2/catch_all.hpp>

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
                    Catch::Matchers::ContainsSubstring(
                        "Expected either a Model object or a filepath"));
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

TEST_CASE("ModOpPrescribeMotion") {
    // model has a slider as a Component
    auto unprescribedModel = ModelFactory::createSlidingPointMass();

    // add another slider as a Joint
    auto* body = new Body("body2", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    unprescribedModel.addComponent(body);
    body->attachGeometry(new Sphere(0.05));
    auto* joint = new SliderJoint("slider2", unprescribedModel.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    unprescribedModel.addJoint(joint);

    unprescribedModel.finalizeConnections();

    // create sine data
    std::string componentPath = "/slider/position/value";
    std::string jointPath = "/jointset/slider2/position/value";
    std::vector<std::string> paths = {componentPath, jointPath};
    double interval = 0.05;
    double duration = 3.0;
    std::vector<double> times;
    int numEntries = duration / interval + 1;
    for (int i = 0; i < numEntries; ++i) {
        times.push_back(i * interval);
    }
    int numColumns = static_cast<int>(paths.size());
    SimTK::Matrix positions(numEntries, numColumns);
    for (int col = 0; col < numColumns; ++col) {
        SimTK::Vector x(1);
        for (int row = 0; row < numEntries; ++row) {
            x[0] = row * interval + col;     // +col makes each column different
            positions.set(row, col, Sine().calcValue(x));
        }
    }
    TimeSeriesTable table(times, positions, paths);

    // prescribe motion data to model
    TableProcessor table_processor = TableProcessor(table);
    ModelProcessor modelProcessor(unprescribedModel);
    modelProcessor.append(ModOpPrescribeCoordinateValues(table_processor));
    Model model = modelProcessor.process();

    // check that positions match
    auto* reporter = new StatesTrajectoryReporter();
    reporter->setName("reporter");
    reporter->set_report_time_interval(interval);
    model.addComponent(reporter);
    SimTK::State& state = model.initSystem();
    model.realizePosition(state);
    Manager manager(model, state);
    manager.integrate(duration);
    StatesTrajectory statesTraj = reporter->getStates();

    for (const std::string& path : paths) {
        int jointColumn = static_cast<int>(table.getColumnIndex(path));
        for (int itime = 0; itime < static_cast<int>(statesTraj.getSize()); ++itime) {
            state = statesTraj[itime];
            double time = state.getTime();
            double posActual = model.getStateVariableValue(state, path);
            SimTK::RowVectorView row = table.getNearestRow(time);
            double posExpected = row[jointColumn];
            REQUIRE(posActual == Catch::Approx(posExpected).margin(1e-3));
        }
    }
}
