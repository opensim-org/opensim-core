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


#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
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

/// creates a model with one sliding mass
std::unique_ptr<Model> createSlidingMassModel() {
    auto model = make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);
    body->attachGeometry(new Sphere(0.05));

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addJoint(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model->addComponent(actu);

    return model;
}

TEST_CASE("ModOpPrescribeMotion") {
    auto unprescribedModel = createSlidingMassModel();
    unprescribedModel->finalizeConnections();

    ModelProcessor modelProcessor(*unprescribedModel);
    TableProcessor table_processor = TableProcessor("linear_move.sto");
    modelProcessor.append(ModOpPrescribeCoordinateValues(table_processor));
    Model model = modelProcessor.process();

    auto* reporter = new StatesTrajectoryReporter();
    reporter->setName("reporter");
    reporter->set_report_time_interval(0.05);
    model.addComponent(reporter);

    SimTK::State& state = model.initSystem();
    model.realizePosition(state);
    Manager manager(model, state);
    manager.integrate(3.0);
    StatesTrajectory statesTraj = reporter->getStates();

    /*double final_t = 3.0;
    double nsteps = 30;
    double dt = final_t / nsteps;
    manager.setIntegratorAccuracy(1e-7);
    state.setTime(0.0);
    manager.initialize(state);*/

    // read file for comparison
    TimeSeriesTable table = TimeSeriesTable("linear_move.sto");

    for (int itime = 0; itime < statesTraj.getSize(); ++itime) {
        state = statesTraj[itime];
        double time = state.getTime();
        double posActual = model.getStateVariableValue(state, "jointset/slider/position/value");
        SimTK::RowVectorView row = table.getNearestRow(time);
        double posExpected = row[table.getColumnIndex("/jointset/slider/position/value")];
        REQUIRE(posActual == Catch::Approx(posExpected).margin(1e-4));
    }
}
