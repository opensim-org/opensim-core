/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testReportersWithModel.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Thomas Uchida                                                   *
 * Contributor(s):                                                            *
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

#include <OpenSim/Common/LogSink.h>
#include <OpenSim/Common/Logger.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

using namespace std;
using namespace SimTK;
using namespace OpenSim;

void testConsoleReporterLabels() {
    // Create a model consisting of a falling ball.
    Model model;
    model.setName("world");

    auto* ball = new OpenSim::Body("ball", 1., Vec3(0), Inertia(0));
    model.addBody(ball);

    auto* slider = new SliderJoint("slider", model.getGround(), Vec3(0),
        Vec3(0,0,Pi/2.), *ball, Vec3(0), Vec3(0,0,Pi/2.));
    model.addJoint(slider);

    // Create ConsoleReporter, and connect Outputs without and with alias.
    auto* reporter = new ConsoleReporter();
    reporter->set_report_time_interval(1.);
    reporter->addToReport(slider->getCoordinate().getOutput("value"));
    reporter->addToReport(slider->getCoordinate().getOutput("value"), "height");
    model.addComponent(reporter);

    State& state = model.initSystem();
    Manager manager(model);
    state.setTime(0.0);
    manager.initialize(state);

    // Create a sink to obtain the ConsoleReporter output.
    auto sink = std::make_shared<StringLogSink>();
    Logger::addSink(sink);
    manager.integrate(1.0);

    // Restore original destination for cout and display ConsoleReporter output.
    Logger::removeSink(sink);
    const std::string output = sink->getString();
    cout << output << endl;

    // Check column headings reported by ConsoleReporter, which should be
    // "time", then "value", then "height". The amount of whitespace appearing
    // between the columns is unimportant.
    const size_t idxHeading1 = output.find("time");
    const size_t idxHeading2 = output.find("value");
    const size_t idxHeading3 = output.find("height");

    SimTK_TEST(idxHeading1 != string::npos);
    SimTK_TEST(idxHeading2 != string::npos);
    SimTK_TEST(idxHeading3 != string::npos);

    SimTK_TEST(idxHeading1 < idxHeading2);
    SimTK_TEST(idxHeading2 < idxHeading3);
}

void testTableReporterLabels() {
    // Create a model consisting of a falling ball.
    Model model;
    model.setName("world");

    auto* ball = new OpenSim::Body("ball", 1., Vec3(0), Inertia(0));
    model.addBody(ball);

    auto* slider = new SliderJoint("slider", model.getGround(), Vec3(0),
        Vec3(0,0,Pi/2.), *ball, Vec3(0), Vec3(0,0,Pi/2.));
    slider->updCoordinate().setName("sliderCoord");
    model.addJoint(slider);

    // Create TableReporter, and connect Outputs without and with alias.
    auto* reporter = new TableReporter();
    reporter->set_report_time_interval(1.);
    reporter->addToReport(slider->getCoordinate().getOutput("value"));
    reporter->addToReport(slider->getCoordinate().getOutput("value"), "height");
    model.addComponent(reporter);

    // Simulate.
    State& state = model.initSystem();
    Manager manager(model);
    state.setTime(0.0);
    manager.initialize(state);
    manager.integrate(1.0);

    // Check column headings for dependent variables reported by TableReporter,
    // which should be "slider/sliderCoord/value" and "height".
    const auto headings = reporter->getTable().getColumnLabels();
    SimTK_TEST(headings[0] == "/jointset/slider/sliderCoord|value");
    SimTK_TEST(headings[1] == "height");
}

int main() {
    SimTK_START_TEST("testReporters");
        SimTK_SUBTEST(testConsoleReporterLabels);
        SimTK_SUBTEST(testTableReporterLabels);
    SimTK_END_TEST();
};
