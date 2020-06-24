/* -------------------------------------------------------------------------- *
 *                           OpenSim:  VisualizerUtilities.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "VisualizerUtilities.h"

#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Common/TableSource.h>
#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/OpenSense/ExperimentalMarker.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/StatesTrajectory.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void VisualizerUtilities::showModel(Model model) {
    model.setUseVisualizer(true);

    // Avoid excessive display of Frames for all Bodies, Ground and additional
    // Frames
    SimTK::State& si = model.initSystem();
    auto silo = &model.updVisualizer().updInputSilo();

    SimTK::DecorativeText help("(hit Esc. to exit)");
    help.setIsScreenText(true);
    auto simtkVisualizer = model.updVisualizer().updSimbodyVisualizer();
    simtkVisualizer.addDecoration(
            SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), help);
    simtkVisualizer.setShutdownWhenDestructed(true);
    // This line is to avoid muscles starting at undefined initialState
    model.equilibrateMuscles(si);
    model.getMultibodySystem().realize(si, SimTK::Stage::Velocity);
    model.getVisualizer().show(si);

    // exit gracefully on key press
    silo->clear(); // Ignore any previous key presses.
    unsigned key, modifiers;
    if (silo->takeKeyHit(key, modifiers)) {
        // Exit.
        if (key == SimTK::Visualizer::InputListener::KeyEsc) {
            std::cout << "Exiting visualization." << std::endl;
            return;
        }
    }
}

// Based on code from simtk.org/projects/predictivesim SimbiconExample/main.cpp.
void VisualizerUtilities::showMotion(Model model, TimeSeriesTable statesTable) {

    const SimTK::Real initialTime = statesTable.getIndependentColumn().front();
    const SimTK::Real finalTime = statesTable.getIndependentColumn().back();
    const SimTK::Real duration = finalTime - initialTime;

    // A data rate of 300 Hz means we can maintain 30 fps down to
    // realTimeScale = 0.1. But if we have more than 20 seconds of data, then
    // we lower the data rate to avoid using too much memory.
    const double desiredNumStates = std::min(300 * duration, 300.0 * 20.0);
    const double dataRate = desiredNumStates / duration; // Hz
    const double frameRate = 30;                         // Hz.

    // Prepare data.
    // -------------
    statesTable =
            TableUtilities::resampleWithInterval(statesTable, 1.0 / dataRate);
    if (TableUtilities::isInDegrees(statesTable)) {
        model.setUseVisualizer(false);
        model.initSystem();
        model.getSimbodyEngine().convertDegreesToRadians(statesTable);
    }
    auto statesTraj = StatesTrajectory::createFromStatesTable(
            model, statesTable, true, true, false);
    const int numStates = (int)statesTraj.getSize();

    // Must setUseVisualizer(true) *after* createFromStatesStorage(), otherwise
    // createFromStatesStorage() spawns a visualizer.
    model.setUseVisualizer(true);
    model.initSystem();

    // This line allows muscle activity to be visualized. To get muscle activity
    // we probably need to realize only to Dynamics, but realizing to Report
    // will catch any other calculations that custom components require for
    // visualizing.
    for (const auto& state : statesTraj) { model.realizeReport(state); }

    // Set up visualization.
    // ---------------------
    // model.updMatterSubsystem().setShowDefaultGeometry(true);
    auto& viz = model.updVisualizer().updSimbodyVisualizer();
    std::string modelName =
            model.getName().empty() ? "<unnamed>" : model.getName();
    std::string title = "Visualizing model '" + modelName + "'";
    title += " (" + getFormattedDateTime(false, "ISO") + ")";
    viz.setWindowTitle(title);
    viz.setMode(SimTK::Visualizer::RealTime);
    // Buffering causes issues when the user adjusts the "Speed" slider.
    viz.setDesiredBufferLengthInSec(0);
    viz.setDesiredFrameRate(frameRate);
    viz.setShowSimTime(true);
    // viz.setBackgroundType(viz.SolidColor);
    // viz.setBackgroundColor(SimTK::White);
    // viz.setShowFrameRate(true);
    // viz.setShowFrameNumber(true);
    auto& silo = model.updVisualizer().updInputSilo();

    // BodyWatcher to control camera.
    // TODO

    // Add sliders to control playback.
    // Real-time factor:
    //      1 means simulation-time = real-time
    //      2 means playback is 2x faster.
    const int realTimeScaleSliderIndex = 1;
    const double minRealTimeScale = 0.01; // can't go to 0.
    const double maxRealTimeScale = 4;
    double realTimeScale = 1.0;
    viz.addSlider("Speed", realTimeScaleSliderIndex, minRealTimeScale,
            maxRealTimeScale, realTimeScale);

    // TODO this slider results in choppy playback if not paused.
    const int timeSliderIndex = 2;
    double time = initialTime;
    viz.addSlider("Time", timeSliderIndex, initialTime, finalTime, time);

    SimTK::Array_<std::pair<SimTK::String, int>> keyBindingsMenu;
    keyBindingsMenu.push_back(std::make_pair(
            "Available key bindings (clicking these menu items has no effect):",
            1));
    keyBindingsMenu.push_back(std::make_pair(
            "-----------------------------------------------------------------",
            2));
    keyBindingsMenu.push_back(std::make_pair("Pause: Space", 3));
    keyBindingsMenu.push_back(std::make_pair("Zoom to fit: R", 4));
    keyBindingsMenu.push_back(std::make_pair("Quit: Esc", 5));
    viz.addMenu("Key bindings", 1, keyBindingsMenu);

    SimTK::DecorativeText pausedText("");
    pausedText.setIsScreenText(true);
    const int pausedIndex = viz.addDecoration(
            SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), pausedText);

    int istate = 0;

    bool paused = false;

    while (true) {
        if (istate == numStates) {
            istate = 0;
            // Without this line, all but the first replay will be shown as
            // fast as possible rather than as real-time.
            viz.setMode(SimTK::Visualizer::RealTime);
        }

        // Slider input.
        int sliderIndex;
        double sliderValue;
        if (silo.takeSliderMove(sliderIndex, sliderValue)) {
            if (sliderIndex == realTimeScaleSliderIndex) {
                viz.setRealTimeScale(sliderValue);
            } else if (sliderIndex == timeSliderIndex) {
                // index = [seconds] * [# states / second]
                auto desiredIndex = (sliderValue - initialTime) * dataRate;
                istate = (int)SimTK::clamp(0, desiredIndex, numStates - 1);
                // Allow the user to drag this slider to visualize different
                // times.
                viz.drawFrameNow(statesTraj[istate]);
            } else {
                log_cout("Internal error: unrecognized slider.");
            }
        }

        // Key input.
        unsigned key, modifiers;
        if (silo.takeKeyHit(key, modifiers)) {
            // Exit.
            if (key == SimTK::Visualizer::InputListener::KeyEsc) {
                log_cout("Exiting visualization.");
                return;
            }
            // Smart zoom.
            else if (key == 'r') {
                viz.zoomCameraToShowAllGeometry();
            }
            // Pause.
            else if (key == ' ') {
                paused = !paused;
                auto& text = static_cast<SimTK::DecorativeText&>(
                        viz.updDecoration(pausedIndex));
                text.setText(paused ? "Paused (hit Space to resume)" : "");
                // Show the updated text.
                viz.drawFrameNow(statesTraj[istate]);
            }
        }

        viz.setSliderValue(realTimeScaleSliderIndex, viz.getRealTimeScale());
        viz.setSliderValue(timeSliderIndex,
                std::round((istate / dataRate + initialTime) * 1000) / 1000);

        if (paused) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } else {
            viz.report(statesTraj[istate]);
            ++istate;
        }
    }
}

void VisualizerUtilities::showMarkerData(
        const TimeSeriesTableVec3& markerTimeSeries) {
    Model previewWorld;

    TimeSeriesTableVec3 markerTimeSeriesInMeters(markerTimeSeries);
    // if units are in mm, convert to meters first
    if (markerTimeSeriesInMeters.getTableMetaData().hasKey("Units")) {
        auto unitsString =
                markerTimeSeriesInMeters.getTableMetaData().getValueAsString(
                        "Units");
        if (unitsString == "mm") {
            for (auto column : markerTimeSeriesInMeters.getColumnLabels())
                markerTimeSeriesInMeters.updDependentColumn(column) /= 1000;
        }
    }
    // Load the marker data into a TableSource that has markers
    // as its output which each markers occupying its own channel
    TableSourceVec3* markersSource =
            new TableSourceVec3(markerTimeSeriesInMeters);
    // Add the markersSource Component to the model
    previewWorld.addComponent(markersSource);

    // Get the underlying Table backing the marker Source so we
    // know how many markers we have and their names
    const auto& markerData = markersSource->getTable();
    auto& times = markerData.getIndependentColumn();

    // Create an ExperimentalMarker Component for every column in the markerData
    for (int i = 0; i < int(markerData.getNumColumns()); ++i) {
        auto marker = new ExperimentalMarker();
        marker->setName(markerData.getColumnLabel(i));

        // markers are owned by the model
        previewWorld.addComponent(marker);
        // the time varying location of the marker comes from the markersSource
        // Component
        marker->updInput("location_in_ground")
                .connect(markersSource->getOutput("column").getChannel(
                        markerData.getColumnLabel(i)));
    }

    previewWorld.setUseVisualizer(true);
    SimTK::State& state = previewWorld.initSystem();
    state.updTime() = times[0];
    const SimTK::Real initialTime = times.front();
    const SimTK::Real finalTime = times.back();

    previewWorld.updVisualizer().updSimbodyVisualizer().setBackgroundType(
            SimTK::Visualizer::SolidColor);
    previewWorld.updVisualizer().updSimbodyVisualizer().setBackgroundColor(
            SimTK::Black);
    previewWorld.realizePosition(state);
    previewWorld.getVisualizer().show(state);

    while (true) {
        for (size_t j = 0; j < times.size(); ++j) {
            // log_cout("time: {} s", times[j]);
            state.setTime(times[j]);
            previewWorld.realizePosition(state);
            previewWorld.getVisualizer().show(state);
        }
    }
}

void VisualizerUtilities::showOrientationData(
        const TimeSeriesTableQuaternion& quatTable, std::string layoutString,
        const Model* modelForPose) {

    Model world;
    std::map<std::string, int> mapOfLayouts = {
            {"line", 0}, {"circle", 1}, {"model", 2}};
    int layout = 0;
    if (mapOfLayouts.find(layoutString) == mapOfLayouts.end()) {
        cout << "Warning: layout option " << layoutString
             << " not found, ignoring and assuming line layout.." << endl;
    } else
        layout = mapOfLayouts.at(layoutString);
    // Will create a Body for every column of data, connect it to
    // Ground with a FreeJoint, set translation based on layout,
    // set rotations based on data.
    const size_t numOrientations = quatTable.getNumColumns();
    auto& times = quatTable.getIndependentColumn();
    SimTK::Array_<FreeJoint*> joints;
    SimTK::Array_<Body*> bodies;
    SimTK::Array_<MobilizedBodyIndex> mobis;

    for (int i = 0; i < (int)numOrientations; ++i) {
        auto name = quatTable.getColumnLabel(i);
        // remove trailing "_imu"
        std::string::size_type pos = name.find("_imu");
        if (pos != string::npos) name = name.substr(0, pos);
        // Create Body
        OpenSim::Body* body = new OpenSim::Body(name, 1, Vec3(0), Inertia(0));
        world.addBody(body);
        bodies.push_back(body);
        // Create FreeJoint
        FreeJoint* free = new FreeJoint(name, world.getGround(), *body);
        world.addJoint(free);
        joints.push_back(free);
    }
    auto applyLayout = [=](int choice) {
        int numJoints = joints.size();
        switch (choice) {
        case 0:
        default:
            for (int i = 0; i < numJoints; i++) {
                // Spacing 0.25 to avoid overlap
                joints[i]
                        ->updCoordinate(FreeJoint::Coord::TranslationZ)
                        .set_default_value(0.25 * (i + 1));
                joints[i]
                        ->updCoordinate(FreeJoint::Coord::TranslationY)
                        .set_default_value(0.2);
            }
            break;
        case 1:
            for (int i = 0; i < numJoints; i++) {
                joints[i]
                        ->updCoordinate(FreeJoint::Coord::TranslationY)
                        .set_default_value(1 + sin((double)i / (numJoints - 1) *
                                                       SimTK::Pi));
                joints[i]
                        ->updCoordinate(FreeJoint::Coord::TranslationZ)
                        .set_default_value(1 + cos((double)i / (numJoints - 1) *
                                                       SimTK::Pi));
            }
            break;
        case 2:
            OPENSIM_THROW_IF(!modelForPose, Exception,
                    "Expected a model for layout 'model', but none provided.");
            State s = modelForPose->getWorkingState();
            modelForPose->realizePosition(s);
            OpenSim::Array<std::string> bNames;
            modelForPose->getBodySet().getNames(bNames);
            for (int i = 0; i < numJoints; i++) {
                bool nameFound = false;
                auto nameFromData = joints[i]->getName();
                for (int j = 0; j < bNames.size() && !nameFound; ++j) {
                    if (nameFromData == bNames[j]) {
                        const Body& bod =
                                modelForPose->getComponent<OpenSim::Body>(
                                        "/bodyset/" + bNames[j]);
                        auto pos = bod.getPositionInGround(s);
                        joints[i]
                                ->updCoordinate(FreeJoint::Coord::TranslationX)
                                .set_default_value(pos[0]);
                        joints[i]
                                ->updCoordinate(FreeJoint::Coord::TranslationY)
                                .set_default_value(pos[1]);
                        joints[i]
                                ->updCoordinate(FreeJoint::Coord::TranslationZ)
                                .set_default_value(pos[2]);
                        nameFound = true;
                        break;
                    }
                }
                if (!nameFound) {
                    cout << "No body with name " << nameFromData
                         << " was found in model."
                         << " Data for this orientation will be displayed at "
                            "origin";
                }
            }
            break;
        }
    };
    applyLayout(layout);
    world.updDisplayHints().set_show_frames(true);
    world.setUseVisualizer(true);
    SimTK::State& state = world.initSystem();
    // Customize visualizer window
    const SimTK::Real initialTime = times.front();
    const SimTK::Real finalTime = times.back();
    bool pause = false;
    addVisualizerControls(world.updVisualizer(), initialTime, finalTime);
    SimTK::DecorativeText pausedText("(hit Esc. to exit, Space to pause)");
    pausedText.setIsScreenText(true);
    const int pausedIndex =
            world.updVisualizer().updSimbodyVisualizer().addDecoration(
                    SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), pausedText);

    // Will add text on screen corresponding to Body names
    for (unsigned b = 0; b < bodies.size(); ++b) {
        MobilizedBodyIndex mbi = bodies.getElt(b)->getMobilizedBodyIndex();
        DecorativeText bodyNameText(bodies.getElt(b)->getName());
        bodyNameText.setScale(0.05);
        bodyNameText.setColor(SimTK::Blue);
        world.updVisualizer().updSimbodyVisualizer().addDecoration(
                SimTK::MobilizedBodyIndex(mbi), SimTK::Vec3(0, -.02, 0),
                bodyNameText);
    }
    world.realizePosition(state);
    world.getVisualizer().show(state);
    auto& simbodyVisualizer = world.getVisualizer().getSimbodyVisualizer();
    auto& dataMatrix = quatTable.getMatrix();
    auto applyFrame = [&](int frameI) {
        state.setTime(times[frameI]);
        for (int iOrient = 0; iOrient < (int)numOrientations; ++iOrient) {

            Quaternion quat = dataMatrix(frameI, iOrient);
            Rotation rot(quat);
            Vec3 angles = rot.convertRotationToBodyFixedXYZ();
            joints.updElt(iOrient)
                    ->updCoordinate(FreeJoint::Coord::Rotation1X)
                    .setValue(state, angles[0]);
            joints.updElt(iOrient)
                    ->updCoordinate(FreeJoint::Coord::Rotation2Y)
                    .setValue(state, angles[1]);
            joints.updElt(iOrient)
                    ->updCoordinate(FreeJoint::Coord::Rotation3Z)
                    .setValue(state, angles[2]);
        }
        world.realizePosition(state);
        world.getVisualizer().show(state);
    };
    while (true) {
        for (int frameNumber = 0; frameNumber < (int)times.size();
                ++frameNumber) {
            applyFrame(frameNumber);
            // Slider input.
            int timeSliderIndex = 1;
            int sliderIndex;
            double sliderValue;
            auto& silo = world.updVisualizer().updInputSilo();

            if (silo.takeSliderMove(sliderIndex, sliderValue)) {
                if (sliderIndex == timeSliderIndex) {
                    auto desiredIndex = (sliderValue - initialTime) /
                                        (finalTime - initialTime);
                    frameNumber = static_cast<int>(desiredIndex * times.size());
                    applyFrame(frameNumber);
                } else {
                    std::cout << "Internal error: unrecognized slider."
                              << std::endl;
                }
            }
            // Key input.
            unsigned key, modifiers;
            if (silo.takeKeyHit(key, modifiers)) {
                // Exit.
                if (key == SimTK::Visualizer::InputListener::KeyEsc) {
                    std::cout << "Exiting visualization." << std::endl;
                    return;
                }
                // Smart zoom.
                else if (key == 'r') {
                    simbodyVisualizer.zoomCameraToShowAllGeometry();
                }
                // Pause.
                else if (key == ' ') {
                    pause = !pause;
                    auto& text = static_cast<SimTK::DecorativeText&>(
                            simbodyVisualizer.updDecoration(pausedIndex));
                    text.setText(pause ? "Paused (hit Space to resume)"
                                       : "(hit Esc. to exit, Space to pause)");
                    simbodyVisualizer.drawFrameNow(state);
                }
            }
            if (pause) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                if (frameNumber > 0) frameNumber--;
            }
            simbodyVisualizer.setSliderValue(
                    timeSliderIndex, times[frameNumber]);
        }
    }
}

void VisualizerUtilities::addVisualizerControls(
        ModelVisualizer& vizualizer, double initialTime, double finalTime) {
    auto& simbodyViz = vizualizer.updSimbodyVisualizer();
    // simbodyViz.setMode(SimTK::Visualizer::RealTime);
    simbodyViz.setDesiredBufferLengthInSec(0);
    simbodyViz.setDesiredFrameRate(30);
    simbodyViz.setShowSimTime(true);
    // viz.setBackgroundType(viz.SolidColor);
    // viz.setBackgroundColor(SimTK::White);
    // viz.setShowFrameRate(true);
    // viz.setShowFrameNumber(true);
    auto& silo = vizualizer.updInputSilo();

    // BodyWatcher to control camera.
    // TODO

    // Add slider to control playback.

    const int realTimeScaleSliderIndex = 1;
    // simbodyViz.addSlider("Speed", realTimeScaleSliderIndex, minRealTimeScale,
    //        maxRealTimeScale, realTimeScale);

    // TODO this slider results in choppy playback if not paused.
    const int timeSliderIndex = 1;
    double time = initialTime;
    simbodyViz.addSlider("Time", timeSliderIndex, initialTime, finalTime, time);

    SimTK::Array_<std::pair<SimTK::String, int>> keyBindingsMenu;
    keyBindingsMenu.push_back(std::make_pair(
            "Available key bindings (clicking these menu items has no effect):",
            1));
    keyBindingsMenu.push_back(std::make_pair(
            "-----------------------------------------------------------------",
            2));
    keyBindingsMenu.push_back(std::make_pair("Pause: Space", 3));
    keyBindingsMenu.push_back(std::make_pair("Zoom to fit: R", 4));
    keyBindingsMenu.push_back(std::make_pair("Quit: Esc", 5));
    simbodyViz.addMenu("Key bindings", 1, keyBindingsMenu);
}
