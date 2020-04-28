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

#include <OpenSim/Common/TableSource.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Simulation/Model/Model.h> 
#include <OpenSim/Simulation/OpenSense/ExperimentalMarker.h>
#include <OpenSim/Simulation/StatesTrajectory.h>


using namespace std;
using namespace OpenSim;

void VisualizerUtilities::showModel(Model model) {
    model.setUseVisualizer(true);
    // If Geometry is located in folders other than where the .osim
    // file is located or in a Geometry folder adjacent to the model
    // file, then define the variable additionalGeometrySearchPath
    // as the folder containing geometry mesh files
    // and uncomment the line below.
    // ModelVisualizer::addDirToGeometrySearchPaths(additionalGeometrySearchPath);
    model.updDisplayHints().set_show_frames(false);
    SimTK::State& si = model.initSystem();
    model.equilibrateMuscles(si);
    model.getMultibodySystem().realize(si, SimTK::Stage::Velocity);
    model.getVisualizer().show(si);
    getchar(); // Keep Visualizer from dying until we inspect the visualization
               // window..
}

// Based on code from simtk.org/projects/predictivesim SimbiconExample/main.cpp.
void VisualizerUtilities::showMotion(Model model, Storage statesSto) {

    const SimTK::Real initialTime = statesSto.getFirstTime();
    const SimTK::Real finalTime = statesSto.getLastTime();
    const SimTK::Real duration = finalTime - initialTime;

    // A data rate of 300 Hz means we can maintain 30 fps down to
    // realTimeScale = 0.1. But if we have more than 20 seconds of data, then
    // we lower the data rate to avoid using too much memory.
    const double desiredNumStates = std::min(300 * duration, 300.0 * 20.0);
    const double dataRate = desiredNumStates / duration; // Hz
    const double frameRate = 30;                         // Hz.

    // Prepare data.
    // -------------
    statesSto.resample(1.0 / dataRate, 4 /* degree */);
    auto statesTraj = StatesTrajectory::createFromStatesStorage(
            model, statesSto, true, true, false);
    const int numStates = (int)statesTraj.getSize();

    // Must setUseVisualizer() *after* createFromStatesStorage(), otherwise
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
    if (!statesSto.getName().empty() && statesSto.getName() != "UNKNOWN")
        title += " with motion '" + statesSto.getName() + "'";
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

    // Load the marker data into a TableSource that has markers
    // as its output which each markers occupying its own channel
    TableSourceVec3* markersSource = new TableSourceVec3(markerTimeSeries);
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

    previewWorld.realizePosition(state);
    previewWorld.getVisualizer().show(state);

    char c;
    log_cout("Press any key to visualize experimental marker data ...");
    std::cin >> c;

    for (size_t j = 0; j < times.size(); j = j + 10) {
        log_cout("time: {} s", times[j]);
        state.setTime(times[j]);
        previewWorld.realizePosition(state);
        previewWorld.getVisualizer().show(state);
    }
}