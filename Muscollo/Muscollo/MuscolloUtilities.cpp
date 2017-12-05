/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MuscolloUtilities.cpp                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MuscolloUtilities.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <Simbody/internal/Visualizer_InputListener.h>

using namespace OpenSim;


SimTK::Vector OpenSim::createVectorLinspace(
        int length, double start, double end) {
    SimTK::Vector v(length);
    for (int i = 0; i < length; ++i) {
        v[i] = start + i * (end - start) / (length - 1);
    }
    return v;
}

Storage OpenSim::convertTableToStorage(const TimeSeriesTable& table) {

    Storage sto;
    if (table.hasTableMetaDataKey("inDegrees") &&
        table.getTableMetaDataAsString("inDegrees") == "yes") {
        sto.setInDegrees(true);
    }

    OpenSim::Array<std::string> labels("", (int)table.getNumColumns() + 1);
    labels[0] = "time";
    for (int i = 0; i < (int)table.getNumColumns(); ++i) {
        labels[i + 1] = table.getColumnLabel(i);
    }
    sto.setColumnLabels(labels);
    const auto& times = table.getIndependentColumn();
    for (unsigned i_time = 0; i_time < table.getNumRows(); ++i_time) {
        auto rowView = table.getRowAtIndex(i_time);
        sto.append(times[i_time], SimTK::Vector(rowView.transpose()));
    }
    return sto;
}

/// TODO: doc
OSIMMUSCOLLO_API TimeSeriesTable OpenSim::filterLowpass(const TimeSeriesTable & table, double cutoffFreq, bool padData) {
    auto storage = convertTableToStorage(table);
    if (padData) {
        storage.pad(storage.getSize() / 2);
    }
    storage.lowpassIIR(cutoffFreq);

    return storage.exportToTable();
}

// Based on code from simtk.org/projects/predictivesim SimbiconExample/main.cpp.
void OpenSim::visualize(Model model, Storage statesSto) {

    const SimTK::Real initialTime = statesSto.getFirstTime();
    const SimTK::Real finalTime = statesSto.getLastTime();
    const SimTK::Real duration = finalTime - initialTime;

    // A data rate of 300 Hz means we can maintain 30 fps down to
    // realTimeScale = 0.1. But if we have more than 20 seconds of data, then
    // we lower the data rate to avoid using too much memory.
    const double desiredNumStates = std::min(300 * duration, 300.0 * 20.0);
    const double dataRate = desiredNumStates / duration; // Hz
    const double frameRate = 30; // Hz.

    // Prepare data.
    // -------------
    statesSto.resample(1.0 / dataRate, 4 /* degree */);
    auto statesTraj =
            StatesTrajectory::createFromStatesStorage(model, statesSto,
                    true, true);
    const int numStates = (int)statesTraj.getSize();

    // Must setUseVisualizer() *after* createFromStatesStorage(), otherwise
    // createFromStatesStorage() spawns a visualizer.
    model.setUseVisualizer(true);
    model.initSystem();

    //OPENSIM_THROW_IF(!statesTraj.isCompatibleWith(model), Exception,
    //        "Model is not compatible with the provided StatesTrajectory.");

    // Set up visualization.
    // ---------------------
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    auto& viz = model.updVisualizer().updSimbodyVisualizer();
    std::string modelName = model.getName().empty() ? "<unnamed>"
                                                    : model.getName();
    std::string motionName = statesSto.getName().empty() ? "<unnamed>"
                                                         : statesSto.getName();
    viz.setWindowTitle("Visualizing model '" + modelName + "' with motion '" +
            motionName + "'.");
    viz.setMode(SimTK::Visualizer::RealTime);
    // Buffering causes issues when the user adjusts the "Speed" slider.
    viz.setDesiredBufferLengthInSec(0);
    viz.setDesiredFrameRate(frameRate);
    viz.setShowSimTime(true);
    //viz.setBackgroundType(viz.SolidColor);
    //viz.setBackgroundColor(SimTK::White);
    //viz.setShowFrameRate(true);
    //viz.setShowFrameNumber(true);
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
    viz.addSlider("Speed", realTimeScaleSliderIndex,
            minRealTimeScale, maxRealTimeScale, realTimeScale);

    // TODO this slider results in choppy playback if not paused.
    const int timeSliderIndex = 2;
    double time = initialTime;
    viz.addSlider("Time", timeSliderIndex, initialTime, finalTime, time);

    SimTK::Array_<std::pair<SimTK::String, int>> keyBindingsMenu;
    keyBindingsMenu.push_back(std::make_pair(
        "Available key bindings (clicking these menu items has no effect):", 1));
    keyBindingsMenu.push_back(std::make_pair(
        "-----------------------------------------------------------------", 2));
    keyBindingsMenu.push_back(std::make_pair("Pause: Space", 3));
    keyBindingsMenu.push_back(std::make_pair("Zoom to fit: R", 4));
    keyBindingsMenu.push_back(std::make_pair("Quit: Esc", 5));
    viz.addMenu("Key bindings", 1, keyBindingsMenu);

    SimTK::DecorativeText pausedText("");
    pausedText.setIsScreenText(true);
    const int pausedIndex = viz.addDecoration(SimTK::MobilizedBodyIndex(0),
            SimTK::Vec3(0), pausedText);

    int istate = 0;

    bool paused = false;

    while (true) {
        if (istate == numStates) istate = 0;

        // Slider input.
        int sliderIndex;
        double sliderValue;
        if (silo.takeSliderMove(sliderIndex, sliderValue)) {
            if (sliderIndex == realTimeScaleSliderIndex) {
                viz.setRealTimeScale(sliderValue);
            } else if (sliderIndex == timeSliderIndex) {
                // index = [seconds] * [# states / second]
                istate = (int)SimTK::clamp(0,
                        (sliderValue - initialTime) * dataRate, numStates - 1);
                // Allow the user to drag this slider to visualize different
                // times.
                viz.drawFrameNow(statesTraj[istate]);
            } else {
                std::cout << "Internal error: unrecognized slider." <<
                        std::endl;
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
                std::round((istate / dataRate + initialTime)*1000)/1000);

        if (paused) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } else {
            viz.report(statesTraj[istate]);
            ++istate;
        }
    }
}

std::unordered_map<std::string, int>
OpenSim::createSystemYIndexMap(const Model& model) {
    std::unordered_map<std::string, int> sysYIndices;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                sysYIndices[svNames[isv]] = iy;
                s.updY()[iy] = 0;
                break;
            }
        }
    }
    SimTK_ASSERT2_ALWAYS(svNames.size() == (int)sysYIndices.size(),
            "Expected to find %i state indices but found %i.", svNames.size(),
            sysYIndices.size());
    return sysYIndices;
}
