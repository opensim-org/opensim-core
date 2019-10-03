/* -------------------------------------------------------------------------- *
 *                     OpenSim:  SimulationUtilities.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2018 Stanford University and the Authors                *
 * Author(s): OpenSim Team                                                    *
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

#include "SimulationUtilities.h"

#include "Manager/Manager.h"
#include "Model/Model.h"
#include "StatesTrajectory.h"

#include <simbody/internal/Visualizer_InputListener.h>

#include <OpenSim/Common/CommonUtilities.h>

using namespace OpenSim;

SimTK::State OpenSim::simulate(Model& model,
    const SimTK::State& initialState,
    double finalTime,
    bool saveStatesFile)
{
    // Returned state begins as a copy of the initial state
    SimTK::State state = initialState;
    SimTK::Visualizer::InputSilo* silo;

    bool simulateOnce = true;

    // Ensure the final time is in the future.
    const double initialTime = initialState.getTime();
    if (finalTime <= initialTime) {
        std::cout << "The final time must be in the future (current time is "
                  << initialTime << "); simulation aborted." << std::endl;
        return state;
    }

    // Configure the visualizer.
    if (model.getUseVisualizer()) {
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        // We use the input silo to get key presses.
        silo = &model.updVisualizer().updInputSilo();

        SimTK::DecorativeText help("Press any key to start a new simulation; "
            "ESC to quit.");
        help.setIsScreenText(true);
        viz.addDecoration(SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), help);

        viz.setShowSimTime(true);
        viz.drawFrameNow(state);
        std::cout << "A visualizer window has opened." << std::endl;

        // if visualizing enable replay
        simulateOnce = false;
    }

    // Simulate until the user presses ESC (or enters 'q' if visualization has
    // been disabled).
    do {
        if (model.getUseVisualizer()) {
            // Get a key press.
            silo->clear(); // Ignore any previous key presses.
            unsigned key, modifiers;
            silo->waitForKeyHit(key, modifiers);
            if (key == SimTK::Visualizer::InputListener::KeyEsc) { break; }
        }

        // reset the state to the initial state
        state = initialState;
        // Set up manager and simulate.
        Manager manager(model);
        state.setTime(initialTime);
        manager.initialize(state);
        state = manager.integrate(finalTime);

        // Save the states to a storage file (if requested).
        if (saveStatesFile) {
            manager.getStateStorage().print(model.getName() + "_states.sto");
        }
    } while (!simulateOnce);

    return state;
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


std::unique_ptr<Storage>
OpenSim::updatePre40KinematicsStorageFor40MotionType(const Model& pre40Model,
        const Storage &kinematics)
{
    // There is no issue if the kinematics are in internal values (i.e. not
    // converted to degrees)
    if(!kinematics.isInDegrees()) return nullptr;
    
    if (pre40Model.getDocumentFileVersion() >= 30415) {
        throw Exception("updateKinematicsStorageForUpdatedModel has no updates "
            "to make because the model '" + pre40Model.getName() + "'is up-to-date.\n"
            "If input motion files were generated with this model version, "
            "nothing further must be done. Otherwise, provide the original model "
            "file used to generate the motion files and try again.");
    }
    
    std::vector<const Coordinate*> problemCoords;
    auto coordinates = pre40Model.getComponentList<Coordinate>();
    for (auto& coord : coordinates) {
        const Coordinate::MotionType oldMotionType =
                coord.getUserSpecifiedMotionTypePriorTo40();
        const Coordinate::MotionType motionType = coord.getMotionType();
        
        if ((oldMotionType != Coordinate::MotionType::Undefined) &&
            (oldMotionType != motionType)) {
            problemCoords.push_back(&coord);
        }
    }
    
    if (problemCoords.size() == 0)
        return nullptr;
    
    std::unique_ptr<Storage> updatedKinematics(kinematics.clone());
    // Cycle the inconsistent Coordinates
    for (const auto& coord : problemCoords) {
        // Get the corresponding column of data and if in degrees
        // undo the radians to degrees conversion on that column.
        int ix = updatedKinematics->getStateIndex(coord->getName());
        
        if (ix < 0) {
            std::cout << "updateKinematicsStorageForUpdatedModel(): motion '"
            << kinematics.getName() << "' does not contain inconsistent "
            << "coordinate '" << coord->getName() << "'." << std::endl;
        }
        else {
            // convert this column back to internal values by undoing the
            // 180/pi conversion to degrees
            updatedKinematics->multiplyColumn(ix, SimTK_DTR);
        }
    }
    return updatedKinematics;
}

    
void OpenSim::updatePre40KinematicsFilesFor40MotionType(const Model& model,
        const std::vector<std::string>& filePaths,
        std::string suffix)
{
    // Cycle through the data files 
    for (const auto& filePath : filePaths) {
        Storage motion(filePath);
        auto updatedMotion =
            updatePre40KinematicsStorageFor40MotionType(model, motion);

        if (updatedMotion == nullptr) {
            continue; // no update was required, move on to next file
        }

        std::string outFilePath = filePath;
        if (suffix.size()) {
            auto back = filePath.rfind(".");
            outFilePath = filePath.substr(0, back) + suffix +
                            filePath.substr(back);
        }
        std::cout << "Writing converted motion '" << filePath << "' to '"
            << outFilePath << "'." << std::endl;

        updatedMotion->print(outFilePath);
    }
}

void OpenSim::updateSocketConnecteesBySearch(Model& model)
{
    int numSocketsUpdated = 0;
    for (auto& comp : model.updComponentList()) {
        const auto socketNames = comp.getSocketNames();
        for (int i = 0; i < socketNames.size(); ++i) {
            auto& socket = comp.updSocket(socketNames[i]);
            try {
                socket.finalizeConnection(model);
            } catch (const ComponentNotFoundOnSpecifiedPath&) {
                const ComponentPath path(socket.getConnecteePath());
                if (path.getNumPathLevels() >= 1) { 
                    const Component* found =
                        model.findComponent(path.getComponentName());
                    if (found) {
                        socket.connect(*found);
                        socket.finalizeConnection(model);
                        numSocketsUpdated += 1;
                    } else {
                        std::cout << "Socket '" << socketNames[i] << "' in "
                                << "Component " << comp.getAbsolutePathString()
                                << " needs updating but a connectee with the "
                                   "specified name could not be found."
                                << std::endl;
                    }
                }
            } catch (const std::exception& e) {
                std::cout << "Warning: Caught exception when processing "
                    "Socket " << socketNames[i] << " in " <<
                    comp.getConcreteClassName() << " at " <<
                    comp.getAbsolutePathString() << ": " << e.what() <<
                    std::endl;
            }
        }
    }
    if (numSocketsUpdated) {
        std::cout << "OpenSim::updateSocketConnecteesBySearch(): updated "
                << numSocketsUpdated << " Sockets in Model '"
                << model.getName() << "'." << std::endl;
    } else {
        std::cout << "OpenSim::updateSocketConnecteesBySearch(): "
                     "no Sockets updated in Model '"
                  << model.getName() << "'." << std::endl;
    }
}
