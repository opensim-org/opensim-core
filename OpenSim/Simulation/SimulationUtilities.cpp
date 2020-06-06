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

#include <simbody/internal/Visualizer_InputListener.h>

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
        log_error("The final time must be in the future (current time is {}) simulation aborted.",
            initialTime);
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
        log_cout("A visualizer window has opened.");

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
            log_warn("updateKinematicsStorageForUpdatedModel(): motion '{}' "
                     "does not contain inconsistent coordinate '{}'.)",
                    kinematics.getName(), coord->getName());
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
        log_info("Writing converted motion '{}' to '{}'.", filePath,
                outFilePath);

        updatedMotion->print(outFilePath);
    }
}

void OpenSim::updateSocketConnecteesBySearch(Model& model)
{
    int numSocketsUpdated = 0;
    for (auto& comp : model.updComponentList()) {
        const auto socketNames = comp.getSocketNames();
        for (size_t i = 0; i < socketNames.size(); ++i) {
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
                        log_warn("Socket '{}' in Component {} needs updating "
                                "but a connectee with the specified name "
                                "could not be found.",
                                socketNames[i], comp.getAbsolutePathString());
                    }
                }
            } catch (const std::exception& e) {
                log_warn("Caught exception when processing Socket {} in {} at "
                         "{} : {}.",
                        socketNames[i], comp.getConcreteClassName(),
                        comp.getAbsolutePathString(), e.what());
            }
        }
    }
    if (numSocketsUpdated) {
        log_info("OpenSim::updateSocketConnecteesBySearch(): updated {} "
                 "Sockets in Model '{}'.)",
                numSocketsUpdated, model.getName());
    } else {
        log_info("OpenSim::updateSocketConnecteesBySearch(): no Sockets updated in Model '{}'.",
                  model.getName());
    }
}
