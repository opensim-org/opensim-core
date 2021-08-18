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

#include <OpenSim/Common/TableUtilities.h>

using namespace OpenSim;

SimTK::State OpenSim::simulate(Model& model,
    const SimTK::State& initialState,
    double finalTime,
    bool saveStatesFile)
{
    // Returned state begins as a copy of the initial state
    SimTK::State state = initialState;

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
            auto& silo = model.updVisualizer().updInputSilo();
            silo.clear(); // Ignore any previous key presses.
            unsigned key, modifiers;
            silo.waitForKeyHit(key, modifiers);
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

void OpenSim::updateStateLabels40(const Model& model,
                                  std::vector<std::string>& labels) {

    TableUtilities::checkNonUniqueLabels(labels);

    const Array<std::string> stateNames = model.getStateVariableNames();
    for (int isv = 0; isv < stateNames.size(); ++isv) {
        int i = TableUtilities::findStateLabelIndex(labels, stateNames[isv]);
        if (i == -1) continue;
        labels[i] = stateNames[isv];
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
        log_info("OpenSim::updateSocketConnecteesBySearch(): no Sockets "
                 "updated in Model '{}'.",
                model.getName());
    }
}

std::vector<std::string> OpenSim::createStateVariableNamesInSystemOrder(
        const Model& model) {
    std::unordered_map<int, int> yIndexMap;
    return createStateVariableNamesInSystemOrder(model, yIndexMap);
}

std::vector<std::string> OpenSim::createStateVariableNamesInSystemOrder(
        const Model& model, std::unordered_map<int, int>& yIndexMap) {
    yIndexMap.clear();
    std::vector<std::string> svNamesInSysOrder;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    std::vector<int> yIndices;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                svNamesInSysOrder.push_back(svNames[isv]);
                yIndices.emplace_back(iy);
                s.updY()[iy] = 0;
                break;
            }
        }
        if (SimTK::isNaN(s.updY()[iy])) {
            // If we reach here, this is an unused slot for a quaternion.
            s.updY()[iy] = 0;
        }
    }
    int count = 0;
    for (const auto& iy : yIndices) {
        yIndexMap.emplace(std::make_pair(count, iy));
        ++count;
    }
    SimTK_ASSERT2_ALWAYS((size_t)svNames.size() == svNamesInSysOrder.size(),
            "Expected to get %i state names but found %i.", svNames.size(),
            svNamesInSysOrder.size());
    return svNamesInSysOrder;
}

std::unordered_map<std::string, int> OpenSim::createSystemYIndexMap(
        const Model& model) {
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
        if (SimTK::isNaN(s.updY()[iy])) {
            // If we reach here, this is an unused slot for a quaternion.
            s.updY()[iy] = 0;
        }
    }
    SimTK_ASSERT2_ALWAYS(svNames.size() == (int)sysYIndices.size(),
            "Expected to find %i state indices but found %i.", svNames.size(),
            sysYIndices.size());
    return sysYIndices;
}

std::vector<std::string> OpenSim::createControlNamesFromModel(
        const Model& model, std::vector<int>& modelControlIndices) {
    std::vector<std::string> controlNames;
    // Loop through all actuators and create control names. For scalar
    // actuators, use the actuator name for the control name. For non-scalar
    // actuators, use the actuator name with a control index appended for the
    // control name.
    // TODO update when OpenSim supports named controls.
    int count = 0;
    modelControlIndices.clear();
    for (const auto& actu : model.getComponentList<Actuator>()) {
        if (!actu.get_appliesForce()) {
            count += actu.numControls();
            continue;
        }
        std::string actuPath = actu.getAbsolutePathString();
        if (actu.numControls() == 1) {
            controlNames.push_back(actuPath);
            modelControlIndices.push_back(count);
            count++;
        } else {
            for (int i = 0; i < actu.numControls(); ++i) {
                controlNames.push_back(actuPath + "_" + std::to_string(i));
                modelControlIndices.push_back(count);
                count++;
            }
        }
    }

    return controlNames;
}
std::vector<std::string> OpenSim::createControlNamesFromModel(
        const Model& model) {
    std::vector<int> modelControlIndices;
    return createControlNamesFromModel(model, modelControlIndices);
}

std::unordered_map<std::string, int> OpenSim::createSystemControlIndexMap(
        const Model& model) {
    // We often assume that control indices in the state are in the same order
    // as the actuators in the model. However, the control indices are
    // allocated in the order in which addToSystem() is invoked (not
    // necessarily the order used by getComponentList()). So until we can be
    // absolutely sure that the controls are in the same order as actuators,
    // we can run the following check: in order, set an actuator's control
    // signal(s) to NaN and ensure the i-th control is NaN.
    // TODO update when OpenSim supports named controls.
    std::unordered_map<std::string, int> controlIndices;
    const SimTK::State state = model.getWorkingState();
    auto modelControls = model.updControls(state);
    int i = 0;
    for (const auto& actu : model.getComponentList<Actuator>()) {
        int nc = actu.numControls();
        SimTK::Vector origControls(nc);
        SimTK::Vector nan(nc, SimTK::NaN);
        actu.getControls(modelControls, origControls);
        actu.setControls(nan, modelControls);
        std::string actuPath = actu.getAbsolutePathString();
        for (int j = 0; j < nc; ++j) {
            OPENSIM_THROW_IF(!SimTK::isNaN(modelControls[i]), Exception,
                    "Internal error: actuators are not in the "
                    "expected order. Submit a bug report.");
            if (nc == 1) {
                controlIndices[actuPath] = i;
            } else {
                controlIndices[fmt::format("{}_{}", actuPath, j)] = i;
            }
            ++i;
        }
        actu.setControls(origControls, modelControls);
    }
    return controlIndices;
}

void OpenSim::checkOrderSystemControls(const Model& model) {
    createSystemControlIndexMap(model);
}

void OpenSim::checkLabelsMatchModelStates(const Model& model,
        const std::vector<std::string>& labels) {
    const auto modelStateNames = model.getStateVariableNames();
    for (const auto& label : labels) {
        OPENSIM_THROW_IF(modelStateNames.rfindIndex(label) == -1, Exception,
                "Expected the provided labels to match the model state "
                "names, but label {} does not correspond to any model "
                "state.",
                label);
    }
}

TimeSeriesTableVec3 OpenSim::createSyntheticIMUAccelerationSignals(
        const Model& model,
        const TimeSeriesTable& statesTable, const TimeSeriesTable& controlsTable,
        const std::vector<std::string>& framePaths) {
    std::vector<std::string> outputPaths;

    ComponentPath path;
    for (const auto& framePath : framePaths) {
        OPENSIM_THROW_IF(path.isLegalPathElement(framePath), Exception,
            "Provided frame path '{}' contains invalid characters.", framePath);
        OPENSIM_THROW_IF(
            !model.hasComponent<PhysicalFrame>(framePath), Exception,
            "Expected provided frame path '{}' to point to component of "
            "type PhysicalFrame, but no such component was found.", framePath);
        outputPaths.push_back(framePath + "\\|linear_acceleration");
    }
    TimeSeriesTableVec3 accelTableEffort = analyze<SimTK::Vec3>(
            model, statesTable, controlsTable, outputPaths);

    // Create synthetic IMU signals by extracting the gravitational acceleration
    // vector from the solution accelerations and expressing them in the model
    // frames.
    const auto& statesTraj =
            StatesTrajectory::createFromStatesTable(model, statesTable);
    const auto& ground = model.getGround();
    const auto& gravity = model.getGravity();
    const auto& timeVec = accelTableEffort.getIndependentColumn();
    TimeSeriesTableVec3 accelTableIMU(timeVec);
    for (const auto& framePath : framePaths) {
        std::string label = framePath + "|linear_acceleration";
        const auto& col = accelTableEffort.getDependentColumn(label);
        const auto& frame = model.getComponent<PhysicalFrame>(framePath);
        SimTK::Vector_<SimTK::Vec3> colIMU(col.size());
        for (int i = 0; i < (int)timeVec.size(); ++i) {
            const auto& state = statesTraj.get(i);
            model.realizeAcceleration(state);
            SimTK::Vec3 accelIMU = ground.expressVectorInAnotherFrame(
                    state, col[i] - gravity, frame);
            colIMU[i] = accelIMU;
        }
        accelTableIMU.appendColumn(label, colIMU);
    }
    accelTableIMU.setColumnLabels(framePaths);

    return accelTableIMU;
}