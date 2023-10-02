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
#include <future>
#include <SimTKcommon/internal/IteratorRange.h>

#include <simbody/internal/Visualizer_InputListener.h>

#include <OpenSim/Common/MultivariatePolynomialFunction.h>

#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/Simulation/Model/FunctionBasedPath.h>

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

void OpenSim::appendCoupledCoordinateValues(
        OpenSim::TimeSeriesTable& table, const OpenSim::Model& model,
        bool overwriteExistingColumns) {

    const CoordinateSet& coordinateSet = model.getCoordinateSet();
    const auto& couplerConstraints =
            model.getComponentList<CoordinateCouplerConstraint>();
    for (const auto& couplerConstraint : couplerConstraints) {

        // Get the dependent coordinate and check if the table already contains
        // values for it. If so, skip this constraint (unless we are
        // overwriting existing columns).
        const Coordinate& coordinate = coordinateSet.get(
                couplerConstraint.getDependentCoordinateName());
        const std::string& coupledCoordinatePath =
                fmt::format("{}/value", coordinate.getAbsolutePathString());
        if (table.hasColumn(coupledCoordinatePath)) {
            if (overwriteExistingColumns) {
                table.removeColumn(coupledCoordinatePath);
            } else {
                continue;
            }
        }

        // Get the paths to the independent coordinate values.
        const Array<std::string>& independentCoordinateNames =
                couplerConstraint.getIndependentCoordinateNames();
        std::vector<std::string> independentCoordinatePaths;
        for (int i = 0; i < independentCoordinateNames.getSize(); ++i) {
            const Coordinate& independentCoordinate = coordinateSet.get(
                    independentCoordinateNames[i]);
            independentCoordinatePaths.push_back(
                    fmt::format("{}/value",
                        independentCoordinate.getAbsolutePathString()));
            OPENSIM_THROW_IF(
                    !table.hasColumn(independentCoordinatePaths.back()),
                    Exception,
                    "Expected the coordinates table to contain a column with "
                    "label '{}', but it does not.",
                    independentCoordinatePaths.back())
        }

        // Compute the dependent coordinate values from the function in the
        // CoordinateCouplerConstraint.
        SimTK::Vector independentValues(
                (int)independentCoordinatePaths.size(), 0.0);
        SimTK::Vector newColumn((int)table.getNumRows());
        const Function& function = couplerConstraint.getFunction();
        for (int irow = 0; irow < table.getNumRows(); ++irow) {
            int ival = 0;
            for (const auto& independentCoordinatePath :
                    independentCoordinatePaths) {
                independentValues[ival++] =
                        table.getDependentColumn(independentCoordinatePath)[irow];
            }
            newColumn[irow] = function.calcValue(independentValues);
        }

        // Append the new column to the table.
        table.appendColumn(coupledCoordinatePath, newColumn);
    }
}


void OpenSim::computePathLengthsAndMomentArms(
        Model model,
        const TimeSeriesTable& coordinateValues,
        TimeSeriesTable& pathLengths,
        TimeSeriesTable& momentArms,
        std::map<std::string, std::vector<std::string>>& momentArmMap,
        double momentArmTolerance,
        int threads) {

    // Check inputs.
    OPENSIM_THROW_IF(
            threads < 1 || threads > (int)std::thread::hardware_concurrency(),
            Exception, "Number of threads must be between 1 and {}.",
            std::thread::hardware_concurrency());
    OPENSIM_THROW_IF(pathLengths.getNumRows() != 0, Exception,
            "Expected 'pathLengths' to be empty.");
    OPENSIM_THROW_IF(momentArms.getNumRows() != 0, Exception,
            "Expected 'momentArms' to be empty.");
    momentArmMap.clear();

    // Load model.
    SimTK::State state = model.initSystem();

    // Load coordinate values.
    // TODO check coordinate values
    // TODO delete coordinate speeds
//    TableProcessor tableProcessor = TableProcessor(coordinateValues) |
//                                    TabOpUseAbsoluteStateNames() |
//                                    TabOpAppendCoupledCoordinateValues();
//    TimeSeriesTable coordinateValuesProcessed =
//            tableProcessor.processAndConvertToRadians(model);

    Array<std::string> stateVariableNames = model.getStateVariableNames();
    for (const auto& label : coordinateValues.getColumnLabels()) {
        OPENSIM_THROW_IF(stateVariableNames.findIndex(label) == -1, Exception,
                "Expected the model to contain the coordinate value state "
                "'{}', but it does not.", label);
    }
    auto statesTrajectory = StatesTrajectory::createFromStatesTable(
            model, coordinateValues, true, false, false);

    // Determine the maximum number of path and moment arm evaluations.
    const auto& paths = model.getComponentList<AbstractPath>();
    int numPaths = (int)std::distance(paths.begin(), paths.end());
    int numCoordinates = (int)coordinateValues.getNumColumns();
    int numColumns = numPaths + (numPaths * numCoordinates);

    // Define helper function for path length and moment arm computations.
    auto calcPathLengthsAndMomentArmsSubset = [numColumns, numPaths](
            Model model, StatesTrajectory::IteratorRange subsetStates)
                -> SimTK::Matrix {
        model.initSystem();

        // Create a matrix to store the results
        SimTK::Matrix results(
                (int)std::distance(subsetStates.begin(), subsetStates.end()),
                numColumns);

        int row = 0;
        const auto& forces = model.getComponentList<Force>();
        for (const auto& state : subsetStates) {
            model.realizePosition(state);

            int ip = 0;
            int ima = 0;
            for (const auto& force : forces) {
                if (force.hasProperty("path")) {
                    const AbstractPath& path =
                            force.getProperty<AbstractPath>("path").getValue();

                    // Compute path length.
                    results(row, ip++) = path.getLength(state);

                    // Compute moment arms.
                    for (const auto& coordinate :
                            model.getComponentList<Coordinate>()) {
                        results(row, numPaths + ima++) =
                                path.computeMomentArm(state, coordinate);
                    }
                }
            }
            row++;
        }

        return results;
    };

    // Divide the path length and moment arm computations across multiple
    // threads.
    int stride = static_cast<int>(
            std::floor(coordinateValues.getNumRows() / threads));
    std::vector<std::future<SimTK::Matrix>> futures;
    int offset = 0;
    for (int ithread = 0; ithread < threads; ++ithread) {
        StatesTrajectory::const_iterator begin_iter =
                statesTrajectory.begin() + offset;
        StatesTrajectory::const_iterator end_iter = (ithread == threads-1) ?
                statesTrajectory.end() :
                statesTrajectory.begin() + offset + stride;
        futures.push_back(std::async(std::launch::async,
                calcPathLengthsAndMomentArmsSubset,
                model,
                makeIteratorRange(begin_iter, end_iter)));
        offset += stride;
    }

    // Wait for threads to finish and collect results
    std::vector<SimTK::Matrix> outputs(threads);
    for (int i = 0; i < threads; ++i) {
        outputs[i] = futures[i].get();
    }

    // Assemble results into one TimeSeriesTable
    std::vector<double> time = coordinateValues.getIndependentColumn();
    int itime = 0;
    for (int i = 0; i < threads; ++i) {
        for (int j = 0; j < outputs[i].nrow(); ++j) {
            pathLengths.appendRow(time[itime], outputs[i].block(j, 0, 1,
                    numPaths).getAsRowVector());
            momentArms.appendRow(time[itime], outputs[i].block(j, numPaths, 1,
                    numPaths * numCoordinates).getAsRowVector());
            itime++;
        }
    }

    int ip = 0;
    int ima = 0;
    std::vector<std::string> pathLengthLabels(numPaths);
    std::vector<std::string> momentArmLabels(numPaths * numCoordinates);
    const auto& forces = model.getComponentList<Force>();
    for (const auto& force : forces) {
        if (force.hasProperty("path")) {
            pathLengthLabels[ip++] =
                    fmt::format("{}_length", force.getAbsolutePathString());
            for (const auto& coordinate :
                    model.getComponentList<Coordinate>()) {
                momentArmLabels[ima++] = fmt::format("{}_moment_arm_{}",
                        force.getAbsolutePathString(), coordinate.getName());
            }
        }
    }

    std::cout << "momentArmLabels size: " << momentArmLabels.size() << std::endl;
    pathLengths.setColumnLabels(pathLengthLabels);
    momentArms.setColumnLabels(momentArmLabels);

    // Remove columns for coupled coordinates.
    for (const auto& couplerConstraint :
            model.getComponentList<CoordinateCouplerConstraint>()) {
        auto momentArmLabel = fmt::format("_moment_arm_{}",
                couplerConstraint.getDependentCoordinateName());
        for (const auto& label : momentArms.getColumnLabels()) {
            if (label.find(momentArmLabel) != std::string::npos) {
                momentArms.removeColumn(label);
            }
        }
    }

    // Remove moment arm columns that contain values below the specified
    // moment arm tolerance.
    for (const auto& label : momentArms.getColumnLabels()) {
        if (label.find("_moment_arm_") != std::string::npos) {
            const auto& col = momentArms.getDependentColumn(label);
            if (col.normInf() < momentArmTolerance) {
                momentArms.removeColumn(label);
            } else {
                std::string path = label.substr(0, label.find("_moment_arm_"));
                std::string coordinate = label.substr(
                        label.find("_moment_arm_") + 12);
                momentArmMap[path].push_back(coordinate);
            }
        }
    }
}

double OpenSim::fitFunctionBasedPathCoefficients(
        Model model,
        const TimeSeriesTable& coordinateValues,
        const TimeSeriesTable& pathLengths,
        const TimeSeriesTable& momentArms,
        const std::map<std::string, std::vector<std::string>>& momentArmMap,
        std::string outputPath,
        const int minOrder, const int maxOrder) {

    // Helper functions.
    // -----------------
    // Factorial function.
    auto factorial = [](int n) {
        int result = 1;
        for (int i = 1; i <= n; ++i) {
            result *= i;
        }
        return result;
    };

    // Initialize model.
    // -----------------
    model.initSystem();

    // Coordinate references.
    // ----------------------
    const CoordinateSet& coordinateSet = model.getCoordinateSet();
    const int numCoordinates = coordinateSet.getSize();
    const int numTimes = (int)coordinateValues.getNumRows();

    // Build a FunctionBasedPath for each path-based force in the model.
    // -----------------------------------------------------------------
    // Solving A*x = b, where x is the vector of coefficients for the
    // FunctionBasedPath, A is a matrix of polynomial terms, and b is a vector
    // of path lengths and moment arms.
    Set<FunctionBasedPath> functionBasedPaths;
    const auto forces = model.getComponentList<Force>();
    const int numForces = (int)std::distance(forces.begin(), forces.end());
    SimTK::Vector bestRootMeanSquareErrors(numForces, SimTK::Infinity);
    int iforce = 0;
    for (const auto& force : model.getComponentList<Force>())  {

        // Check if the current force is dependent on any coordinates in the
        // model. If not, skip it.
        if (momentArmMap.find(force.getAbsolutePathString()) ==
                momentArmMap.end()) {
            bestRootMeanSquareErrors[iforce] = 0.0;
            ++iforce;
            continue;
        }

        // The current force path and the number of coordinates it depends on.
        const std::string& forcePath = force.getAbsolutePathString();
        std::vector<std::string> coordinatesNamesThisForce =
                momentArmMap.at(forcePath);
        int numCoordinatesThisForce = (int)coordinatesNamesThisForce.size();
        std::vector<std::string> coordinatePathsThisForce;
        for (const auto& coordinateName : coordinatesNamesThisForce) {
            coordinatePathsThisForce.push_back(
                    coordinateSet.get(coordinateName).getAbsolutePathString());
        }

        // Initialize the 'b' vector. This is the same for all polynomial
        // orders.
        SimTK::Vector b(numTimes * (numCoordinatesThisForce + 1), 0.0);

        // The path lengths for this force. This is the first N elements of the
        // 'b' vector.
        b(0, numTimes) = pathLengths.getDependentColumn(
                fmt::format("{}_length", forcePath));

        // The moment arms and coordinate values for this force.
        SimTK::Matrix coordinatesThisForce(numTimes, numCoordinatesThisForce,
                0.0);
        for (int ic = 0; ic < numCoordinatesThisForce; ++ic) {
            const std::string& coordinateName = coordinatesNamesThisForce[ic];
            b((ic+1)*numTimes, numTimes) = momentArms.getDependentColumn(
                    fmt::format("{}_moment_arm_{}", forcePath, coordinateName));

            const SimTK::VectorView coordinateValuesThisCoordinate =
                    coordinateValues.getDependentColumn(fmt::format("{}/value",
                            coordinatePathsThisForce[ic]));
            for (int itime = 0; itime < numTimes; ++itime) {
                coordinatesThisForce.set(
                        itime, ic, coordinateValuesThisCoordinate[itime]);
            }
        }

        // Polynomial fitting.
        // -------------------
        double bestRootMeanSquareError = SimTK::Infinity;
        SimTK::Vector bestCoefficients;
        int bestOrder = minOrder;
        for (int order = minOrder; order <= maxOrder; ++order) {

            // Initialize the multivariate polynomial function.
            int n = numCoordinatesThisForce + order;
            int k = order;
            int numCoefficients =
                    factorial(n) / (factorial(k) * factorial(n - k));
            SimTK::Vector dummyCoefficients(numCoefficients, 0.0);
            MultivariatePolynomialFunction dummyFunction(dummyCoefficients,
                    numCoordinatesThisForce, order);

            // Initialize the 'A' matrix.
            SimTK::Matrix A(numTimes * (numCoordinatesThisForce + 1),
                    numCoefficients, 0.0);

            // Fill in the A matrix. This contains the polynomial terms for the
            // path length and moment arms.
            for (int itime = 0; itime < numTimes; ++itime) {
                A(itime, 0, 1, numCoefficients) = dummyFunction.getTermValues(
                        coordinatesThisForce.row(itime).getAsVector());

                for (int ic = 0; ic < numCoordinatesThisForce; ++ic) {
                    SimTK::Vector termDerivatives =
                        dummyFunction.getTermDerivatives({ic},
                        coordinatesThisForce.row(itime).getAsVector()).negate();
                    A((ic+1)*numTimes + itime, 0, 1, numCoefficients) =
                            termDerivatives;
                }
            }

            // Solve the least-squares problem. A is a rectangular matrix with
            // full column rank, so we can use the left pseudo-inverse to solve
            // for the coefficients.
            SimTK::Matrix pinv_A = (~A * A).invert() * ~A;
            SimTK::Vector coefficients = pinv_A * b;

            // Calculate the RMS error.
            SimTK::Vector b_fit = A * coefficients;
            SimTK::Vector error = b - b_fit;
            const double rmsError = std::sqrt((error.normSqr() / error.size()));

            // Save best solution.
            if (rmsError < bestRootMeanSquareErrors[iforce]) {
                bestRootMeanSquareErrors[iforce] = rmsError;
                bestCoefficients = coefficients;
                bestOrder = order;
            }

            ++order;
        }

        // Create a FunctionBasedPath for the current path-based force.
        MultivariatePolynomialFunction lengthFunction;
        lengthFunction.setDimension(numCoordinatesThisForce);
        lengthFunction.setOrder(bestOrder);
        lengthFunction.setCoefficients(bestCoefficients);

        auto functionBasedPath = std::make_unique<FunctionBasedPath>();
        functionBasedPath->setName(forcePath);
        functionBasedPath->setCoordinatePaths(coordinatePathsThisForce);
        functionBasedPath->setLengthFunction(lengthFunction);
        functionBasedPaths.adoptAndAppend(functionBasedPath.release());

        ++iforce;
    }

    // Save the function-based paths to a file.
    // ----------------------------------------
    functionBasedPaths.print(outputPath);

    // Return the average RMS error.
    // -----------------------------
    double averageRootMeanSquareError = bestRootMeanSquareErrors.sum() /
            bestRootMeanSquareErrors.size();

    return averageRootMeanSquareError;
}

void OpenSim::sampleAndAppendValues(TimeSeriesTable& values,
        int numSamples, double rangeMultiplier) {

    // Generate coordinate bounds.
    SimTK::Matrix lowerBounds = values.getMatrix();
    SimTK::Matrix upperBounds = values.getMatrix();
    SimTK::RowVector offset = SimTK::mean(lowerBounds);
    OPENSIM_ASSERT_ALWAYS(offset.size() == values.getNumColumns());
    for (int icol = 0; icol < values.getNumColumns(); ++icol) {

        const SimTK::VectorView col = lowerBounds.col(icol);
        const SimTK::Real range = SimTK::max(col) - SimTK::min(col);

        lowerBounds.col(icol).updAsVector() -= offset[icol];
        upperBounds.col(icol).updAsVector() -= offset[icol];

        lowerBounds.col(icol).updAsVector() -= rangeMultiplier * range;
        upperBounds.col(icol).updAsVector() += rangeMultiplier * range;
    }

    // Create a vector of time indices to sample from.
    std::vector<int> sampleTimeIndices;
    sampleTimeIndices.reserve(numSamples);
    const double dN = (double)lowerBounds.nrow() / (double)numSamples;
    for (int i = 0; i < numSamples; ++i) {
        sampleTimeIndices.push_back((int)std::floor(i * dN));
    }

    // Create and seed a random number generator.
    SimTK::Random::Uniform random;
    random.setSeed(static_cast<int>(std::time(nullptr)));

    // Create a matrix to store the samples.
    const int numCoordinates = lowerBounds.ncol();
    SimTK::Matrix samples(2*numSamples, numCoordinates, 0.0);

    // Create vector of indices.
    std::vector<int> indices(numSamples);
    for (int i = 0; i < numSamples; ++i) {
        indices[i] = i;
    }

    // Generate a sample matrix by randomly permuting the indices.
    for (int i = 0; i < numCoordinates; ++i) {
        std::random_shuffle(indices.begin(), indices.end());
        for (int j = 0; j < numSamples; ++j) {
            samples(j, i) = (random.getValue() + indices[j]) / numSamples;
            samples(j, i) *= lowerBounds(sampleTimeIndices[j], i);
            samples(j, i) += offset[i];
        }
    }

    for (int i = 0; i < numCoordinates; ++i) {
        std::random_shuffle(indices.begin(), indices.end());
        for (int j = 0; j < numSamples; ++j) {
            samples(j+numSamples, i) =
                    (random.getValue() + indices[j]) / numSamples;
            samples(j+numSamples, i) *= upperBounds(sampleTimeIndices[j], i);
            samples(j+numSamples, i) += offset[i];
        }
    }


    // Create new time column equal to twice the number of samples.
    const auto& time = values.getIndependentColumn();
    const double dt = time[1] - time[0];
    double timeSampled = time.back();
    for (int itime = 0; itime < 2*numSamples; ++itime) {
        timeSampled += dt;
        values.appendRow(timeSampled, samples.row(itime));
    }
}








