/* -------------------------------------------------------------------------- *
* OpenSim Moco: MocoTrack.cpp                                                *
* -------------------------------------------------------------------------- *
* Copyright (c) 2019 Stanford University and the Authors                     *
*                                                                            *
* Author(s): Nicholas Bianco                                                 *
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

#include "MocoTrack.h"
#include "MocoWeightSet.h"
#include "MocoStudy.h"
#include "MocoProblem.h"
#include "MocoCost/MocoControlCost.h"
#include "MocoCost/MocoControlTrackingCost.h"
#include "MocoCost/MocoStateTrackingCost.h"
#include "MocoCost/MocoMarkerTrackingCost.h"
#include "MocoCasADiSolver/MocoCasADiSolver.h"
#include "MocoUtilities.h"

#include <OpenSim/Common/FileAdapter.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>

using namespace OpenSim;

void MocoTrack::constructProperties() {

    constructProperty_states_tracking_file("");
    constructProperty_states_tracking_weight(1);
    constructProperty_state_weights(MocoWeightSet());
    constructProperty_track_state_reference_derivatives(false);
    constructProperty_markers_tracking_file("");
    constructProperty_markers_tracking_weight(1);
    constructProperty_lowpass_cutoff_frequency_for_kinematics(-1);
    constructProperty_ik_setup_file("");
    constructProperty_external_loads_file("");
    constructProperty_guess_type("bounds");
    constructProperty_guess_file("");
    constructProperty_minimize_controls(-1);
    constructProperty_control_weights(MocoWeightSet());
}

MocoStudy MocoTrack::initialize() {

    MocoStudy moco;
    moco.setName(getName());
    MocoProblem& problem = moco.updProblem();

    // Modeling.
    // ---------
    Model model(m_model);
    model.finalizeFromProperties();
    model.initSystem();

    // Costs.
    // ------

    // State tracking cost.
    if (!get_states_tracking_file().empty()) {
        configureStateTracking(problem, model);
    }

    // Marker tracking cost.
    if (!get_markers_tracking_file().empty()) {
        configureMarkerTracking(problem, model);
    }

    // Apply external loads, if any.
    if (!get_external_loads_file().empty()) {
        InverseDynamicsTool idtool;
        idtool.createExternalLoads(get_external_loads_file(), model);
        model.initSystem();
    }

    // Set the model on the MocoProblem, now that we're done configuring costs.
    problem.setModelCopy(model);

    // Control effort minimization.
    // ----------------------------
    if (get_minimize_controls() != -1) {
        auto* effort = problem.addCost<MocoControlCost>("control_effort");
        assert(get_minimize_controls() > 0);
        effort->set_weight(get_minimize_controls());

        const auto& control_weights = get_control_weights();
        for (int c = 0; c < control_weights.getSize(); ++c) {
            const auto& control_weight = control_weights.get(c);
            effort->setWeight(control_weight.getName(),
                control_weight.getWeight());
        }
    } else {
        OPENSIM_THROW_IF(get_control_weights().getSize() != 0, Exception,
            "Control weight set provided, but control minimization was not "
            "specified by the user.");
    }

    // Set the time range.
    // -------------------
    // Set the time bounds based on the time range in the states file.
    // Pad the beginning and end time points to allow room for finite 
    // difference calculations.
    double pad = 1e-3;
    problem.setTimeBounds(m_timeInfo.initial + pad, m_timeInfo.final - pad);

    // Activation states.
    for (auto& musc : model.getComponentList<Muscle>()) {
        const auto& path = musc.getAbsolutePathString();
        problem.setStateInfo(format("%s/activation", path), {0, 1});
    }

    // Configure solver.
    // -----------------
    MocoCasADiSolver& solver = moco.initCasADiSolver();
    // TODO: Use MocoTool::mesh_interval property
    solver.set_num_mesh_points(25);
    solver.set_dynamics_mode("explicit");
    solver.set_optim_convergence_tolerance(1e-2);
    solver.set_optim_constraint_tolerance(1e-2);
    solver.set_enforce_constraint_derivatives(true);
    solver.set_transcription_scheme("hermite-simpson");
    solver.set_optim_finite_difference_scheme("forward");

    // Set the problem guess.
    // ----------------------
    checkPropertyInSet(*this, getProperty_guess_type(),
        {"bounds", "from_data", "from_file"});
    OPENSIM_THROW_IF(get_guess_type() == "from_file" &&
        get_guess_file().empty(), Exception,
        "Guess type set to 'from_file' but no guess file was provided.");

    if (get_guess_type() == "from_file") {
        solver.setGuessFile(getFilePath(get_guess_file()));
    } else if (get_guess_type() == "from_data") {
        auto guess = solver.createGuess("bounds");
        if (m_states_from_file.getNumRows()) {
            applyStatesToGuess(m_states_from_file, model, guess);
        } else if (m_states_from_markers.getNumRows()) {
            applyStatesToGuess(m_states_from_markers, model, guess);
        }
        solver.setGuess(guess);
    } else {
        solver.setGuess("bounds");
    }

    return moco;
}

void MocoTrack::solve() {
    MocoStudy moco = initialize();

    // Solve!
    // ------
    MocoSolution solution = moco.solve();
    moco.visualize(solution);
}

std::string MocoTrack::getFilePath(const std::string& file) const {
    using SimTK::Pathname;
    // Get the directory containing the setup file.
    std::string setupDir;
    {
        bool dontApplySearchPath;
        std::string fileName, extension;
        Pathname::deconstructPathname(getDocumentFileName(),
            dontApplySearchPath, setupDir, fileName, extension);
    }

    std::string filepath =
        Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(
            setupDir, file);

    return filepath;
}

void MocoTrack::configureStateTracking(MocoProblem& problem, Model& model) {

    // Read in the states reference data, filter, and spline.
    TimeSeriesTable statesRaw = readTableFromFile(get_states_tracking_file());
    TimeSeriesTable states;
    if (get_lowpass_cutoff_frequency_for_kinematics() != -1) {
        states = filterLowpass(statesRaw,
            get_lowpass_cutoff_frequency_for_kinematics(), true);
    } else {
        states = statesRaw;
    }
    auto stateSplines = GCVSplineSet(states, states.getColumnLabels());

    // Check that there are no redundant columns in the reference data.
    auto labelsSorted = states.getColumnLabels();
    std::sort(labelsSorted.begin(), labelsSorted.end());
    auto it = std::adjacent_find(labelsSorted.begin(), labelsSorted.end());
    OPENSIM_THROW_IF(it != labelsSorted.end(), Exception,
        "Multiple reference data provided for the same state variable.");

    // Loop through all coordinates and compare labels in the reference data
    // to coordinate variable names. 
    auto time = states.getIndependentColumn();
    auto labels = states.getColumnLabels();
    int numRefStates = (int)states.getNumColumns();
    MocoWeightSet weights;
    MocoWeightSet user_weights = get_state_weights();
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        std::string coordPath = coord.getAbsolutePathString();
        std::string valueName = coordPath + "/value";
        std::string speedName = coordPath + "/speed";
        std::string coordName = coord.getName();
        bool trackingValue = false;
        bool trackingSpeed = false;
        int valueIdx;
        for (int i = 0; i < numRefStates; ++i) {
            if (labels[i] == valueName) {
                trackingValue = true;
                valueIdx = i;
            }
            else if (labels[i] == coordName) {
                // Accept reference data labeled with coordinate names only, 
                // but set only for position data. Also, update the column label
                // to the complete state variable path.
                states.setColumnLabel(i, valueName);
                trackingValue = true;
                valueIdx = i;
            }
            else if (labels[i] == speedName) {
                trackingSpeed = true;
            }
        }

        // If a coordinate value was provided to track in the reference data, 
        // but no corresponding speed, append the derivative of the coordinate
        // value to the tracking reference.
        if (trackingValue && !trackingSpeed &&
            get_track_state_reference_derivatives()) {
            auto value = states.getDependentColumnAtIndex(valueIdx);
            auto* valueSpline = stateSplines.getGCVSpline(valueIdx);
            SimTK::Vector speed((int)time.size());
            for (int j = 0; j < (int)time.size(); ++j) {
                speed[j] = valueSpline->calcDerivative({0},
                    SimTK::Vector(1, time[j]));
            }
            states.appendColumn(speedName, speed);
        }

        // State variable tracking weights.
        bool valueWeightProvided = false;
        bool speedWeightProvided = false;
        for (int w = 0; w < user_weights.getSize(); ++w) {
            const auto& user_weight = user_weights.get(w);
            if (user_weight.getName() == valueName) {
                weights.cloneAndAppend(user_weight);
                valueWeightProvided = true;
            }
            else if (user_weight.getName() == coordName) {
                // Similar to reference data, accept weights labeled with 
                // coordinate names only, but set only for position data. Also, 
                // update the column label to the complete state variable path.
                weights.cloneAndAppend({valueName, user_weight.getWeight()});
                speedWeightProvided = true;
            }
            else if (user_weight.getName() == speedName) {
                weights.cloneAndAppend(user_weight);
                speedWeightProvided = true;
            }
        }

        // Unless the user already specified weights, don't track state
        // variables that are already constrained.
        double weight = coord.isConstrained(model.getWorkingState()) ? 0 : 1;
        if (!valueWeightProvided) {
            weights.cloneAndAppend({valueName, weight});
        }
        if (!speedWeightProvided) {
            weights.cloneAndAppend({speedName, weight});
        }
    }

    // Add state tracking cost to the MocoProblem.
    auto* stateTracking =
        problem.addCost<MocoStateTrackingCost>("state_tracking",
            get_states_tracking_weight());
    stateTracking->setReference(states);
    stateTracking->setWeightSet(weights);
    stateTracking->setAllowUnusedReferences(true);

    updateTimeInfo("states", states.getIndependentColumn().front(),
            states.getIndependentColumn().back(), m_timeInfo);

    // Update units before printing.
    if (states.hasTableMetaDataKey("inDegrees") &&
        states.getTableMetaDataAsString("inDegrees") == "yes") {
        model.getSimbodyEngine().convertDegreesToRadians(states);
    }
    // Write tracked states to file in case any label updates or filtering
    // occured.
    writeTableToFile(states, getName() + "_tracked_states.mot");
    m_states_from_file = states;
    if (m_min_data_length == -1 ||
        m_min_data_length > (int)states.getNumRows()) {
        m_min_data_length = (int)states.getNumRows();
    }
}

void MocoTrack::configureMarkerTracking(MocoProblem& problem, Model& model) {

    // Read in the markers reference data and filter.
    TimeSeriesTable_<SimTK::Vec3> markersRaw =
        readTableFromFileT<SimTK::Vec3>(get_markers_tracking_file());
    TimeSeriesTable markersRawFlat = markersRaw.flatten();
    TimeSeriesTable markersFlat;
    if (get_lowpass_cutoff_frequency_for_kinematics() != -1) {
        markersFlat = filterLowpass(markersRawFlat,
            get_lowpass_cutoff_frequency_for_kinematics(), true);
    } else {
        markersFlat = markersRawFlat;
    }
    TimeSeriesTable_<SimTK::Vec3> markers = markersFlat.pack<SimTK::Vec3>();
    MarkersReference markersRef(markers);

    // If the user provided an IK setup file, get the marker weights and add
    // them to the MarkersReference.
    if (!get_ik_setup_file().empty()) {
        InverseKinematicsTool iktool(get_ik_setup_file());
        iktool.setModel(model);
        auto& iktasks = iktool.getIKTaskSet();
        Set<MarkerWeight> markerWeights;
        iktasks.createMarkerWeightSet(markerWeights);
        markersRef.setMarkerWeightSet(markerWeights);

        // Create an initial guess with inverse kinematics if the guess_type 
        // property was set to "from_data".
        if (get_guess_type() == "from_data") {
            iktool.run();
            TimeSeriesTable states =
                readTableFromFile(iktool.getOutputMotionFileName());
            // Update state labels to full paths. 
            for (const auto& coord : model.getComponentList<Coordinate>()) {
                std::string path = coord.getAbsolutePathString();
                for (int i = 0; i < (int)states.getNumColumns(); ++i) {
                    if (path.find(states.getColumnLabel(i))
                        != std::string::npos) {
                        states.setColumnLabel(i, path + "/value");
                    }
                }
            }
            // Update units.
            if (states.hasTableMetaDataKey("inDegrees") &&
                states.getTableMetaDataAsString("inDegrees") == "yes") {
                model.getSimbodyEngine().convertDegreesToRadians(states);
            }
            m_states_from_markers = states;
        }
    }

    // Add marker tracking cost to the MocoProblem.
    auto* markerTracking =
        problem.addCost<MocoMarkerTrackingCost>("marking_tracking",
            get_markers_tracking_weight());
    markerTracking->setMarkersReference(markersRef);
    markerTracking->setAllowUnusedReferences(true);

    updateTimeInfo("markers", markers.getIndependentColumn().front(),
            markers.getIndependentColumn().back(), m_timeInfo);

    // Write tracked markers to file in case any label updates or filtering
    // occured.
    writeTableToFile(markers.flatten(), getName() + "_tracked_markers.mot");

    if (m_min_data_length == -1 ||
        m_min_data_length > (int)markers.getNumRows()) {
        m_min_data_length = (int)markers.getNumRows();
    }
}

void MocoTrack::applyStatesToGuess(const TimeSeriesTable& states,
    const Model& model, MocoIterate& guess) {

    guess.resampleWithNumTimes(m_min_data_length);
    auto time = guess.getTime();
    GCVSplineSet stateSplines(states);

    SimTK::Vector currTime(1);
    SimTK::Vector value(m_min_data_length);
    SimTK::Vector speed(m_min_data_length);
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        auto path = coord.getAbsolutePathString();
        for (int i = 0; i < (int)states.getNumColumns(); ++i) {
            auto label = states.getColumnLabel(i);
            if (path.find(label) != std::string::npos) {

                for (int j = 0; j < m_min_data_length; ++i) {
                    currTime[0] = time[j];
                    auto* spline = stateSplines.getGCVSpline(i);

                    value[j] = spline->calcValue(currTime);
                    speed[j] = spline->calcDerivative({0}, currTime);
                }

                guess.setState(path + "/value", value);
                guess.setState(path + "/speed", speed);
            }
        }
    }
}
