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

#include "MocoCasADiSolver/MocoCasADiSolver.h"
#include "MocoGoal/MocoControlGoal.h"
#include "MocoGoal/MocoMarkerTrackingGoal.h"
#include "MocoGoal/MocoStateTrackingGoal.h"
#include "MocoProblem.h"
#include "MocoStudy.h"
#include "MocoUtilities.h"
#include "MocoWeightSet.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/MarkersReference.h>

using namespace OpenSim;

void MocoTrack::constructProperties() {
    constructProperty_states_reference(TableProcessor());
    constructProperty_states_global_tracking_weight(1);
    constructProperty_states_weight_set(MocoWeightSet());
    constructProperty_scale_state_weights_with_range(false);
    constructProperty_track_reference_position_derivatives(false);
    constructProperty_markers_reference(TableProcessor());
    constructProperty_markers_global_tracking_weight(1);
    constructProperty_markers_weight_set(MocoWeightSet());
    constructProperty_allow_unused_references(false);
    constructProperty_guess_file("");
    constructProperty_apply_tracked_states_to_guess(false);
    constructProperty_minimize_control_effort(true);
    constructProperty_control_effort_weight(0.001);
}

MocoStudy MocoTrack::initialize() {

    MocoStudy study;
    study.setName(getName());
    MocoProblem& problem = study.updProblem();

    // Modeling.
    // ---------
    Model model = get_model().process(getDocumentDirectory());
    model.initSystem();

    // Goals.
    // ------
    // State tracking cost.
    TimeSeriesTable tracked_states;
    if (!get_states_reference().empty()) {
        tracked_states = configureStateTracking(problem, model);
    } else {
        OPENSIM_THROW_IF(get_apply_tracked_states_to_guess(), Exception,
                "Property 'apply_tracked_states_to_guess' was enabled, but no "
                "states reference data was provided.");
    }

    // Marker tracking cost.
    if (!get_markers_reference().empty()) {
        configureMarkerTracking(problem, model);
    }

    // Set the model on the MocoProblem, now that we're done configuring costs.
    problem.setModelAsCopy(model);

    // Control effort minimization.
    // ----------------------------
    if (get_minimize_control_effort()) {
        OPENSIM_THROW_IF(get_control_effort_weight() < 0, Exception,
                "Expected a non-negative control effort weight, "
                "but got a weight with value {}.",
                get_control_effort_weight());

        auto* effort = problem.addGoal<MocoControlGoal>("control_effort");
        effort->setWeight(get_control_effort_weight());
    }

    // Set the time range.
    // -------------------
    if (get_clip_time_range()) {
        m_timeInfo.initial += 1e-3;
        m_timeInfo.final -= 1e-3;
    }
    problem.setTimeBounds(m_timeInfo.initial, m_timeInfo.final);

    // Configure solver.
    // -----------------
    MocoCasADiSolver& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(m_timeInfo.numMeshIntervals);
    solver.set_multibody_dynamics_mode("explicit");
    solver.set_optim_convergence_tolerance(1e-2);
    solver.set_optim_constraint_tolerance(1e-2);
    solver.set_optim_finite_difference_scheme("forward");

    // Set the problem guess.
    // ----------------------
    // If the user provided a guess file, use that guess in the solver.
    if (!get_guess_file().empty()) {
        solver.setGuessFile(getFilePath(get_guess_file()));
    } else {
        solver.setGuess("bounds");
    }

    // Apply states from the reference data the to solver guess if specified by
    // the user.
    if (get_apply_tracked_states_to_guess()) {
        auto guess = solver.getGuess();
        applyStatesToGuess(tracked_states, guess);
        solver.setGuess(guess);
    }

    return study;
}

MocoSolution MocoTrack::solveInternal(bool visualize) {
    // Generate the base MocoStudy.
    MocoStudy study = initialize();

    // Solve!
    // ------
    MocoSolution solution = study.solve();
    if (visualize) { study.visualize(solution); }

    return solution;
}

TimeSeriesTable MocoTrack::configureStateTracking(
        MocoProblem& problem, Model& model) {

    // Read in the states reference data and spline.
    // TODO convert Degrees to Radians.
    TimeSeriesTable states = get_states_reference().processAndConvertToRadians(
            getDocumentDirectory(), model);
    auto stateSplines = GCVSplineSet(states, states.getColumnLabels());

    // Loop through all coordinates and compare labels in the reference data
    // to coordinate variable names.
    auto time = states.getIndependentColumn();
    auto labels = states.getColumnLabels();
    int numRefStates = (int)states.getNumColumns();
    MocoWeightSet weights;
    MocoWeightSet user_weights = get_states_weight_set();
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        std::string coordPath = coord.getAbsolutePathString();
        std::string valueName = coordPath + "/value";
        std::string speedName = coordPath + "/speed";
        bool trackingValue = false;
        bool trackingSpeed = false;
        int valueIdx = -1;
        for (int i = 0; i < numRefStates; ++i) {
            if (labels[i] == valueName) {
                trackingValue = true;
                valueIdx = i;
            } else if (labels[i] == speedName) {
                trackingSpeed = true;
            }
        }

        // If a coordinate value was provided to track in the reference data,
        // but no corresponding speed, append the derivative of the coordinate
        // value to the tracking reference.
        if (trackingValue && !trackingSpeed &&
                get_track_reference_position_derivatives()) {
            auto value = states.getDependentColumnAtIndex(valueIdx);
            auto* valueSpline = stateSplines.getGCVSpline(valueIdx);
            SimTK::Vector speed((int)time.size());
            for (int j = 0; j < (int)time.size(); ++j) {
                speed[j] = valueSpline->calcDerivative(
                        {0}, SimTK::Vector(1, time[j]));
            }
            states.appendColumn(speedName, speed);
            trackingSpeed = true;
        }

        // Unless the user already specified weights, don't track state
        // variables that are already constrained.
        bool isConstrained = coord.isConstrained(model.getWorkingState());
        // Value weights.
        if (user_weights.contains(valueName)) {
            auto uw = user_weights.get(valueName);
            weights.cloneAndAppend({valueName, uw.getWeight()});
        } else if (isConstrained) {
            weights.cloneAndAppend({valueName, 0});
        }
        // Speed weights.
        if (trackingSpeed) {
            if (user_weights.contains(speedName)) {
                auto uw = user_weights.get(speedName);
                weights.cloneAndAppend({speedName, uw.getWeight()});
            } else if (isConstrained) {
                weights.cloneAndAppend({speedName, 0});
            }
        }
    }

    // Add state tracking cost to the MocoProblem.
    auto* stateTracking = problem.addGoal<MocoStateTrackingGoal>(
            "state_tracking", get_states_global_tracking_weight());
    stateTracking->setReference(states);
    stateTracking->setWeightSet(weights);
    stateTracking->setAllowUnusedReferences(get_allow_unused_references());
    stateTracking->setScaleWeightsWithRange(
            get_scale_state_weights_with_range());

    // Update the time info struct.
    updateTimeInfo("states", states.getIndependentColumn().front(),
            states.getIndependentColumn().back(), m_timeInfo);

    // Write tracked states to file in case any label updates or filtering
    // occurred.
    STOFileAdapter::write(states, getName() + "_tracked_states.sto");

    // Return tracked states to possibly include in the guess.
    return states;
}

void MocoTrack::configureMarkerTracking(MocoProblem& problem, Model& model) {

    // Read in the markers reference data.
    TimeSeriesTable markersFlat =
            get_markers_reference().process(getDocumentDirectory(), &model);
    TimeSeriesTable_<SimTK::Vec3> markers = markersFlat.pack<SimTK::Vec3>();
    MarkersReference markersRef(markers, Set<MarkerWeight>());

    // If the user provided marker weights, append them to the markers
    // reference.
    if (get_markers_weight_set().getSize()) {
        Set<MarkerWeight> markerWeights;
        for (int i = 0; i < get_markers_weight_set().getSize(); ++i) {
            const auto& weight = get_markers_weight_set().get(i);
            markerWeights.cloneAndAppend(
                    MarkerWeight(weight.getName(), weight.getWeight()));
        }
        markersRef.setMarkerWeightSet(markerWeights);
    }

    // Add marker tracking cost to the MocoProblem.
    auto* markerTracking = problem.addGoal<MocoMarkerTrackingGoal>(
            "marker_tracking", get_markers_global_tracking_weight());
    markerTracking->setMarkersReference(markersRef);
    markerTracking->setAllowUnusedReferences(get_allow_unused_references());

    // Update the time info struct.
    updateTimeInfo("markers", markers.getIndependentColumn().front(),
            markers.getIndependentColumn().back(), m_timeInfo);

    // Write tracked markers to file in case any label updates or filtering
    // occurred.
    STOFileAdapter::write(markers.flatten(),
            getName() + "_tracked_markers.sto");
}

void MocoTrack::applyStatesToGuess(
        const TimeSeriesTable& states, MocoTrajectory& guess) const {

    guess.resampleWithNumTimes((int)states.getNumRows());
    std::vector<std::string> names = guess.getStateNames();
    for (int i = 0; i < (int)states.getNumColumns(); ++i) {
        const auto& label = states.getColumnLabel(i);
        const auto& col = states.getDependentColumnAtIndex(i);

        if (std::find(names.begin(), names.end(), label) != names.end()) {
            guess.setState(label, col);
        } else {
            OPENSIM_THROW_IF(!get_allow_unused_references(), Exception,
                    "Tried to apply data for state '{}' to guess, "
                    "but this state does not exist in the model.",
                    label);
        }
    }
}
