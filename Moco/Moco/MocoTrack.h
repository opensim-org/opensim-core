#ifndef MOCO_MOCOTRACK_H
#define MOCO_MOCOTRACK_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTrack.h                                                  *
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

#include "osimMocoDLL.h"
#include "MocoStudy.h"
#include "MocoTool.h"

#include <OpenSim/Simulation/Model/Model.h>
#include "Common/TableProcessor.h"

namespace OpenSim {

class MocoWeightSet;
class MocoProblem;
class MocoIterate;

/// This tool constructs problems in which state and/or marker trajectory data 
/// are tracked while solving for model kinematics and actuator controls. 
/// "Tracking" refers to cost terms that minimize the error between provided 
/// reference data and the associated model quantities (joint angles, joint 
/// velocities, marker positions, etc). 
///
/// State reference data (joint angles and velocities), marker reference data 
/// (x/y/z marker motion capture trajectories), or both may be provided via the
/// `states_reference` and `markers_reference` properties. For each set of 
/// reference data provided, a tracking cost term is added to the internal 
/// MocoProblem. 
///
/// A time range that is compatible with all reference data may be provided. 
/// If no time range is set, the widest time range that is compatible will all 
/// reference data will be used. 
///
/// The `states_global_tracking_weight` and `markers_global_tracking_weight` 
/// properties apply a cost function weight to all tracking error associated 
/// provided reference data. The `states_weight_set` and `markers_weight_set`
/// properties give you finer control over the tracking costs, letting you set
/// weights for individual reference data tracking errors.
///     
/// If you would like to track joint velocities but only have joint angles in 
/// your states reference, enable the `track_reference_position_derivatives` 
/// property. When enabled, the provided position-level states reference data 
/// will be splined in order to compute derivatives. If some velocity-level 
/// information exists in the reference, this option will fill in the missing 
/// data with position derivatives and leave the existing velocity data intact. 
///
/// Default solver settings:
///  - Explicit dynamics
///  - Constraint tolerance: 1e-2
///  - Convergence tolerance: 1e-2
///  - Finite difference scheme: 'forward'
/// 
/// @underdevelopment
class OSIMMOCO_API MocoTrack : public MocoTool {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoTrack, MocoTool);

public:
    OpenSim_DECLARE_PROPERTY(states_reference, TableProcessor,
        "States reference data to be tracked. If provided, a "
        "MocoStateTrackingCost term is created and added to the internal "
        "MocoProblem. ");

    OpenSim_DECLARE_PROPERTY(states_global_tracking_weight, double,
        "The weight for the MocoStateTrackingCost which applies to tracking " 
        "errors for all states in the reference.");

    OpenSim_DECLARE_PROPERTY(states_weight_set, MocoWeightSet,
        "A set of tracking weights for individual state variables. The "
        "weight names should match the names of the column labels in the "
        "file associated with the 'states_reference' property.");

    OpenSim_DECLARE_PROPERTY(track_reference_position_derivatives, bool,
        "Option to track the derivative of position-level state reference "
        "data if no velocity-level state reference data was included in "
        "the `states_reference`. If velocity-reference reference data was "
        "provided for some coordinates but not others, this option will only "
        "apply to the coordinates without speed reference data. "
        "(default: false)");

    OpenSim_DECLARE_PROPERTY(markers_reference, TableProcessor,
        "Motion capture marker reference data to be tracked. If provided, a "
        "MocoMarkerTrackingCost term is create and added to the internal "
        "MocoProblem.");

    OpenSim_DECLARE_PROPERTY(markers_global_tracking_weight, double,
        "The weight for the MocoMarkerTrackingCost which applies to tracking "
        "errors for all markers in the reference.");

    OpenSim_DECLARE_PROPERTY(markers_weight_set, MocoWeightSet,
        "A set of tracking weights for individual marker positions. The "
        "weight names should match the names of the column labels in the "
        "file associated with the 'markers_reference' property.");

    OpenSim_DECLARE_PROPERTY(guess_file, std::string,
        "Path to a STO file containing a guess for the problem. The path can "
        "be absolute or relative to the setup file. If no file is provided, "
        "then a guess constructed from the variable bounds midpoints will be "
        "used.");

    OpenSim_DECLARE_PROPERTY(apply_tracked_states_to_guess, bool,
        "If a `states_reference` has been provided, use this setting to "
        "replace the states in the guess with the states reference data. This "
        "will override any guess information provided via `guess_file`.");

    OpenSim_DECLARE_PROPERTY(minimize_controls, double,
        "Whether or not to minimize actuator controls in the problem. The "
        "property value enabling the control cost is the weight passed to "
        "the internal MocoControlCost."
        "(default: -1, meaning no control cost.");

    MocoTrack() { constructProperties(); }

    void setStatesReference(TableProcessor states) {
        set_states_reference(std::move(states));
    }
    void setMarkersReference(TableProcessor markers) {
        set_markers_reference(std::move(markers));
    }

    MocoStudy initialize();
    MocoSolution solve();

private:
    Model m_model;
    TimeInfo m_timeInfo;

    void constructProperties();

    // Cost configuration methods.
    // ---------------------------
    TimeSeriesTable configureStateTracking(MocoProblem& problem, Model& model);
    void configureMarkerTracking(MocoProblem& problem, Model& model);

    // Convenience methods.
    // --------------------
    std::string getFilePath(const std::string& file) const;
    void applyStatesToGuess(const TimeSeriesTable& states, const Model& model,
        MocoIterate& guess);
};

} // namespace OpenSim

#endif // MOCO_MOCOTRACK_H
