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

namespace OpenSim {

class MocoWeightSet;
class MocoProblem;
class MocoIterate;

/// This tool constructs problems in which any combination of state trajectory
/// data, marker trajectory data, or external force data is tracked while
/// solving for the model's kinematics and actuator controls in a prescribed
/// time window. It is upon the user to ensure that the tracking data and model
/// provided are consistent, but this tool will try to construct a valid problem 
/// for certain assumed data formats.

// TODO allowing extra columns for everything
// TODO "from_data", states data will take precedence over data from markers
class OSIMMOCO_API MocoTrack : public MocoTool {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoTrack, MocoTool);

public:

    OpenSim_DECLARE_PROPERTY(states_tracking_file, std::string,
        "Path to a STO file containing reference state variable data "
        "to track via a MocoStateTrackingCost. "
        "The path can be absolute or relative to the setup file."
        "If the state file columns are labeled using only model "
        "coordinate names, it is assumed that position-level state "
        "should track this data.");

    OpenSim_DECLARE_PROPERTY(states_tracking_weight, double,
        "The weight for the MocoStateTrackingCost. ");

    OpenSim_DECLARE_PROPERTY(state_weights, MocoWeightSet,
        "A set of tracking weights for individual state variables. The "
        "weight names should match the names of the column labels in the "
        "file associated with the 'states_file' property.");

    OpenSim_DECLARE_PROPERTY(track_state_reference_derivatives, bool,
        "Option to track the derivative of position-level state reference "
        "data if no velocity-level state reference data was included in "
        "the `states_file`. If speed reference data was provided for some "
        "coordinates but not others, this option will only apply to the "
        "coordinates without speed reference data. "
        "(default: false)");

    OpenSim_DECLARE_PROPERTY(markers_tracking_file, std::string,
        "Path to a STO file containing reference marker data to track "
        "via a MocoMarkerTrackingCost. "
        "The path can be absolute or relative to the setup file.");

    OpenSim_DECLARE_PROPERTY(markers_tracking_weight, double,
        "The weight for the MocoMarkerTrackingCost. ");

    OpenSim_DECLARE_PROPERTY(ik_setup_file, std::string,
        "Path to an OpenSim::InverseKinematicsTool setup file. This can "
        "be used to specify individual tracking weights for markers in "
        "problem. It is also used to create an initial guess for the state "
        "variables when the 'guess_type' property is set to 'from_data'.");

    OpenSim_DECLARE_PROPERTY(lowpass_cutoff_frequency_for_kinematics, double,
        "The frequency (Hz) at which to filter the kinematics "
        "(markers and states). "
        "(default is -1, which means no filtering; for walking, "
        "consider 6 Hz).");

    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string,
        "Path to an XML file describing ExternalForces to be tracked or "
        "applied to the model.");

    OpenSim_DECLARE_PROPERTY(guess_type, std::string,
        "Options: 'bounds', 'from_data', or 'from_file'. "
        "'bounds' uses variable bound midpoint values to create an initial "
        "guess. 'from_data' creates an initial guess with the data "
        "provided. 'from_file' creates an initial guess from the file set "
        "on the 'guess_file' property (see below)."
        "(default: 'bounds').");

    OpenSim_DECLARE_PROPERTY(guess_file, std::string,
        "Path to a STO file containing reference marker data to track. "
        "The path can be absolute or relative to the setup file.");

    OpenSim_DECLARE_PROPERTY(minimize_controls, double,
        "Whether or not to minimize actuator controls in the problem. The "
        "property value enabling the control cost is the weight passed to "
        "the internal MocoControlCost."
        "(default: -1, meaning no control cost.");

    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
        "Individual control weights to be applied to the MocoControlCost "
        "in the problem (if enabled by the 'minimize_controls' property).");

    MocoTrack() { constructProperties(); }

    void setModel(Model model) { m_model = std::move(model); }

    MocoStudy initialize();
    void solve();

private:
    Model m_model;
    TimeInfo m_timeInfo;
    TimeSeriesTable m_states_from_file;
    TimeSeriesTable m_states_from_markers;
    int m_min_data_length;

    void constructProperties();

    // Utilities.
    std::string getFilePath(const std::string& file) const;

    // Cost configuration methods.
    void configureStateTracking(MocoProblem& problem, Model& model);
    void configureMarkerTracking(MocoProblem& problem, Model& model);

    // Convenience methods.
    void applyStatesToGuess(const TimeSeriesTable& states, const Model& model,
        MocoIterate& guess);
};

} // namespace OpenSim

#endif // MOCO_MOCOTRACK_H
