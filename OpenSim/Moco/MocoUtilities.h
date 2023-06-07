#ifndef OPENSIM_MOCOUTILITIES_H
#define OPENSIM_MOCOUTILITIES_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoUtilities.h                                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

#include "MocoTrajectory.h"
#include "osimMocoDLL.h"
#include <condition_variable>
#include <regex>

#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Logger.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>
#include <OpenSim/Simulation/StatesTrajectory.h>

namespace OpenSim {

class StatesTrajectory;
class Model;
class MocoTrajectory;
class MocoProblem;

/// Calculate the requested outputs using the model in the problem and the
/// provided StatesTrajectory and controls table.
/// The controls table is used to set the model's controls vector.
/// We assume the StatesTrajectory and controls table contain the same time
/// points.
/// The output paths can be regular expressions. For example,
/// ".*activation" gives the activation of all muscles.
///
/// The output paths must correspond to outputs that match the type provided in
/// the template argument, otherwise they are not included in the report.
///
/// @note The provided trajectory is not modified to satisfy kinematic
/// constraints, but SimTK::Motions in the Model (e.g., PositionMotion) are
/// applied. Therefore, this function expects that you've provided a trajectory
/// that already satisfies kinematic constraints. If your provided trajectory
/// does not satisfy kinematic constraints, many outputs will be incorrect.
/// For example, in a model with a patella whose location is determined by a
/// CoordinateCouplerConstraint, the length of a muscle that crosses the patella
/// will be incorrect.
///
/// @note Parameters and Lagrange multipliers in the MocoTrajectory are **not**
///       applied to the model.
/// @ingroup mocoutil
template <typename T>
TimeSeriesTable_<T> analyzeMocoTrajectory(
        Model model, const MocoTrajectory& trajectory,
        const std::vector<std::string>& outputPaths) {
    const TimeSeriesTable statesTable = trajectory.exportToStatesTable();
    const TimeSeriesTable controlsTable = trajectory.exportToControlsTable();
    const TimeSeriesTable derivativesWithoutAccelerationsTable =
            trajectory.exportToDerivativesWithoutAccelerationsTable();
    return analyze<T>(
            std::move(model), statesTable, controlsTable, outputPaths,
            derivativesWithoutAccelerationsTable);
}

/// Given a MocoTrajectory and the associated OpenSim model, return the model
/// with a prescribed controller appended that will compute the control values
/// from the MocoTrajectory. This can be useful when computing state-dependent
/// model quantities that require realization to the Dynamics stage or later.
/// The function used to fit the controls can either be GCVSpline or
/// PiecewiseLinearFunction.
/// @ingroup mocoutil
OSIMMOCO_API void prescribeControlsToModel(const MocoTrajectory& trajectory,
        Model& model, std::string functionType = "GCVSpline");

/// Use the controls and initial state in the provided trajectory to simulate
/// the model using an ODE time stepping integrator (OpenSim::Manager), and
/// return the resulting states and controls. We return a MocoTrajectory (rather
/// than a StatesTrajectory) to facilitate comparing optimal control solutions
/// with time stepping. Use integratorAccuracy to override the default setting.
///
/// @note This function expects all Actuator%s in the model to be in the Model's
/// ForceSet.
/// @ingroup mocoutil
OSIMMOCO_API MocoTrajectory simulateTrajectoryWithTimeStepping(
        const MocoTrajectory& trajectory, Model model,
        double integratorAccuracy = SimTK::NaN);

/// Convert a trajectory covering half the period of a symmetric motion into a
/// trajectory over the full period. This is useful for simulations of half a
/// gait cycle.
/// This converts time, states, controls, and derivatives; all other quantities
/// from the input trajectory are ignored.
/// If a column in the trajectory does not match addPatterns, negatePatterns,
/// negateAndShiftPatterns, or symmetryPatterns, then the second half of the
/// period contains the same data as the first half.
///
/// @param halfPeriodTrajectory The input trajectory covering half a period.
/// @param addPatterns If a column label matches an addPattern, then the second
/// half of the period for that column is (first_half_trajectory +
/// half_period_value - initial_value).
/// @param negatePatterns If a column label matches a negatePattern, then the
/// second half of the period for that column is (-first_half_trajectory).
/// This is usually relevant for only 3D models.
/// @param negateAndShiftPatterns If a column label matches a
/// negateAndShiftPattern, then the second half of the period for that column is
/// (-first_half_trajectory + 2 * half_period_value). This is usually relevant
/// for only 3D models.
/// @param symmetryPatterns This argument is a list of pairs, where the first
/// element of the pair is a pattern to match, and the second is a substitution
/// to convert the column label into the opposite column label of the symmetric
/// pair. If a column label matches a symmetryPattern, then its first
/// half-period is copied into the second half of the period for the column
/// identified by the substitution.
///
/// The default values for the patterns are intended to handle the column labels
/// for typical 2D or 3D OpenSim gait models.
/// The default values for negatePatterns, negateAndShiftPatterns, and
/// symmetryPatterns warrant an explanation. The string pattern before the
/// regex "(?!/value)" is followed by
/// anything except "/value" since it is contained in the negative lookahead
/// "(?!...)".  R"()" is a string literal that permits us to not escape
/// backslash characters. The regex "_r(\/|_|$)" matches "_r" followed by either
/// a forward slash (which is escaped), an underscore, OR the end of the string
/// ($). Since the forward slash and end of the string are within parentheses,
/// whatever matches this is captured and is available in the substitution (the
/// second element of the pair) as $1. The default symmetry patterns cause the
/// following replacements:
/// - "/jointset/hip_r/hip_flexion_r/value" becomes "/jointset/hip_l/hip_flexion_l/value"
/// - "/forceset/soleus_r" becomes "/forceset/soleus_l"
/// @ingroup mocoutil
OSIMMOCO_API MocoTrajectory createPeriodicTrajectory(
        const MocoTrajectory& halfPeriodTrajectory,
        std::vector<std::string> addPatterns = {".*pelvis_tx/value"},
        std::vector<std::string> negatePatterns = {
                                            ".*pelvis_list(?!/value).*",
                                            ".*pelvis_rotation.*",
                                            ".*pelvis_tz(?!/value).*",
                                            ".*lumbar_bending(?!/value).*",
                                            ".*lumbar_rotation.*"},
        std::vector<std::string> negateAndShiftPatterns = {
                                                   ".*pelvis_list/value",
                                                   ".*pelvis_tz/value",
                                                   ".*lumbar_bending/value"},
        std::vector<std::pair<std::string, std::string>> symmetryPatterns =
                {{R"(_r(\/|_|$))", "_l$1"}, {R"(_l(\/|_|$))", "_r$1"}});

/// This obtains the value of the OPENSIM_MOCO_PARALLEL environment variable.
/// The value has the following meanings:
/// - 0: run in series (not parallel).
/// - 1: run in parallel using all cores.
/// - greater than 1: run in parallel with this number of parallel jobs.
/// If the environment variable is not set, this function returns -1.
///
/// This variable does not indicate which calculations are parallelized
/// or how the parallelization is achieved. Moco may even ignore or override
/// the setting from the environment variable. See documentation elsewhere
/// (e.g., from a specific MocoSolver) for more information.
/// @ingroup mocoutil
OSIMMOCO_API int getMocoParallelEnvironmentVariable();

/// Thrown by FileDeletionThrower::throwIfDeleted().
/// @ingroup mocoutil
class FileDeletionThrowerException : public Exception {
public:
    FileDeletionThrowerException(const std::string& file, size_t line,
            const std::string& func, const std::string& deletedFile)
            : Exception(file, line, func) {
        addMessage("File '" + deletedFile + "' deleted.");
    }
};
/// This class helps a user cause an exception within the code. The constructor
/// writes a file, and the destructor deletes the file. The programmer can call
/// throwIfDeleted() to throw the FileDeletionThrowerException exception if the
/// file is deleted (by a user) before the object is destructed. If the file
/// could not be written by the constructor, then throwIfDeleted() does not
/// throw an exception.
/// @ingroup mocoutil
class FileDeletionThrower {
public:
    FileDeletionThrower()
            : FileDeletionThrower(
                      "OpenSimMoco_delete_this_to_throw_exception_" +
                      getFormattedDateTime() + ".txt") {}
    FileDeletionThrower(std::string filepath)
            : m_filepath(std::move(filepath)) {
        std::ofstream f(m_filepath);
        m_wroteInitialFile = f.good();
        f.close();
    }
    ~FileDeletionThrower() {
        if (m_wroteInitialFile) {
            std::ifstream f(m_filepath);
            if (f.good()) {
                f.close();
                std::remove(m_filepath.c_str());
            }
        }
    }
    void throwIfDeleted() const {
        if (m_wroteInitialFile) {
            OPENSIM_THROW_IF(!std::ifstream(m_filepath).good(),
                    FileDeletionThrowerException, m_filepath);
        }
    }

private:
    bool m_wroteInitialFile = false;
    const std::string m_filepath;
};

/// Obtain the ground reaction forces, centers of pressure, and torques
/// resulting from Force elements (e.g., SmoothSphereHalfSpaceForce), using a
/// model and states trajectory. Forces and torques are expressed in the ground
/// frame with respect to the ground origin. Paths to Force elements should be
/// provided separately for elements of the right and left feet. The output is a
/// table formatted for use with OpenSim tools; the labels of the columns
/// distinguish between right ("<>_r") and left ("<>_l") forces, centers of
/// pressure, and torques. Centers of pressure are computed assuming the
/// that the contact plane's normal is in the y-direction, which is the OpenSim
/// convention.
///
/// The forces and torques are computed from the first six outputs of
/// getRecordValues(), while the centers of pressure are computed from the second
/// six outputs. The first six outputs should correspond to the contact force
/// components applied to the foot bodies (e.g., the "sphere" forces in
/// SmoothSphereHalfSpaceForce), and the second six outputs should correspond to
/// the contact force components applied to the contact place (e.g., the
/// "half-space" forces in SmoothSphereHalfSpaceForce). The contact plane is
/// often attached to ground for foot-ground contact models, but it need not be,
/// as long as the contact plane normal is in the y-direction.
///
/// In general, this utility needs getRecordValues() to report the
/// following force and torque information at the specified indices:
///
/// index - component (body)
/// ------------------------
///     0 - force-x (foot)
///     1 - force-y (foot)
///     2 - force-z (foot)
///     3 - torque-x (foot)
///     4 - torque-y (foot)
///     5 - torque-z (foot)
///     6 - force-x (contact plane)
///     7 - force-y (contact plane)
///     8 - force-z (contact plane)
///     9 - torque-x (contact plane)
///    10 - torque-y (contact plane)
///    11 - torque-z (contact plane)
///
/// @ingroup mocoutil
OSIMMOCO_API
TimeSeriesTable createExternalLoadsTableForGait(Model model,
        const StatesTrajectory& trajectory,
        const std::vector<std::string>& forcePathsRightFoot,
        const std::vector<std::string>& forcePathsLeftFoot);

/// Same as above, but with a MocoTrajectory instead of a StatesTrajectory.
/// @ingroup mocoutil
OSIMMOCO_API
TimeSeriesTable createExternalLoadsTableForGait(Model model,
        const MocoTrajectory& trajectory,
        const std::vector<std::string>& forcePathsRightFoot,
        const std::vector<std::string>& forcePathsLeftFoot);

} // namespace OpenSim

#endif // OPENSIM_MOCOUTILITIES_H
