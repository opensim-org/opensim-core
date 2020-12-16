#ifndef OPENSIM_INVERSEMUSCLESOLVERMOTIONDATA_H
#define OPENSIM_INVERSEMUSCLESOLVERMOTIONDATA_H
/* -------------------------------------------------------------------------- *
 * OpenSim: InverseMuscleSolverMotionData.h                                   *
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

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>

// TODO should not expose Eigen. This whole file could be private.
// or forward declare MatrixXd.
#include "../../Moco/osimMocoDLL.h"
#include <Eigen/Dense>

namespace OpenSim {

class Model;
class Coordinate;

/// This class contains the motion data required to solve inverse problems
/// for muscle activity:
///   - inverse dynamics net joint moments
///   - muscle-tendon lengths
///   - moment arms
/// This class also allows interpolating these data for use by solvers.
class OSIMMOCO_API InverseMuscleSolverMotionData {
public:
    InverseMuscleSolverMotionData() = default;
    /// From the given kinematics trajectory (joint angles), this constructor
    /// will perform inverse dynamics and a muscle analysis to provide net
    /// joint moments, muscle-tendon lengths, and moment arms.
    /// The coordsToActuate must be listed in multibody tree order.
    /// The inverse dynamics moments are filtered with the provided lowpass
    /// cutoff frequency; use -1 to not filter.
    InverseMuscleSolverMotionData(const Model& model,
            const std::vector<const Coordinate*>& coordsToActuate,
            const double& initialTime, const double& finalTime,
            const TimeSeriesTable& kinematicsData,
            const double& lowpassCutoffKinematics,
            const double& lowpassCutoffJointMoments);
    /// From the given kinematics trajectory (joint angles), this constructor
    /// will perform a muscle analysis to provide net
    /// joint moments, muscle-tendon lengths, and moment arms.
    /// The coordsToActuate must be listed in multibody tree order.
    /// The inverse dynamics net generalized forces are taken from the provided
    /// table rather than computed from the provided kinematics.
    InverseMuscleSolverMotionData(const Model& model,
            const std::vector<const Coordinate*>& coordsToActuate,
            const double& initialTime, const double& finalTime,
            const TimeSeriesTable& kinematicsData,
            const double& lowpassCutoffKinematics,
            const double& lowpassCutoffJointMoments,
            const TimeSeriesTable& netGeneralizedForcesData);

    /// Get the paths (relative to the model) of the coordinates that are to be
    /// actuated by an InverseMuscleSolver.
    const std::vector<std::string>& getCoordinatesToActuate() const
    { return _coordPathsToActuate; }
    /// Get the initial time to use in the solver.
    double getInitialTime() const { return _initialTime; }
    /// Get the final time to use in the solver.
    double getFinalTime() const { return _finalTime; }
    /// Interpolate the desired net joint moments at the provided times.
    /// @param times The times at which to interpolate; must be between the
    ///     initial and final times.
    /// @param[in,out] desiredNetGenForces Inverse dynamics net joint moments
    ///     Dimensions: degrees of freedom x time.
    void interpolateNetGeneralizedForces(const Eigen::VectorXd& times,
            Eigen::MatrixXd& desiredNetGenForces) const;
    /// Interpolate the muscle-tendon lengths at the provided times.
    /// @param times The times at which to interpolate; must be between the
    ///     initial and final times.
    /// @param[in,out] muscleTendonLengths Expressed in meters.
    ///     Dimensions: muscles x time.
    void interpolateMuscleTendonLengths(const Eigen::VectorXd& times,
            Eigen::MatrixXd& muscleTendonLengths) const;
    /// Interpolate the muscle-tendon velocities at the provided times. These
    /// are computed using the first derivative of the muscle-tendon length
    /// splines.
    /// @param times The times at which to interpolate; must be between the
    ///     initial and final times.
    /// @param[in,out] muscleTendonVelocities Expressed in meters/second,
    ///     positive for lengthening.
    ///     Dimensions: muscles x time.
    void interpolateMuscleTendonVelocities(const Eigen::VectorXd& times,
            Eigen::MatrixXd& muscleTendonVelocities) const;
    /// Interpolate the moment arms at the provided times.
    /// @param times The times at which to interpolate; must be between the
    ///     initial and final times.
    /// @param[in,out] momentArms Elements of the std::vector correspond to
    ///     times, and each element of the std::vectorhas dimensions
    ///     (degrees of freedom) x muscles
    void interpolateMomentArms(const Eigen::VectorXd& times,
            std::vector<Eigen::MatrixXd>& momentArms) const;
private:
    /// This constructor performs the muscle analysis.
    /// The coordsToActuate must be listed in multibody tree order.
    InverseMuscleSolverMotionData(const Model& model,
            const std::vector<const Coordinate*>& coordsToActuate,
            const double& initialTime, const double& finalTime,
            const TimeSeriesTable& kinematicsData,
            const double& lowpassCutoffKinematics);
    std::vector<std::string> createCoordPathsToActuate(const Model& model,
            const std::vector<const Coordinate*>& coordsToActuate) const;
    /// The coordPathsToActuate must be listed in multibody tree order.
    void computeInverseDynamics(const OpenSim::Model& model,
            const std::vector<std::string>& coordPathsToActuate,
            const TimeSeriesTable& kinematicsData,
            const double& lowpassCutoffJointMoments);
    std::vector<std::string> _coordPathsToActuate;
    double _initialTime;
    double _finalTime;
    size_t _numCoordsToActuate;
    size_t _numActiveMuscles;
    GCVSplineSet _netGeneralizedForces;
    GCVSplineSet _muscleTendonLengths;
    // GCVSplineSet _muscleTendonVelocities;
    // The vector contains an entry for each coordinate; the spline set is
    // across muscles.
    std::vector<GCVSplineSet> _momentArms;
};

} // namespace OpenSim

#endif // OPENSIM_INVERSEMUSCLESOLVERMOTIONDATA_H
