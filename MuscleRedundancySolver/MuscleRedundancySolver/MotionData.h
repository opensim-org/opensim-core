#ifndef TOMU_MOTIONDATA_H
#define TOMU_MOTIONDATA_H

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>

#include <Eigen/Dense>

namespace OpenSim {

class Model;

/// This class contains the motion data required to solve inverse problems
/// for muscle activity:
///   - inverse dynamics net joint moments
///   - muscle-tendon lengths
///   - moment arms
/// This class also allows interpolating these data for use by solvers.
class MotionData {
public:
    /// From the given kinematics trajectory (joint angles), this constructor
    /// will perform inverse dynamics and a muscle analysis to provide net
    /// joint moments, muscle-tendon lengths, and moment arms.
    /// The inverse dynamics moments are filtered with the provided lowpass
    /// cutoff frequency; use -1 to not filter.
    MotionData(const OpenSim::Model& model,
               const OpenSim::TimeSeriesTable& kinematicsData,
               const double& lowpassCutoffJointMoments);
    /// Get the first time in the kinematicsData table.
    double getInitialTime() const { return _initialTime; }
    /// Get the last time in the kinematicsData table.
    double getFinalTime() const { return _finalTime; }
    /// Interpolate the desired net joint moments and the muscle-tendon
    /// lengths at the provided times.
    /// @param times The times at which to interpolate; must be between the
    ///     initial and final times.
    /// @param[in,out] desiredMoments Inverse dynamics net joint moments
    ///     Dimensions: degrees of freedom by time.
    /// @param[in,out] muscleTendonLengths Expressed in meters.
    ///     Dimensions: muscles by time.
    void interpolate(const Eigen::VectorXd& times,
                     Eigen::MatrixXd& desiredMoments,
                     Eigen::MatrixXd& muscleTendonLengths) const;
private:
    const OpenSim::TimeSeriesTable& _kinematicsData;
    double _initialTime;
    double _finalTime;
    GCVSplineSet _inverseDynamics;
    GCVSplineSet _muscleTendonLengths;
};

} // namespace OpenSim

#endif // TOMU_MOTIONDATA_H
