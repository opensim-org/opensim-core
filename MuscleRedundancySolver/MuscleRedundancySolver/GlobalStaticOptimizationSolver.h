#ifndef TOMU_GLOBALSTATICOPTIMIZATIONSOLVER_H
#define TOMU_GLOBALSTATICOPTIMIZATIONSOLVER_H

#include <OpenSim/OpenSim.h>

namespace OpenSim {

// TODO document
class GlobalStaticOptimizationSolver : public OpenSim::Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(GlobalStaticOptimizationSolver,
            OpenSim::Object);
public:

    OpenSim_DECLARE_PROPERTY(lowpass_cutoff_frequency_for_joint_moments, double,
    "The frequency (Hz) at which to filter inverse dynamics joint moments, "
    "which are computed internally from the kinematics (default is -1, "
    "which means no filtering; for walking, consider 6 Hz).");

    OpenSim_DECLARE_PROPERTY(create_reserve_actuators, double,
    "Create a reserve actuator (CoordinateActuator) for each unconstrained "
    "coordinate in the model, and add each to the model. Each actuator "
    "will have the specified `optimal_force`, which should be set low to "
    "discourage the use of the reserve actuators. (default is -1, which "
    "means no reserves are created)");

    // TODO rename to iterate?
    struct Solution {
        /// The activation trajectories for all enabled (appliesForce) muscles.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable activation;
        /// The control for enabled (appliesForce) CoordinateActuators, etc.
        /// This will be empty if there are no CoordinateActuators, etc.
        /// enabled.
        TimeSeriesTable other_controls;
        // TODO could have separate functions to compute these length/vel tables
        // given a Solution.
        /// This is not one of the variables in the optimization problem,
        /// but may be interesting nonetheless.
        TimeSeriesTable norm_fiber_length;
        /// This is not one of the variables in the optimization problem,
        /// but may be interesting nonetheless.
        TimeSeriesTable norm_fiber_velocity;
        /// This is not one of the variables in the optimization problem,
        /// but may be interesting nonetheless. In units of Newtons.
        TimeSeriesTable tendon_force;
        void write(const std::string& prefix) const;
    };

    GlobalStaticOptimizationSolver();

    void setModel(const Model& model) {
        _model = model;
        _model.finalizeFromProperties();
    }
    const Model& getModel() const { return _model; }

    void setKinematicsData(const TimeSeriesTable& kinematics) {
        if (kinematics.getNumRows() == 0) {
            throw std::runtime_error(
                    "The provided kinematics table has no rows.");
        }
        _kinematics = kinematics;
    }
    const TimeSeriesTable& getKinematicsData() const { return _kinematics; }

    Solution solve();

private:
    Model _model;
    // TODO make this a StatesTrajectory?
    TimeSeriesTable _kinematics;
};

} // namespace OpenSim
#endif // TOMU_GLOBALSTATICOPTIMIZATIONSOLVER_H
