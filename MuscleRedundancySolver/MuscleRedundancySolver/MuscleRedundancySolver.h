#ifndef TOMU_MUSCLEREDUNDANCYSOLVER_H
#define TOMU_MUSCLEREDUNDANCYSOLVER_H

#include <OpenSim/OpenSim.h>

namespace OpenSim {

// TODO if this solves for the activity of any actuator, then it should not
// be called "muscle" redundancy solver.
/// TODO describe filtering.

/// ### Reserve actuators
///
/// Sometimes it is not possible to achieve the desired net joint moments using
/// muscles alone. This may be caused by a number of reasons:
///   - the muscles are not strong enough to achieve the net joint moments,
///   - the net joint moments change more rapidly than activation and
///     deactivation time constants allow,
///   - the filtering of the data causes unrealistic desired net joint moments.
/// For this reason, you may want to add "reserve" actuators to your model.
/// This will be done automatically for you if you set the property
/// `create_reserve_actuators`; this option will cause a CoordinateActuator
/// to be added to the model for each unconstrained coordinate.
/// The main knob on these actuators is their `optimal_force`.
/// If the optimal force is $F$ and the actuator's control
/// signal is $e$, then the cost of using the actuator is $e*e$, but the
/// generalized force it applies is $F*e$. A smaller optimal force means a
/// greater control value is required to generate a given force.
/// TODO suggest an optimal force to use.
/// The actuators *can* generate (generalized) forces larger than their
/// optimal force. The optimal force for reserve actuators should be set very
/// low to discourage their use.
///
/// After solving, the control signal $e$ for each reserve actuator is
/// reported in the Solution's `other_controls` table.
///
/// If you need to customize the reserve actuators more than is possible via
/// `create_reserve_actuators`, you can create your own and add them to your
/// model.
// TODO replace optimal force with something easier to interpret: controls
// between 0 and 1, and weights in the objective function.
class MuscleRedundancySolver : public OpenSim::Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleRedundancySolver, OpenSim::Object);
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

    OpenSim_DECLARE_PROPERTY(initial_guess, std::string,
        "How to compute the initial guess for the optimal control problem. "
        "'static_optimization': Solve a static optimization problem to obtain "
        "a guess. "
        "'bounds': The guess is the midpoint of the variables' constraints. "
        "Default: 'static_optimization'. "
        "Only change this if static optimization is causing issues; static "
        " optimization can help solve the problem up 10 times faster.");

    //OpenSim_DECLARE_PROPERTY(model_file, std::string,
    //    "Path to a model file (.osim).");
    //OpenSim_DECLARE_PROPERTY(kinematics_file, std::string,
    //    "Path to a Storage (.sto) file containing kinematics.");

    struct Solution {
        /// The excitation trajectories for all enabled (appliesForce) muscles.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable excitation;
        /// The activation trajectories for all enabled (appliesForce) muscles.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable activation;
        /// The fiber length trajectories for all enabled (appliesForce)
        /// muscles, normalized by optimal fiber length.
        /// This will be empty if there are no enabled muscles.
        TimeSeriesTable norm_fiber_length;
        /// The fiber length trajectories for all enabled (appliesForce)
        /// muscles, normalized by max contraction velocity (e.g., within
        /// [-1, 1]). This will be empty if there are no enabled muscles.
        TimeSeriesTable norm_fiber_velocity;
        /// The control for enabled (appliesForce) CoordinateActuators, etc.
        /// This will be empty if there are no CoordinateActuators, etc.
        /// enabled.
        TimeSeriesTable other_controls;
        /// Write the solution to a series of OpenSim Storage files (one
        /// for each table) whose paths begin with `prefix`. Empty tables are
        /// not written.
        void write(const std::string& prefix) const;
    };

    MuscleRedundancySolver();

    /// Set the model whose muscles will be used to achieve the desired
    /// motion. The actuators in the model will be used to achieve the motion;
    /// you can choose which actuators are used by the method by setting
    /// their appliesForce property. Currently, only Muscles and
    /// CoordinateActuators are supported; you will get an error if other
    /// types of actuators are enabled (e.g., appliesForce).
    void setModel(const Model& model) {
        _model = model;
        _model.finalizeFromProperties();
    }
    const Model& getModel() const { return _model; }

    /// Set the kinematics (joint angles) to achieve.
    /// The table should contain generalized coordinates; generalized speeds
    /// are not used (yet, TODO). The column names for the generalized speeds
    /// should be state variable paths (e.g., `/hip/flexion/value`).
    /// You could create such a table via StatesTrajectory::exportToTable().
    // TODO rename to states_file or coordinates_file to be consistent with
    // other tools? Take Storage files?
    void setKinematicsData(const TimeSeriesTable& kinematics) {
        if (kinematics.getNumRows() == 0) {
            throw std::runtime_error(
                    "The provided kinematics table has no rows.");
        }
        _kinematics = kinematics;
    }
    const TimeSeriesTable& getKinematicsData() const { return _kinematics; }

    /// Solve for muscle activity. You must have provide a model and
    /// kinematics data before calling this function.
    /// @returns A struct containing muscle excitation, activation, fiber
    ///     length, fiber velocity, and other control signals.
    Solution solve();

private:
    Model _model;
    // TODO make this a StatesTrajectory?
    TimeSeriesTable _kinematics;
};

} // namespace OpenSim

#endif // TOMU_MUSCLEREDUNDANCYSOLVER_H
