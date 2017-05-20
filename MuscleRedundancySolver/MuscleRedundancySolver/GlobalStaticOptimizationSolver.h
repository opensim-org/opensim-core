#ifndef TOMU_GLOBALSTATICOPTIMIZATIONSOLVER_H
#define TOMU_GLOBALSTATICOPTIMIZATIONSOLVER_H

#include <OpenSim/OpenSim.h>

namespace OpenSim {

// TODO document
/// To prevent certain Muscle%s or CoordinateActuator%s from being used in the
/// solver, set the actuator's "appliesForce" property to false. To avoid
/// tracking certain Coordinate%s, set the coordinates' "locked" property to
/// true. TODO alternatively, use properties in this class.
/// TODO example usage.
class GlobalStaticOptimizationSolver : public OpenSim::Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(GlobalStaticOptimizationSolver,
            OpenSim::Object);
public:

    OpenSim_DECLARE_PROPERTY(model_file, std::string,
    "Path to the OSIM file containing the OpenSim model to use. The path can "
    "be absolute or relative to the setup file.");

    OpenSim_DECLARE_PROPERTY(kinematics_file, std::string,
    "Path to a data file (CSV, STO) containing generalized coordinates "
    "to track. The path can be absolute or relative to the setup file.");

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

    // TODO rename to Iterate?
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

    /// Load a solver from an XML setup file.
    ///
    /// Note: Use print() to save solver settings to an XML file that can
    /// be subsequently loaded via this constructor.
    ///
    /// If the model or kinematics files are provided, they are not
    /// loaded until you call solve(). If these files are not provided, you
    /// must call the setModel() and/or setKinematicsData() functions before
    /// calling solve().
    explicit GlobalStaticOptimizationSolver(const std::string& setupFilePath);

    /// Set the model to use. If you set a model this way, make sure to set
    /// the model_file property to an empty string
    /// (`solver.set_model_file ("")`).
    void setModel(const Model& model) {
        _model.reset(model.clone());
        _model->finalizeFromProperties();
    }
    /// This throws an exception if setModel() has not been called. A model
    /// specified via model_file cannot be accessed via this method.
    const Model& getModel() const {
        OPENSIM_THROW_IF_FRMOBJ(!_model, Exception,
                "An attempt was made to dereference a null pointer.");
        return *_model.get();
    }
    /// Set the kinematics data to track. If you set kinematics this way, make
    /// sure to set the kinematics_file property to an empty string
    /// (`solver.set_kinematics_file ("")`).
    void setKinematicsData(const TimeSeriesTable& kinematics)
    { _kinematics.reset(new TimeSeriesTable(kinematics)); }
    /// This throws an exception if setKinematicsData() has not been called.
    /// Kinematic specified via kinematics_file cannot be accessed via this
    /// method.
    const TimeSeriesTable& getKinematicsData() const {
        OPENSIM_THROW_IF_FRMOBJ(!_kinematics, Exception,
                "An attempt was made to dereference a null pointer.");
        return *_kinematics.get();
    }

    Solution solve();

private:
    void constructProperties();

    SimTK::ResetOnCopy<std::unique_ptr<Model>> _model;
    // TODO make this a StatesTrajectory?
    SimTK::ResetOnCopy<std::unique_ptr<TimeSeriesTable>> _kinematics;
};

} // namespace OpenSim
#endif // TOMU_GLOBALSTATICOPTIMIZATIONSOLVER_H
