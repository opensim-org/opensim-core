#ifndef OPENSIM_INVERSEMUSCLESOLVER_H
#define OPENSIM_INVERSEMUSCLESOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: InverseMuscleSolver.h                                             *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/TimeSeriesTable.h>

#include <SimTKcommon/internal/ResetOnCopy.h>

// TODO make this a forward declaration (ran into issues with unique_ptr's
// destructor).
#include "../../Moco/osimMocoDLL.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

// TODO rename DynamicRedundancySolver or ActuatorRedundancySolver

/// This is a base class for methods that solve for muscle activity for a
/// known motion (kinematics) using direct collocation.
/// Currently, the actuators that the inverse muscle solvers support are only
/// Muscles and CoordinateActuators.
/// To prevent certain Muscle%s or CoordinateActuator%s from being used in the
/// solver, set the actuator's "appliesForce" property to false. To avoid
/// tracking certain Coordinate%s, set the coordinates' "locked" property to
/// true. TODO alternatively, use properties in this class.
///
/// ### Input data
///
/// The inverse muscle solvers require two sets of data as input:
///   - kinematics (that is, generalized coordinates, or joint angles),
///   - net generalized forces (that is, "inverse dynamics" results, or net
///     joint moments).
/// You must provide kinematics, but providing net generalized forces is
/// optional (the methods can compute the net generalized forces for you by
/// running inverse dynamics internally).
/// Direct collocation works best if these data are smooth. To help with
/// this, you can request that these data are filtered (see the properties
/// named `lowpass_cutoff_frequency_...`).
///
/// TODO explain the benefit of using filtering on kinematics and joint
/// moments: must be smooth, otherwise will get noisy muscle activations, etc.
///
/// ### Mesh point frequency
///
/// For gait, you should use between 100-300 mesh points per second.
/// This setting has a greater effect for INDYGO than for Global Static
/// Optimization; see INDYGO's documentation for more information.
///
/// ### Reserve actuators
///
/// Sometimes it is not possible to achieve the desired net joint moments using
/// muscles alone. There are multiple possible causes for this:
///   - the muscles are not strong enough to achieve the net joint moments,
///   - the net joint moments change more rapidly than activation and
///     deactivation time constants allow,
///   - the filtering of the data causes unrealistic desired net joint moments.
/// For this reason, you may want to add "reserve" actuators to your model.
/// This can be done automatically for you if you set the property
/// `create_reserve_actuators` appropriately ; this option will cause a
/// CoordinateActuator to be added to the model for each unconstrained
/// coordinate. The main knob on these actuators is their `optimal_force`. If
/// the optimal force is $F$ and the actuator's control signal is $e$, then the
/// cost of using the actuator is $e*e$, but the generalized force it applies is
/// $F*e$. A smaller optimal force means a greater control value is required to
/// generate a given force.
/// The reserve actuators *can* generate (generalized) forces larger than their
/// optimal force. The optimal force for reserve actuators should be set very
/// low to discourage their use.
/// TODO suggest an optimal force to use.
///
/// After solving, the control signal $e$ for each reserve actuator is
/// reported in the Solution's `other_controls` table.
///
/// If you need to customize the reserve actuators more than is possible via
/// `create_reserve_actuators`, you can create your own and add them to your
/// model.
class OSIMMOCO_API InverseMuscleSolver : public OpenSim::Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(InverseMuscleSolver, Object);
public:

    OpenSim_DECLARE_PROPERTY(write_solution, std::string,
    "Provide the folder path (relative to working directory) to which the "
    "solution files should be written. Set to 'false' to not write the "
    "solution to disk.");

    OpenSim_DECLARE_PROPERTY(model_file, std::string,
    "Path to the OSIM file containing the OpenSim model to use. The path can "
    "be absolute or relative to the setup file.");

    OpenSim_DECLARE_PROPERTY(kinematics_file, std::string,
    "Path to a data file (CSV, STO) containing generalized coordinates "
    "to track. The path can be absolute or relative to the setup file.");

    OpenSim_DECLARE_PROPERTY(lowpass_cutoff_frequency_for_kinematics, double,
    "The frequency (Hz) at which to filter the kinematics. "
    "(default is -1, which means no filtering; for walking, consider 6 Hz).");

    // TODO add ability to specify external loads, and update comment.
    OpenSim_DECLARE_OPTIONAL_PROPERTY(net_generalized_forces_file, std::string,
    "(Optional) Path to a data file (CSV, STO) containing net generalized "
    "forces (joint moments) to achieve. Column labels should be coordinate "
    "paths (e.g., 'knee/knee_angle_r') or use the Inverse Dynamics Tool "
    "format (e.g., 'knee_angle_r_moment'). "
    "If not provided, inverse dynamics will be performed internally.");

    // TODO this should be used even if net_generalized_forces_file is provided.
    // TODO replace "joint_moments" with "net_generalized_forces."
    OpenSim_DECLARE_PROPERTY(lowpass_cutoff_frequency_for_joint_moments, double,
    "The frequency (Hz) at which to filter inverse dynamics joint moments, "
    "which are computed internally from the kinematics if "
    "net generalized forces are not provided. "
    "This is applied whether or not inverse dynamics is performed internally. "
    "(default is -1, which means no filtering; for walking, consider 6 Hz).");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(initial_time, double,
    "The start of the time interval in which to solve for muscle activity. "
    "All data must start at or before this time. "
    "(default: earliest time available in all provided data)");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(final_time, double,
    "The end of the time interval in which to solve for muscle activity. "
    "All data must end at or after this time. "
    "(default: latest time available in all provided data)");

    OpenSim_DECLARE_PROPERTY(mesh_point_frequency, int,
    "The number of mesh points per second of motion "
    "(default: 200 mesh points / second).");

    OpenSim_DECLARE_PROPERTY(create_reserve_actuators, double,
    "Create a reserve actuator (CoordinateActuator) for each unconstrained "
    "coordinate in the model, and add each to the model. Each actuator "
    "will have the specified `optimal_force`, which should be set low to "
    "discourage the use of the reserve actuators. (default is -1, which "
    "means no reserves are created)");

    OpenSim_DECLARE_LIST_PROPERTY(coordinates_to_include, std::string,
    "A list of coordinates (absolute paths) whose net generalized force should "
    "be achieved. Other coordinates still affect muscle-tendon length, but "
    "there is no attempt to achieve their net generalized force. (default: "
    "emtpy, which means all non-constrained coordinates are included.)");

    OpenSim_DECLARE_LIST_PROPERTY(actuators_to_include, std::string,
    "A list of actuators (absolute paths) that should apply force. This "
    "setting overrides the appliesForce property of the actuator. "
    "(default: empty, which means all actuators' appliesForce property are "
    "obeyed.)");

    InverseMuscleSolver();

    explicit InverseMuscleSolver(const std::string& setupFilePath);

    virtual ~InverseMuscleSolver();

    /// Set the model to use. If you set a model this way, make sure to set
    /// the model_file property to an empty string
    /// (`solver.set_model_file ("")`).
    void setModel(const Model& model);
    /// This throws an exception if setModel() has not been called. A model
    /// specified via model_file cannot be accessed via this method.
    const Model& getModel() const {
        OPENSIM_THROW_IF_FRMOBJ(!_model, Exception,
                "An attempt was made to dereference a null pointer.");
        return *_model.get();
    }
    /// Set the generalized coordinate values and speeds to track. There should
    /// be a column in the table for each generalized coordinate, and the labels
    /// for the columns should be the absolute path names for the generalized
    /// coordinate state variables (e.g., `hip/flexion/value`). If you set
    /// kinematics this way, make sure to set the kinematics_file property to an
    /// empty string (`solver.set_kinematics_file ("")`). The function
    /// StatesTrajectory::exportToTable() may be helpful in creating this table.
    // TODO allow not specifying generalized speeds (compute with splines),
    // update docs.
    // TODO rename to states_file or coordinates_file to be consistent with
    // other tools? Take Storage files?
    void setKinematicsData(const TimeSeriesTable& kinematics);
    /// This throws an exception if setKinematicsData() has not been called.
    /// Kinematic specified via kinematics_file cannot be accessed via this
    /// method.
    const TimeSeriesTable& getKinematicsData() const {
        OPENSIM_THROW_IF_FRMOBJ(!_kinematics, Exception,
                "An attempt was made to dereference a null pointer.");
        return *_kinematics.get();
    }

    // TODO document
    // TODO rename to remove "Data".
    void setNetGeneralizedForcesData(const TimeSeriesTable& netGenForces);

    const TimeSeriesTable& getNetGeneralizedForcesData() const {
        OPENSIM_THROW_IF_FRMOBJ(!_netGeneralizedForces, Exception,
                "An attempt was made to dereference a null pointer.");
        return *_netGeneralizedForces.get();
    }

protected:
    /// This function provides the model and data to be used in
    /// solving for actuator controls. This function decides whether to use
    /// programmatically-set quantities (e.g., via setModel()) or load
    /// objects from files. This function also checks for some errors.
    /// The netGeneralizedForces table is set to an empty table if this data
    /// table was not provided.
    void loadModelAndData(Model& model,
                          TimeSeriesTable& kinematics,
                          TimeSeriesTable& netGeneralizedForces) const;
    void processActuatorsToInclude(Model& model) const;
    void determineInitialAndFinalTimes(TimeSeriesTable& kinematics,
                                       TimeSeriesTable& netGeneralizedForces,
                                       const int& meshPointFrequency,
                                       double& initialTime,
                                       double& finalTime,
                                       int& numMeshPoints) const;
    SimTK::ResetOnCopy<std::unique_ptr<Model>> _model;
    // TODO make this a StatesTrajectory?
    SimTK::ResetOnCopy<std::unique_ptr<TimeSeriesTable>> _kinematics;
    SimTK::ResetOnCopy<std::unique_ptr<TimeSeriesTable>> _netGeneralizedForces;

private:
    void constructProperties();

};

} // namespace OpenSim

#endif // OPENSIM_INVERSEMUSCLESOLVER_H
