/* -------------------------------------------------------------------------- *
 * OpenSim Moco: GlobalStaticOptimization.cpp                                 *
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
#include "GlobalStaticOptimization.h"

#include "../../Moco/MocoUtilities.h"
#include "DeGrooteFregly2016MuscleStandalone.h"
#include "InverseMuscleSolverMotionData.h"
#include <algorithm>

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>

#include <tropter/tropter.h>

using namespace OpenSim;

void GlobalStaticOptimization::Solution::write(const std::string& prefix)
        const {
    auto write = [&](const TimeSeriesTable& table, const std::string& suffix)
    {
        if (table.getNumRows()) {
            STOFileAdapter_<double>::write(table,
                                           prefix + "_" + suffix + ".sto");
        }
    };
    write(activation, "activation");
    write(other_controls, "other_controls");
    write(norm_fiber_length, "norm_fiber_length");
    write(norm_fiber_velocity, "norm_fiber_velocity");
    write(tendon_force, "tendon_force");
    write(norm_tendon_force, "norm_tendon_force");
}

/// "Separate" denotes that the dynamics are not coming from OpenSim, but
/// rather are coded separately.
template<typename T>
class GSOProblemSeparate : public tropter::Problem<T> {
public:
    GSOProblemSeparate(const GlobalStaticOptimization& mrs,
                       const Model& model,
                       const InverseMuscleSolverMotionData& motionData)
            : tropter::Problem<T>("GSO"),
              _mrs(mrs), _model(model), _motionData(motionData) {
        SimTK::State state = _model.initSystem();

        // Set the time bounds.
        _initialTime = motionData.getInitialTime();
        _finalTime = motionData.getFinalTime();
        this->set_time({_initialTime}, {_finalTime});

        // States and controls for actuators.
        // ----------------------------------

        // TODO handle different actuator types more systematically.
        auto actuators = _model.getComponentList<ScalarActuator>();
        for (const auto& actuator : actuators) {
            if (actuator.get_appliesForce() &&
                    !dynamic_cast<const Muscle*>(&actuator) &&
                    !dynamic_cast<const CoordinateActuator*>(&actuator)) {
                throw std::runtime_error("[GSO] Only Muscles and "
                        "CoordinateActuators are currently supported but the"
                        " model contains an enabled "
                        + actuator.getConcreteClassName() + ". Either set "
                        "appliesForce=false for this actuator, or remove it "
                        "from the model.");
            }
        }

        // CoordinateActuators.
        // --------------------
        _numCoordActuators = 0;
        for (const auto& actuator :
                _model.getComponentList<CoordinateActuator>()) {
            if (!actuator.get_appliesForce()) continue;

            const auto& actuPath = actuator.getAbsolutePathString();
            this->add_control(actuPath + "_control",
                              {actuator.get_min_control(),
                               actuator.get_max_control()});

            _otherControlsLabels.push_back(actuPath);
            _numCoordActuators++;
        }
        const auto& coordPathsToActuate = motionData.getCoordinatesToActuate();
        _optimalForce.resize(_numCoordActuators);
        _coordActuatorDOFs.resize(_numCoordActuators);
        const auto modelPath = model.getAbsolutePath();
        int i_act = 0;
        for (const auto& actuator :
                _model.getComponentList<CoordinateActuator>()) {
            if (!actuator.get_appliesForce()) continue;
            _optimalForce[i_act] = actuator.getOptimalForce();
            // Figure out which DOF each coordinate actuator is actuating.
            const auto* coord = actuator.getCoordinate();
            const auto coordPath = coord->getAbsolutePathString();
            size_t i_coord = 0;
            while (i_coord < coordPathsToActuate.size() &&
                    coordPathsToActuate[i_coord] != coordPath) {
                ++i_coord;
            }
            // TODO move this into InverseMuscleSolver.
            if (i_coord == coordPathsToActuate.size()) {
                throw std::runtime_error("[GSO] Could not find Coordinate '" +
                        coord->getAbsolutePathString() + "' used in "
                        "CoordinateActuator '" +
                        actuator.getAbsolutePathString()
                        + "'. Is the coordinate locked?");
            }
            _coordActuatorDOFs[i_act] = i_coord;
            ++i_act;
        }

        // Muscles.
        // --------
        _numMuscles = 0;
        const auto muscleList = _model.getComponentList<Muscle>();
        for (const auto& actuator : muscleList) {
            if (!actuator.get_appliesForce()) continue;

            const auto& actuPath = actuator.getAbsolutePathString();

            // TODO use activation bounds, not excitation bounds.
            this->add_control(actuPath + "_activation", {0, 1});
            // PR #1728 on opensim-core causes the min_control for muscles
            // to be the minimum activation, and using non-zero minimum
            // activation here causes issues (actually, only causes issues
            // with INDYGO, but for consistency, we also
            // ignore min_control for GSO).
            //                  {actuator.get_min_control(),
            //                   actuator.get_max_control()});

            _muscleLabels.push_back(actuPath);
            _numMuscles++;
        }

        // Create De Groote muscles.
        _muscles.resize(_numMuscles);
        int i_mus = 0;
        for (const auto& osimMus : muscleList) {
            if (!osimMus.get_appliesForce()) continue;

            _muscles[i_mus] = DeGrooteFregly2016MuscleStandalone<T>(
                    osimMus.get_max_isometric_force(),
                    osimMus.get_optimal_fiber_length(),
                    osimMus.get_tendon_slack_length(),
                    osimMus.get_pennation_angle_at_optimal(),
                    osimMus.get_max_contraction_velocity());

            i_mus++;
        }

        this->add_cost("effort", 1);

        // Add a constraint for each coordinate we want to actuate.
        _numCoordsToActuate = (int)coordPathsToActuate.size();
        for (const auto& coordPath : coordPathsToActuate) {
            this->add_path_constraint("net_gen_force_" + coordPath, 0);
        }
    }

    void initialize_on_mesh(const Eigen::VectorXd& mesh) const override {

        // For caching desired joint moments.
        auto* mutableThis = const_cast<GSOProblemSeparate<T>*>(this);

        // "array()" b/c Eigen matrix types do not support elementwise add.
        // Writing the equation this way (rather than tau*(t_f - t_i) + t_i)
        // avoids issues with roundoff where times.tail(1) != _finalTime.
        const auto meshArray = mesh.array();
        Eigen::VectorXd times =
                (1 - meshArray) * _initialTime + meshArray * _finalTime;

        _motionData.interpolateNetGeneralizedForces(times,
                mutableThis->_desiredMoments);
        if (_numMuscles) {
            _motionData.interpolateMuscleTendonLengths(times,
                    mutableThis->_muscleTendonLengths);
            _motionData.interpolateMuscleTendonVelocities(times,
                    mutableThis->_muscleTendonVelocities);
            _motionData.interpolateMomentArms(times,
                    mutableThis->_momentArms);
        }

        // TODO precompute matrix A and vector b such that the muscle-generated
        // moments are A*a + b.
        // TODO
        // b(i_act, i_time) = calcRigidTendonNormFiberForceAlongTendon(0,
        //      musTenLength(i_act, i_mesh), musTenVelocity(i_act, i_mesh));
        // TODO diagonal matrix:
        // A[i_time](i_act, i_act) = calcRigidTendonNormFiberForceAlongTendon(1,
        //      musTenLength(i_act, i_mesh), musTenVelocity(i_act, i_mesh))
        //      - b(i_act, i_time);
        // Multiply A and b by the moment arm matrix.
    }

    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        const auto& i_time = in.time_index;

        // Actuator equilibrium.
        // =====================
        // TODO in the future, we want this to be:
        // model.calcImplicitResidual(state); (would handle muscles AND moments)

        // Assemble generalized forces to apply to the joints.
        // TODO avoid reallocating this each time?
        tropter::VectorX<T> genForce(_numCoordsToActuate);
        genForce.setZero();

        // CoordinateActuators.
        // --------------------
        for (Eigen::Index i_act = 0; i_act < _numCoordActuators; ++i_act) {
            genForce[_coordActuatorDOFs[i_act]]
                    += _optimalForce[i_act] * in.controls[i_act];
        }

        // Muscles.
        // --------
        if (_numMuscles) {
            tropter::VectorX<T> muscleForces(_numMuscles);
            for (Eigen::Index i_act = 0; i_act < _numMuscles; ++i_act) {
                // Unpack variables.
                const T& activation = in.controls[_numCoordActuators + i_act];

                // Get the total muscle-tendon length and velocity from the
                // data.
                const T& musTenLen = _muscleTendonLengths(i_act, i_time);
                const T& musTenVel = _muscleTendonVelocities(i_act, i_time);

                muscleForces[i_act] =
                        _muscles[i_act].calcRigidTendonFiberForceAlongTendon(
                                activation, musTenLen, musTenVel);
            }

            // Compute generalized forces from muscles.
            const auto& momArms = _momentArms[i_time];
            genForce += momArms.template cast<adouble>() * muscleForces;
        }

        // Achieve the motion.
        // ===================
        if (out.path.size() != 0) {
            out.path = _desiredMoments.col(i_time).template cast<adouble>()
                     - genForce;
        }
    }
    void calc_cost(int /*cost_index*/, const tropter::CostInput<T>& in,
            T& cost) const override {
        cost = in.integral;
    }
    void calc_cost_integrand(int /*cost_index*/, const tropter::Input<T>& in,
            T& integrand) const override {
        const auto& controls = in.controls;
        integrand = controls.squaredNorm();
    }
    GlobalStaticOptimization::Solution deconstruct_iterate(
            const tropter::Iterate& ocpVars) const {

        GlobalStaticOptimization::Solution vars;
        if (_numCoordActuators) {
            vars.other_controls.setColumnLabels(_otherControlsLabels);
        }
        if (_numMuscles) {
            vars.activation.setColumnLabels(_muscleLabels);
            vars.norm_fiber_length.setColumnLabels(_muscleLabels);
            vars.norm_fiber_velocity.setColumnLabels(_muscleLabels);
            vars.tendon_force.setColumnLabels(_muscleLabels);
            vars.norm_tendon_force.setColumnLabels(_muscleLabels);
        }

        // TODO would it be faster to use vars.activation.updMatrix()?
        for (int i_time = 0; i_time < ocpVars.time.cols(); ++i_time) {
            const auto& time = ocpVars.time[i_time];
            const auto& controls = ocpVars.controls.col(i_time);

            // Other controls.
            // ---------------
            // The first _numCoordActuators rows of the controls matrix
            // are for the CoordinateActuators.
            if (_numCoordActuators) {
                SimTK::RowVector other_controls(_numCoordActuators,
                                                controls.data(),
                                                true /* <- this is a view */);
                vars.other_controls.appendRow(time, other_controls);
            }

            // Muscle-related quantities.
            // --------------------------
            if (_numMuscles == 0) continue;
            SimTK::RowVector activationRow(_numMuscles,
                                           controls.data() + _numCoordActuators,
                                           true /* makes this a view */);
            vars.activation.appendRow(time, activationRow);

            // Compute fiber length and velocity.
            // ----------------------------------
            // TODO this does not depend on the solution; this could be done in
            // initialize_on_mesh.
            SimTK::RowVector normFibLenRow(_numMuscles);
            SimTK::RowVector normFibVelRow(_numMuscles);
            SimTK::RowVector tenForceRow(_numMuscles);
            SimTK::RowVector normTenForceRow(_numMuscles);
            for (int i_act = 0; i_act < _numMuscles; ++i_act) {
                const auto& musTenLen = _muscleTendonLengths(i_act, i_time);
                const auto& musTenVel = _muscleTendonVelocities(i_act, i_time);
                double normFiberLength;
                double normFiberVelocity;
                const auto muscle = _muscles[i_act].convert_scalartype_double();
                muscle.calcRigidTendonFiberKinematics(musTenLen, musTenVel,
                        normFiberLength, normFiberVelocity);
                normFibLenRow[i_act] = normFiberLength;
                normFibVelRow[i_act] = normFiberVelocity;

                const auto& activation = activationRow.getElt(0, i_act);
                tenForceRow[i_act] =
                        muscle.calcRigidTendonFiberForceAlongTendon(
                                activation, musTenLen, musTenVel);
                normTenForceRow[i_act] =
                        tenForceRow[i_act] / muscle.get_max_isometric_force();
            }
            vars.norm_fiber_length.appendRow(time, normFibLenRow);
            vars.norm_fiber_velocity.appendRow(time, normFibVelRow);
            vars.tendon_force.appendRow(time, tenForceRow);
            vars.norm_tendon_force.appendRow(time, normTenForceRow);
        }
        return vars;
    }
private:
    const GlobalStaticOptimization& _mrs;
    Model _model;
    const InverseMuscleSolverMotionData& _motionData;
    double _initialTime = SimTK::NaN;
    double _finalTime = SimTK::NaN;

    // Bookkeeping.
    int _numCoordsToActuate;
    int _numCoordActuators;
    int _numMuscles;
    std::vector<std::string> _muscleLabels;
    std::vector<std::string> _otherControlsLabels;
    // The index of the DOF that is actuated by each CoordinateActuator.
    std::vector<Eigen::Index> _coordActuatorDOFs;

    // Motion data to use during the optimization.
    Eigen::MatrixXd _desiredMoments;
    Eigen::MatrixXd _muscleTendonLengths;
    Eigen::MatrixXd _muscleTendonVelocities;
    // TODO use Eigen::SparseMatrixXd
    std::vector<Eigen::MatrixXd> _momentArms;

    // CoordinateActuator optimal forces.
    Eigen::VectorXd _optimalForce;

    // De Groote muscles.
    std::vector<DeGrooteFregly2016MuscleStandalone<T>> _muscles;
};

GlobalStaticOptimization::GlobalStaticOptimization(
        const std::string& setupFilePath) :
        InverseMuscleSolver(setupFilePath) {
    updateFromXMLDocument();
}

GlobalStaticOptimization::Solution
GlobalStaticOptimization::solve() const {

    // Load model and kinematics files.
    // --------------------------------
    Model model;
    TimeSeriesTable kinematics;
    TimeSeriesTable netGeneralizedForces;
    loadModelAndData(model, kinematics, netGeneralizedForces);

    // Decide the coordinates for which net generalized forces will be achieved.
    // -------------------------------------------------------------------------
    SimTK::State state = model.initSystem();
    std::vector<const Coordinate*> coordsToActuate;
    const auto coordsInOrder = model.getCoordinatesInMultibodyTreeOrder();
    const auto modelPath = model.getAbsolutePath();
    if (!getProperty_coordinates_to_include().empty()) {
        // Our goal is to create a list of Coordinate* in multibody tree order.
        // We will keep track of which requested coordinates we actually find.
        std::set<std::string> coordsToInclude;
        auto numCoordsToInclude = getProperty_coordinates_to_include().size();
        for (int iInclude = 0; iInclude < numCoordsToInclude; ++iInclude) {
            coordsToInclude.insert(get_coordinates_to_include(iInclude));
        }
        // Go through coordinates in order.
        for (auto& coord : coordsInOrder) {
            // Remove the model name from the coordinate path.
            const auto coordPath = coord->getAbsolutePathString();
            // Should this coordinate be included?
            const auto foundCoordPath = coordsToInclude.find(coordPath);
            if (foundCoordPath != coordsToInclude.end()) {
                OPENSIM_THROW_IF_FRMOBJ(coord->isConstrained(state), Exception,
                        format("Coordinate '%s' is constrained and "
                        "thus cannot be listed under "
                        "'coordinates_to_include'.", coordPath));
                coordsToActuate.push_back(coord.get());
                // No longer need to search for this coordinate.
                coordsToInclude.erase(foundCoordPath);
            }
        }
        // Any remaining "coordsToInclude" are not in the model.
        if (!coordsToInclude.empty()) {
            std::string msg = "Could not find the following coordinates "
                    "listed under 'coordinates_to_include' (make sure to "
                    "use the *path* to the coordinate):\n";
            for (const auto& coordPath : coordsToInclude) {
                msg += "  " + coordPath + "\n";
            }
            OPENSIM_THROW_FRMOBJ(Exception, msg);
        }
    } else {
        // User did not specify coords to include, so include all of them.
        for (auto& coord : model.getCoordinatesInMultibodyTreeOrder()) {
            if (!coord->isConstrained(state)) {
                coordsToActuate.push_back(coord.get());
            }
        }
    }
    std::cout << "The following Coordinates will be actuated:" << std::endl;
    for (const auto* coord : coordsToActuate) {
        std::cout << "  " << coord->getAbsolutePathString() << std::endl;
    }

    // Process which actuators are included.
    // -------------------------------------
    processActuatorsToInclude(model);

    // Create reserve actuators.
    // -------------------------
    if (get_create_reserve_actuators() != -1) {
        const auto& optimalForce = get_create_reserve_actuators();
        OPENSIM_THROW_IF(optimalForce <= 0, Exception,
                format("Invalid value (%g) for create_reserve_actuators; "
                       "should be -1 or positive.", optimalForce));

        std::cout << "Adding reserve actuators with an optimal force of "
                << optimalForce << "..." << std::endl;

        std::vector<std::string> coordPaths;
        // Borrowed from CoordinateActuator::CreateForceSetOfCoordinateAct...
        for (const auto* coord : coordsToActuate) {
            auto* actu = new CoordinateActuator();
            actu->setCoordinate(const_cast<Coordinate*>(coord));
            auto path = coord->getAbsolutePathString();
            coordPaths.push_back(path);
            // Get rid of model name.
            path = coord->getAbsolutePathString();
            // Get rid of slashes in the path; slashes not allowed in names.
            std::replace(path.begin(), path.end(), '/', '_');
            actu->setName("reserve_" + path);
            actu->setOptimalForce(optimalForce);
            model.addComponent(actu);
        }
        // Re-make the system, since there are new actuators.
        model.initSystem();
        std::cout << "Added " << coordPaths.size() << " reserve actuator(s), "
                "for each of the following coordinates:" << std::endl;
        for (const auto& name : coordPaths) {
            std::cout << "  " << name << std::endl;
        }
    }

    // Determine initial and final times.
    // ----------------------------------
    // TODO create tests for initial_time and final_time.
    // TODO check for errors in initial_time and final_time.
    double initialTime;
    double finalTime;
    int numMeshPoints;
    OPENSIM_THROW_IF(get_mesh_point_frequency() <= 0, Exception,
            format("Invalid value (%g) for mesh_point_frequency; "
                   "must be positive.", get_mesh_point_frequency()));
    determineInitialAndFinalTimes(kinematics, netGeneralizedForces,
            get_mesh_point_frequency(),
            initialTime, finalTime, numMeshPoints);

    // Process experimental data.
    // --------------------------
    // TODO move to InverseMuscleSolver
    InverseMuscleSolverMotionData motionData;
    OPENSIM_THROW_IF(
            get_lowpass_cutoff_frequency_for_kinematics() <= 0 &&
            get_lowpass_cutoff_frequency_for_kinematics() != -1,
            Exception,
            "Invalid value for cutoff frequency for kinematics.");
    OPENSIM_THROW_IF(
            get_lowpass_cutoff_frequency_for_joint_moments() <= 0 &&
            get_lowpass_cutoff_frequency_for_joint_moments() != -1,
            Exception,
            "Invalid value for cutoff frequency for joint moments.");
    if (netGeneralizedForces.getNumRows()) {
        motionData = InverseMuscleSolverMotionData(model, coordsToActuate,
                initialTime, finalTime, kinematics,
                get_lowpass_cutoff_frequency_for_kinematics(),
                get_lowpass_cutoff_frequency_for_joint_moments(),
                netGeneralizedForces);
    } else {
        // We must perform inverse dynamics.
        motionData = InverseMuscleSolverMotionData(model, coordsToActuate,
                initialTime, finalTime, kinematics,
                get_lowpass_cutoff_frequency_for_kinematics(),
                get_lowpass_cutoff_frequency_for_joint_moments());
    }

    // Solve the optimal control problem.
    // ----------------------------------
    auto ocp = std::make_shared<GSOProblemSeparate<adouble>>(*this, model,
            motionData);
    ocp->print_description();
    tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
            "ipopt", numMeshPoints - 1);
    tropter::Solution ocp_solution = dircol.solve();

    // Return the solution.
    // --------------------
    // TODO remove
    ocp_solution.write("GlobalStaticOptimization_OCP_solution.csv");
    // dircol.print_constraint_values(ocp_solution);
    Solution solution = ocp->deconstruct_iterate(ocp_solution);
    if (get_write_solution() != "false") {
        IO::makeDir(get_write_solution());
        std::string prefix = getName().empty() ?
                             "GlobalStaticOptimization" : getName();
        solution.write(get_write_solution() +
                SimTK::Pathname::getPathSeparator() + prefix +
                "_solution");
    }
    return solution;
}
