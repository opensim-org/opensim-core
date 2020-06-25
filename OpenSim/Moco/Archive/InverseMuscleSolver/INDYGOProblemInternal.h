#ifndef OPENSIM_INDYGOPROBLEMINTERNAL_H
#define OPENSIM_INDYGOPROBLEMINTERNAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: INDYGOProblemInternal.cpp                                         *
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

#include "../../Moco/MocoUtilities.h"
#include "DeGrooteFregly2016MuscleStandalone.h"
#include "GlobalStaticOptimization.h"
#include "INDYGO.h"
#include "InverseMuscleSolverMotionData.h"
#include <algorithm>

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Model/Model.h>

#include <tropter/tropter.h>

namespace OpenSim {


/// "Internal" denotes that the dynamics are not coming from OpenSim, but
/// rather are coded separately.
template<typename T>
class INDYGOProblemInternal : public tropter::Problem<T> {
public:
    INDYGOProblemInternal(const INDYGO& mrs,
            const Model& model,
            const InverseMuscleSolverMotionData& motionData,
            const std::string& fiberDynamicsMode)
            : tropter::Problem<T>("INDYGO"),
              _mrs(mrs), _model(model), _motionData(motionData),
              _useFiberLengthState(fiberDynamicsMode == "fiber_length") {
        SimTK::State state = _model.initSystem();

        // Set the time bounds.
        _initialTime = motionData.getInitialTime();
        _finalTime = motionData.getFinalTime();
        this->set_time({_initialTime}, {_finalTime});

        // States and controls for actuators.
        // ----------------------------------
        // TODO handle different actuator types more systematically.

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
                throw std::runtime_error("[INDYGO] Could not find Coordinate '" +
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
        for (const auto& actuator : _model.getComponentList<Muscle>()) {
            // TODO consolidate with MotionData.
            if (!actuator.get_appliesForce()) continue;

            const auto& actuPath = actuator.getAbsolutePathString();
            // Activation dynamics.
            // PR #1728 on opensim-core causes the min_control for muscles
            // to be the minimum activation, and using non-zero minimum
            // excitation/activation here causes issues (very noisy
            // excitation/activation). Moreover, allowing zero muscle
            // activation is possible with implicit muscle models and has
            // numerical benefits.
            this->add_control(actuPath + "_excitation", {0, 1});
            // {actuator.get_min_control(),
            //  actuator.get_max_control()});
            // TODO use activation bounds, not excitation bounds (then
            // also update the property comment for zero_initial_activation).
            const double initialActiv = mrs.get_zero_initial_activation() ?
                                        0.0 : SimTK::NaN;
            // const double initialActiv = mrs.get_zero_initial_activation() ?
            //         std::max(actuator.get_min_control(), 0.0) : SimTK::NaN;
            this->add_state(actuPath + "_activation", {0, 1},
                    initialActiv); // TODO fix noisy activation bug
            // {actuator.get_min_control(),
            //  actuator.get_max_control()},

            // Fiber dynamics.
            if (_useFiberLengthState) {
                // TODO initial value should be 0? That is what CMC does (via
                // equilibrateMuscles()).
                this->add_control(actuPath + "_norm_fiber_velocity", {-1, 1});
                // These bounds come from simtk.org/projects/optcntrlmuscle.
                this->add_state(actuPath + "_norm_fiber_length", {0.2, 1.8});
            } else {
                // TODO play with these bounds.
                this->add_control(actuPath + "_tendon_force_rate_control",
                        {-50, 50});
                this->add_state(actuPath + "_norm_tendon_force", {0, 5});
            }
            this->add_path_constraint(actuPath + "_equilibrium", 0);

            _muscleLabels.push_back(actuPath);
            _numMuscles++;
        }

        // Create De Groote muscles.
        _muscles.resize(_numMuscles);
        int i_mus = 0;
        for (const auto& osimMus : _model.getComponentList<Muscle>()) {
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
        auto* mutableThis = const_cast<INDYGOProblemInternal<T>*>(this);

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
            _motionData.interpolateMomentArms(times,
                    mutableThis->_momentArms);
            if (!_useFiberLengthState) {
                _motionData.interpolateMuscleTendonVelocities(times,
                        mutableThis->_muscleTendonVelocities);
            }
        }
    }

    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {

        const auto& i_time = in.time_index;
        const auto& states = in.states;
        const auto& controls = in.controls;

        // Actuator dynamics.
        // ==================
        for (Eigen::Index i_act = 0; i_act < _numMuscles; ++i_act) {

            // Unpack variables.
            const T& excitation = controls[_numCoordActuators + 2 * i_act];
            const T& activation = states[2 * i_act];

            // Activation dynamics.
            _muscles[i_act].calcActivationDynamics(excitation, activation,
                    out.dynamics[2 * i_act]);

            // Fiber dynamics.
            if (_useFiberLengthState) {
                const T& normFibVel =
                        controls[_numCoordActuators + 2 * i_act + 1];
                out.dynamics[2 * i_act + 1] =
                        _muscles[i_act].get_max_contraction_velocity()
                                * normFibVel;
            } else {
                const T& tenForceRateControl =
                        controls[_numCoordActuators + 2 * i_act + 1];
                out.dynamics[2 * i_act + 1] = _tendonForceDynamicsScalingFactor
                        * tenForceRateControl;
            }
        }

        // TODO std::cout << "DEBUG dynamics " << derivatives << std::endl;

        // Actuator equilibrium.
        // =====================

        // Assemble generalized forces to apply to the joints.
        tropter::VectorX<T> genForce(_numCoordsToActuate);
        genForce.setZero();

        // CoordinateActuators.
        // --------------------
        for (Eigen::Index i_act = 0; i_act < _numCoordActuators; ++i_act) {
            genForce[_coordActuatorDOFs[i_act]]
                    += _optimalForce[i_act] * controls[i_act];
        }

        // Muscles.
        // --------
        if (_numMuscles) {
            tropter::VectorX<T> tendonForces(_numMuscles);
            for (Eigen::Index i_act = 0; i_act < _numMuscles; ++i_act) {
                // Unpack variables.
                const T& activation = states[2 * i_act];
                // Get the total muscle-tendon length from the data.
                const T& musTenLen = _muscleTendonLengths(i_act, i_time);

                if (_useFiberLengthState) {
                    const T& normFibVel = controls[_numCoordActuators+2*i_act+1];
                    const T& normFibLen = states[2 * i_act + 1];
                    T normTenForce;
                    _muscles[i_act].calcEquilibriumResidual(activation,
                            musTenLen, normFibLen, normFibVel,
                            out.path[i_act],
                            normTenForce);
                    tendonForces[i_act] =
                            _muscles[i_act].get_max_isometric_force()
                                    * normTenForce;
                } else {
                    const T& tenForceRateControl =
                            controls[_numCoordActuators + 2 * i_act + 1];
                    const T& normTenForce = states[2 * i_act + 1];
                    const T& musTenVel = _muscleTendonVelocities(i_act, i_time);

                    const T normTenForceRate =
                            _tendonForceDynamicsScalingFactor
                                    * tenForceRateControl;
                    _muscles[i_act].calcTendonForceStateEquilibriumResidual(
                            activation, musTenLen, musTenVel, normTenForce,
                            normTenForceRate,
                            out.path[i_act], tendonForces[i_act]);
                }
            }

            // Compute generalized forces from muscles.
            const auto& momArms = _momentArms[i_time];
            // TODO convert to type T once instead of in every iteration.
            genForce += momArms.template cast<T>() * tendonForces;
        }


        // Achieve the motion.
        // ===================
        if (out.path.size() != 0) {
            out.path.segment(_numMuscles, _numCoordsToActuate)
                    = _desiredMoments.col(i_time).template cast<T>()
                    - genForce;
        }
    }
    void calc_cost(int /*cost_index*/, const tropter::CostInput<T>& in,
            T& cost) const override {
        cost = in.integral;
    }
    void calc_cost_integrand(int /*cost_index*/, const tropter::Input<T>& in,
            T& integrand) const override {
        // Unpack variables.
        const auto& states = in.states;
        const auto& controls = in.controls;
        // Use a map to skip over fiber velocities.
        using ExcitationsVector = Eigen::Map<const tropter::VectorX<T>,
                /* pointer alignment: Unaligned */   0,
                /* pointer increment btn elements */ Eigen::InnerStride<2>>;
        ExcitationsVector muscleExcit(controls.data() + _numCoordActuators,
                _numMuscles);

        // Minimize activations to prevent the initial
        // activation from being incorrectly large (b/c no penalty)
        // TODO could also just minimize initial activation.
        // Use a map to skip over fiber lengths.
        using ActivationsVector = Eigen::Map<const tropter::VectorX<T>,
                /* pointer alignment: Unaligned */   0,
                /* pointer increment btn elements */ Eigen::InnerStride<2>>;
        ActivationsVector muscleActiv(states.data(), _numMuscles);

        integrand = controls.head(_numCoordActuators).squaredNorm()
                + muscleExcit.squaredNorm() +
                + muscleActiv.squaredNorm();
    }
    /// If mrsVars seems to be empty (no rows in either the
    /// mrsVars.activation or mrsVars.other_controls tables), then this returns
    /// an empty iterate.
    // TODO should take a "Iterate" instead.
    tropter::Iterate construct_iterate(
            const INDYGO::Solution& mrsVars) const {
        using Eigen::Map;
        using Eigen::VectorXd;
        using Eigen::MatrixXd;

        tropter::Iterate vars;
        // The mrsVars has time as the row dimension, but mrsVars has time as
        // the column dimension. As a result, some quantities must be
        // transposed.
        size_t numTimes;
        if (mrsVars.activation.getNumRows() != 0) {
            numTimes = mrsVars.activation.getNumRows();
            vars.time = Map<const VectorXd>(
                    mrsVars.activation.getIndependentColumn().data(), numTimes);
        } else if (mrsVars.other_controls.getNumRows() != 0) {
            numTimes = mrsVars.other_controls.getNumRows();
            vars.time = Map<const VectorXd>(
                    mrsVars.other_controls.getIndependentColumn().data(),
                    numTimes);
        } else {
            // mrsVars seems to be empty (no muscles or other controls);
            // return an empty Iterate.
            return vars;
        }

        // Allocate memory.
        // Each muscle has excitation and norm. fiber velocity (or tendon force
        // rate) controls.
        const int numControls = _numCoordActuators + 2 * _numMuscles;
        // Each muscle has activation and fiber length (or tendon force) states.
        const int numStates = 2 * _numMuscles;
        vars.controls.resize(numControls, numTimes);
        vars.states.resize(numStates, numTimes);
        vars.control_names.resize(numControls);
        vars.state_names.resize(2 * _numMuscles);

        if (_numCoordActuators) {
            // Set control names.
            const auto& otherControlsLabels =
                    mrsVars.other_controls.getColumnLabels();
            assert(_numCoordActuators == (int)otherControlsLabels.size());
            for (int i_act = 0; i_act < _numCoordActuators; ++i_act) {
                vars.control_names[i_act] =
                        otherControlsLabels[i_act] + "_control";
            }
            vars.controls.topRows(_numCoordActuators) = Map<const MatrixXd>(
                    &mrsVars.other_controls.getMatrix()(0, 0),
                    numTimes, _numCoordActuators).transpose();
        }

        if (_numMuscles) {
            // Set state and control names.
            const auto& muscleNames = mrsVars.activation.getColumnLabels();
            for (int i_musc = 0; i_musc < _numMuscles; ++i_musc) {
                const auto& muscleName = muscleNames[i_musc];
                vars.control_names[_numCoordActuators + 2 * i_musc] =
                        muscleName + "_excitation";
                vars.control_names[_numCoordActuators + 2 * i_musc + 1] =
                        muscleName +
                                (_useFiberLengthState ? "_norm_fiber_velocity"
                                                      : "_tendon_force_rate_control");
                vars.state_names[2 * i_musc] = muscleName + "_activation";
                vars.state_names[2 * i_musc + 1] = muscleName +
                        (_useFiberLengthState ? "_norm_fiber_length"
                                              : "_norm_tendon_force");
            }
            // SkipRowView is a view onto a Matrix wherein every other row of
            // the original matrix is skipped. We use this to edit every
            // other row using a single assignment. Eigen's inner stride (the
            // second template argument below) is the pointer increment between
            // two consecutive entries; so a stride of 2 skips 1 row.
            // The first argument is the outer stride (increment between
            // columns), which we only know during run-time (e.g., Dynamic).
            using SkipRowStride = Eigen::Stride<Eigen::Dynamic, 2>;
            // These tell Map to interpret the passed-in data as a matrix;
            // the outer stride is just the number of rows (in this case).
            SkipRowStride controlsStride(vars.controls.outerStride(), 2);
            SkipRowStride statesStride(vars.states.outerStride(), 2);
            using SkipRowView = Map<MatrixXd, Eigen::Unaligned, SkipRowStride>;

            SkipRowView excitationView(
                    // Skip over coordinate actuator controls.
                    vars.controls.bottomRows(2 *_numMuscles).data(),
                    _numMuscles, numTimes, controlsStride);
            excitationView = Map<const MatrixXd>(
                    &mrsVars.excitation.getMatrix()(0, 0),
                    numTimes, _numMuscles).transpose();

            SkipRowView activationView(
                    vars.states.data(), _numMuscles, numTimes, statesStride);
            activationView = Map<const MatrixXd>(
                    &mrsVars.activation.getMatrix()(0, 0),
                    numTimes, _numMuscles).transpose();

            if (_useFiberLengthState) {
                SkipRowView fiberLengthView(
                        // Skip the first state row, which has activations
                        // for the first muscle.
                        vars.states.bottomRows(2 * _numMuscles - 1).data(),
                        _numMuscles, numTimes, statesStride);
                fiberLengthView = Map<const MatrixXd>(
                        &mrsVars.norm_fiber_length.getMatrix()(0, 0),
                        numTimes, _numMuscles).transpose();

                SkipRowView fiberVelocityView(
                        // Skip the coordinate actuator rows and the excitation
                        // for the first muscle.
                        vars.controls.bottomRows(2 * _numMuscles - 1).data(),
                        _numMuscles, numTimes, controlsStride);
                fiberVelocityView = Map<const MatrixXd>(
                        &mrsVars.norm_fiber_velocity.getMatrix()(0, 0),
                        numTimes, _numMuscles).transpose();
            } else {
                SkipRowView tendonForceView(
                        vars.states.bottomRows(2 * _numMuscles - 1).data(),
                        _numMuscles, numTimes, statesStride);
                tendonForceView = Map<const MatrixXd>(
                        &mrsVars.norm_tendon_force.getMatrix()(0, 0),
                        numTimes, _numMuscles).transpose();

                SkipRowView tendonForceRateControlView(
                        vars.controls.bottomRows(2 * _numMuscles - 1).data(),
                        _numMuscles, numTimes, controlsStride);
                tendonForceRateControlView = Map<const MatrixXd>(
                        &mrsVars.tendon_force_rate_control.getMatrix()(0, 0),
                        numTimes, _numMuscles).transpose();
            }
        }
        return vars;
    }
    /// The mesh points in the iterate must match those that this solver is
    /// expecting (for the current mesh refinement iteration); this is
    /// because computing tendon length requires on the cached muscle-tendon
    /// lengths.
    // TODO rename to make_indygo_iterate and make_optimal_control_iterate
    INDYGO::Solution deconstruct_iterate(
            const tropter::Iterate& ocpVars) const {

        INDYGO::Solution sol;
        if (_numCoordActuators) {
            sol.other_controls.setColumnLabels(_otherControlsLabels);
        }
        if (_numMuscles) {
            sol.excitation.setColumnLabels(_muscleLabels);
            sol.activation.setColumnLabels(_muscleLabels);
            sol.norm_fiber_length.setColumnLabels(_muscleLabels);
            sol.norm_fiber_velocity.setColumnLabels(_muscleLabels);
            sol.tendon_force.setColumnLabels(_muscleLabels);
            sol.norm_tendon_force.setColumnLabels(_muscleLabels);
            if (!_useFiberLengthState) {
                sol.tendon_force_rate_control.setColumnLabels(_muscleLabels);
            }
        }

        for (int i_time = 0; i_time < ocpVars.time.cols(); ++i_time) {
            const auto& time = ocpVars.time[i_time];
            const auto& controls = ocpVars.controls.col(i_time);
            const auto& states = ocpVars.states.col(i_time);

            // Other controls.
            // ---------------
            // The first _numCoordActuators rows of the controls matrix
            // are for the CoordinateActuators.
            if (_numCoordActuators) {
                SimTK::RowVector other_controls(_numCoordActuators,
                        controls.data(),
                        true /* <- this is a view */);
                sol.other_controls.appendRow(time, other_controls);
            }

            // Muscle-related quantities.
            // --------------------------
            if (_numMuscles == 0) continue;
            SimTK::RowVector excitation(_numMuscles,
                    2 /* stride: skip over fiber vel. */,
                    controls.data() + _numCoordActuators,
                    true /* makes this a view */);
            sol.excitation.appendRow(time, excitation);

            SimTK::RowVector activation(_numMuscles,
                    2 /* stride: skip over fiber length */,
                    states.data(),
                    true /* makes this a view */);
            sol.activation.appendRow(time, activation);

            if (_useFiberLengthState) {
                SimTK::RowVector fiber_length(_numMuscles, /*TODO*/
                        2 /* stride: skip over activation */,
                        states.data() + 1,
                        true /* makes this a view */);
                sol.norm_fiber_length.appendRow(time, fiber_length);

                SimTK::RowVector fiber_velocity(_numMuscles,
                        2 /* stride: skip over excit. */,
                        controls.data() + _numCoordActuators + 1,
                        true /* makes this a view */);
                sol.norm_fiber_velocity.appendRow(time, fiber_velocity);

                // Compute other helpful quantities from the states.
                SimTK::RowVector tendon_force(_numMuscles);
                SimTK::RowVector norm_tendon_force(_numMuscles);
                for (int i_musc = 0; i_musc < _numMuscles; ++i_musc) {
                    const auto muscle =
                            _muscles[i_musc].convert_scalartype_double();
                    const auto& musTenLen = _muscleTendonLengths(i_musc, i_time);
                    const auto& normFibLen = states[2 * i_musc + 1];

                    norm_tendon_force[i_musc] =
                            muscle.calcNormTendonForce(musTenLen, normFibLen);
                    tendon_force[i_musc] = muscle.get_max_isometric_force() *
                            norm_tendon_force[i_musc];
                }
                sol.tendon_force.appendRow(time, tendon_force);
                sol.norm_tendon_force.appendRow(time, norm_tendon_force);

                // Don't worry about tendon_force_rate_control. It's unlikely
                // that users are interested in this. It will be necessary to
                // carry over if one wants to first solve a problem with
                // fiber length state and use the solution as an initial
                // guess to a problem with tendon force state.
            } else {
                // TODO
                SimTK::RowVector norm_tendon_force(_numMuscles,
                        2, states.data() + 1, true);
                sol.norm_tendon_force.appendRow(time, norm_tendon_force);

                SimTK::RowVector tendon_force_rate_control(_numMuscles,
                        2, controls.data() + _numCoordActuators + 1,
                        true);
                sol.tendon_force_rate_control.appendRow(time,
                        tendon_force_rate_control);

                // We must compute fiber length, fiber velocity, and tendon
                // force from the states and controls.
                SimTK::RowVector norm_fiber_length(_numMuscles);
                SimTK::RowVector norm_fiber_velocity(_numMuscles);
                SimTK::RowVector tendon_force(_numMuscles);
                for (int i_musc = 0; i_musc < _numMuscles; ++i_musc) {
                    const auto muscle =
                            _muscles[i_musc].convert_scalartype_double();
                    const auto& musTenLen =
                            _muscleTendonLengths(i_musc, i_time);
                    const auto& musTenVel =
                            _muscleTendonVelocities(i_musc, i_time);
                    const auto& normTenForce = states[2 * i_musc + 1];
                    const auto& tendonForceRateControl =
                            controls[_numCoordActuators + 2* i_musc + 1];

                    const auto normTenForceRate =
                            _tendonForceDynamicsScalingFactor *
                                    tendonForceRateControl;
                    double cosPenn; // unused.
                    muscle.calcIntermediatesForTendonForceState(
                            musTenLen, musTenVel,
                            normTenForce, normTenForceRate,
                            norm_fiber_length[i_musc],
                            norm_fiber_velocity[i_musc],
                            cosPenn,
                            tendon_force[i_musc]);
                }
                sol.norm_fiber_length.appendRow(time, norm_fiber_length);
                sol.norm_fiber_velocity.appendRow(time, norm_fiber_velocity);
                sol.tendon_force.appendRow(time, tendon_force);
            }
        }
        return sol;
    }
private:
    const INDYGO& _mrs;
    Model _model;
    const InverseMuscleSolverMotionData& _motionData;
    const bool _useFiberLengthState;
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
    // TODO use Eigen::SparseMatrixXd
    std::vector<Eigen::MatrixXd> _momentArms;
    Eigen::MatrixXd _muscleTendonVelocities;
    const double _tendonForceDynamicsScalingFactor = 10.0;

    // CoordinateActuator optimal forces.
    Eigen::VectorXd _optimalForce;

    // De Groote muscles.
    std::vector<DeGrooteFregly2016MuscleStandalone<T>> _muscles;
};

} // namespace OpenSim

#endif // OPENSIM_INDYGOPROBLEMINTERNAL_H
