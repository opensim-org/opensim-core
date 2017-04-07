#include "GlobalStaticOptimizationSolver.h"

#include "DeGroote2016Muscle.h"
#include "MotionData.h"

#include <mesh.h>

#include <OpenSim/OpenSim.h>

#include <algorithm>

using namespace OpenSim;

void GlobalStaticOptimizationSolver::Solution::write(const std::string& prefix)
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
}

/// "Separate" denotes that the dynamics are not coming from OpenSim, but
/// rather are coded separately.
template<typename T>
class GSOProblemSeparate : public mesh::OptimalControlProblemNamed<T> {
public:
    GSOProblemSeparate(const GlobalStaticOptimizationSolver& mrs,
                       const Model& model, const MotionData& motionData)
            : mesh::OptimalControlProblemNamed<T>("GSO"),
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

        _numCoordActuators = 0;
        for (const auto& actuator :
                _model.getComponentList<CoordinateActuator>()) {
            if (!actuator.get_appliesForce()) continue;

            const auto& actuPath = actuator.getAbsolutePathName();
            this->add_control(actuPath + "_control",
                              {actuator.get_min_control(),
                               actuator.get_max_control()});

            _otherControlsLabels.push_back(actuPath);
            _numCoordActuators++;
        }
        _optimalForce.resize(_numCoordActuators);
        int i_act = 0;
        for (const auto& actuator :
                _model.getComponentList<CoordinateActuator>()) {
            if (!actuator.get_appliesForce()) continue;
            _optimalForce[i_act] = actuator.getOptimalForce();
        }


        _numMuscles = 0;
        for (const auto& actuator : _model.getComponentList<Muscle>()) {
            if (!actuator.get_appliesForce()) continue;

            const auto& actuPath = actuator.getAbsolutePathName();

            // TODO use activation bounds, not excitation bounds.
            this->add_control(actuPath + "_activation",
                              {actuator.get_min_control(),
                               actuator.get_max_control()});

            this->add_path_constraint(
                    actuator.getAbsolutePathName() + "_equilibrium", 0);

            _muscleLabels.push_back(actuPath);
            _numMuscles++;
        }

        // Create De Groote muscles.
        _muscles.resize(_numMuscles);
        int i_mus = 0;
        for (const auto& osimMus : _model.getComponentList<Muscle>()) {
            if (!osimMus.get_appliesForce()) continue;

            _muscles[i_mus] = DeGroote2016Muscle<T>(
                    osimMus.get_max_isometric_force(),
                    osimMus.get_optimal_fiber_length(),
                    osimMus.get_tendon_slack_length(),
                    osimMus.get_pennation_angle_at_optimal(),
                    osimMus.get_max_contraction_velocity());

            i_mus++;
        }

        // Add a constraint for each joint moment.
        _numDOFs = 0;
        for (int i = 0; i < state.getNU(); ++i) {
            this->add_path_constraint("joint_moment_" + std::to_string(i), 0);
            _numDOFs++;
        }

    }

    void initialize_on_mesh(const Eigen::VectorXd& mesh) const override {

        // For caching desired joint moments.
        auto* mutableThis = const_cast<GSOProblemSeparate<T>*>(this);

        Eigen::VectorXd times = (_finalTime - _initialTime) * mesh;
        _motionData.interpolateNetMoments(times, mutableThis->_desiredMoments);
        _motionData.interpolateMuscleTendonLengths(times,
                mutableThis->_muscleTendonLengths);
        _motionData.interpolateMuscleTendonVelocities(times,
                mutableThis->_muscleTendonVelocities);

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

    void path_constraints(unsigned i_mesh,
                          const T& /*time*/,
                          const mesh::VectorX<T>& /*states*/,
                          const mesh::VectorX<T>& controls,
                          Eigen::Ref<mesh::VectorX<T>> constraints)
    const override {
        // Actuator equilibrium.
        // =====================

        // Assemble generalized forces to apply to the joints.
        mesh::VectorX<T> genForce(_numDOFs);
        genForce.setZero();

        // CoordinateActuators.
        // --------------------
        for (Eigen::Index i_act = 0; i_act < _numCoordActuators; ++i_act) {
            genForce[0] += _optimalForce[i_act] * controls[i_act];
        }

        // Muscles.
        // --------
        for (Eigen::Index i_act = 0; i_act < _numMuscles; ++i_act) {
            // Unpack variables.
            const T& activation = controls[_numCoordActuators + i_act];

            // Get the total muscle-tendon length and velocity from the data.
            const T& musTenLen = _muscleTendonLengths(i_act, i_mesh);
            const T& musTenVel = _muscleTendonVelocities(i_act, i_mesh);

            const T normTenForce =
                    _muscles[i_act].calcRigidTendonNormFiberForceAlongTendon(
                            activation, musTenLen, musTenVel);

            // TODO use moment arms to take care of the sign.
            genForce[0] += -
                    _muscles[i_act].get_max_isometric_force() * normTenForce;
        }


        // Achieve the motion.
        // ===================
        // /*const TODO*/ auto generatedMoments = _mrs.controlToMoment * controls;
        // TODO constraints = _desiredMoments[index] - generatedMoments;
        constraints.segment(_numMuscles, _numDOFs)
                = _desiredMoments.col(i_mesh).template cast<adouble>()
                - genForce;

        // std::cout << "DEBUG constraints " << constraints << std::endl;
    }
    void integral_cost(const T& /*time*/,
                       const mesh::VectorX<T>& /*states*/,
                       const mesh::VectorX<T>& controls,
                       T& integrand) const override {
        integrand = controls.squaredNorm();
    }
    GlobalStaticOptimizationSolver::Solution deconstruct_iterate(
            const mesh::OptimalControlIterate& ocpVars) const {

        GlobalStaticOptimizationSolver::Solution vars;
        if (_numCoordActuators) {
            vars.other_controls.setColumnLabels(_otherControlsLabels);
        }
        if (_numMuscles) {
            vars.activation.setColumnLabels(_muscleLabels);
            vars.norm_fiber_length.setColumnLabels(_muscleLabels);
            vars.norm_fiber_velocity.setColumnLabels(_muscleLabels);
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
            SimTK::RowVector activation(_numMuscles,
                                        controls.data() + _numCoordActuators,
                                        true /* makes this a view */);
            vars.activation.appendRow(time, activation);

            // Compute fiber length and velocity.
            // ----------------------------------
            // TODO this does not depend on the solution; this could be done in
            // initialize_on_mesh.
            SimTK::RowVector normFibLenRow(_numMuscles);
            SimTK::RowVector normFibVelRow(_numMuscles);
            for (Eigen::Index i_act = 0; i_act < _numMuscles; ++i_act) {
                const auto& musTenLen = _muscleTendonLengths(i_act, i_time);
                const auto& musTenVel = _muscleTendonVelocities(i_act, i_time);
                double normFiberLength;
                double normFiberVelocity;
                _muscles[i_act].calcRigidTendonFiberKinematics(
                        musTenLen, musTenVel,
                        normFiberLength, normFiberVelocity);
                normFibLenRow[i_act] = normFiberLength;
                normFibVelRow[i_act] = normFiberVelocity;
            }
            vars.norm_fiber_length.appendRow(time, normFibLenRow);
            vars.norm_fiber_velocity.appendRow(time, normFibVelRow);
        }
        return vars;
    }
private:
    const GlobalStaticOptimizationSolver& _mrs;
    Model _model;
    const MotionData& _motionData;
    double _initialTime = SimTK::NaN;
    double _finalTime = SimTK::NaN;

    // Bookkeeping.
    int _numDOFs;
    int _numCoordActuators;
    int _numMuscles;
    std::vector<std::string> _muscleLabels;
    std::vector<std::string> _otherControlsLabels;

    // Motion data to use during the optimization.
    Eigen::MatrixXd _desiredMoments;
    Eigen::MatrixXd _muscleTendonLengths;
    Eigen::MatrixXd _muscleTendonVelocities;
    // TODO std::vector<Eigen::SparseMatrixXd> moment arms.

    // CoordinateActuator optimal forces.
    Eigen::VectorXd _optimalForce;

    // De Groote muscles.
    std::vector<DeGroote2016Muscle<T>> _muscles;
};

GlobalStaticOptimizationSolver::GlobalStaticOptimizationSolver() {
    constructProperty_lowpass_cutoff_frequency_for_joint_moments(-1);
    constructProperty_create_reserve_actuators(-1);
    //constructProperty_model_file("");
    //constructProperty_kinematics_file("");
}

// TODO move this to a "InverseMuscleSolver" base class.
GlobalStaticOptimizationSolver::Solution GlobalStaticOptimizationSolver::solve() {

    // Process experimental data.
    // --------------------------
    OPENSIM_THROW_IF(get_lowpass_cutoff_frequency_for_joint_moments() <= 0 &&
            get_lowpass_cutoff_frequency_for_joint_moments() != -1, Exception,
                     "Invalid value for cutoff frequency for joint moments.");
    MotionData motionData(_model, getKinematicsData(),
                          get_lowpass_cutoff_frequency_for_joint_moments());

    // Create reserve actuators.
    // -------------------------
    Model model(_model);
    if (get_create_reserve_actuators() != -1) {
        const auto& optimalForce = get_create_reserve_actuators();
        OPENSIM_THROW_IF(optimalForce <= 0, Exception,
                "Invalid value (" + std::to_string(optimalForce)
                + ") for create_reserve_actuators; should be -1 or positive.");

        std::cout << "Adding reserve actuators with an optimal force of "
                << optimalForce << "..." << std::endl;

        SimTK::State state = model.initSystem();
        std::vector<std::string> coordNames;
        // Borrowed from CoordinateActuator::CreateForceSetOfCoordinateAct...
        for (auto& coord : model.getCoordinatesInMultibodyTreeOrder()) {
            if (!coord->isConstrained(state)) {
                auto* actu = new CoordinateActuator();
                actu->setCoordinate(const_cast<Coordinate*>(coord.get()));
                auto path = coord->getAbsolutePathName();
                coordNames.push_back(path);
                // Get rid of model name.
                path = ComponentPath(path).formRelativePath(
                        model.getAbsolutePathName()).toString();
                // Get rid of slashes in the path; slashes not allowed in names.
                std::replace(path.begin(), path.end(), '/', '_');
                actu->setName("reserve_" + path);
                actu->setOptimalForce(optimalForce);
                model.addComponent(actu);
            }
        }
        std::cout << "Added " << coordNames.size() << " reserve actuator(s), "
                "for each of the following coordinates:" << std::endl;
        for (const auto& name : coordNames) {
            std::cout << "    " << name << std::endl;
        }
    }

    // Solve the optimal control problem.
    // ----------------------------------
    auto ocp = std::make_shared<GSOProblemSeparate<adouble>>(*this,
                                                             model,
                                                             motionData);
    ocp->print_description();
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt",
                                                  100);
    mesh::OptimalControlSolution ocp_solution = dircol.solve();

    // Return the solution.
    // --------------------
    // TODO remove
    ocp_solution.write("GlobalStaticOptimizationSolver_OCP_solution.csv");
    return ocp->deconstruct_iterate(ocp_solution);
}
