

#include "MuscleRedundancySolver.h"

#include <mesh.h>
#include <OpenSim/OpenSim.h>
// TODO should not be needed:
#include <OpenSim/Simulation/InverseDynamicsSolver.h>

using namespace OpenSim;

void MuscleRedundancySolver::Solution::write(const std::string& prefix) const
{
    auto write = [&](const TimeSeriesTable& table, const std::string& suffix)
    {
        if (table.getNumRows()) {
            STOFileAdapter_<double>::write(table,
                                           prefix + "_" + suffix + ".sto");
        }
    };
    write(excitation, "excitation");
    write(activation, "activation");
    write(norm_fiber_length, "norm_fiber_length");
    write(norm_fiber_velocity, "norm_fiber_velocity");
    write(other_controls, "other_controls");
}

/// Given a table, create a spline for each colum in `labels`, and provide all
/// of these splines in a set.
/// This function exists because GCVSplineSet's constructor takes a Storage,
/// not a TimeSeriesTable.
GCVSplineSet createGCVSplineSet(const TimeSeriesTable& table,
                                const std::vector<std::string>& labels,
                                int degree = 5,
                                double errorVariance = 0.0) {
    GCVSplineSet set;
    const auto& time = table.getIndependentColumn();
    for (const auto& label : labels) {
        const auto& column = table.getDependentColumn(label);
        set.adoptAndAppend(new GCVSpline(degree, column.size(), time.data(),
                                         &column[0], label, errorVariance));
    }
    return set;
}


/// "Separate" denotes that the dynamics are not coming from OpenSim, but
/// rather are coded separately.
template<typename T>
class MRSProblemSeparate : public mesh::OptimalControlProblemNamed<T> {
public:
    MRSProblemSeparate(const MuscleRedundancySolver& mrs,
                       const GCVSplineSet& inverseDynamics)
            : mesh::OptimalControlProblemNamed<T>("MRS"),
              _mrs(mrs), _model(mrs.getModel()),
              _inverseDynamics(inverseDynamics) {
        SimTK::State state = _model.initSystem();

        // Set the time bounds.
        // TODO when time is a variable, this has to be more advanced:
        const auto& times = _mrs.getKinematicsData().getIndependentColumn();
        _initialTime = times.front();
        _finalTime = times.back();
        this->set_time({_initialTime}, {_finalTime});

        // States and controls for actuators.
        // ----------------------------------

        // TODO handle different actuator types more systematically.
        auto actuators = _model.getComponentList<ScalarActuator>();
        for (const auto& actuator : actuators) {
            if (actuator.get_appliesForce() &&
                    !dynamic_cast<const Muscle*>(&actuator) &&
                    !dynamic_cast<const CoordinateActuator*>(&actuator)) {
                throw std::runtime_error("[MRS] Only Muscles and "
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
            const auto& actuPath = actuator.getAbsolutePathName();
            this->add_control(actuPath + "_control",
                              {actuator.get_min_control(),
                               actuator.get_max_control()});
            _controlToMoment = actuator.getOptimalForce();

            _otherControlsLabels.push_back(actuPath);
            _numCoordActuators++;
        }

        _numMuscles = 0;
        for (const auto& actuator : _model.getComponentList<Muscle>()) {
            if (!actuator.get_appliesForce()) continue;

            const auto& actuPath = actuator.getAbsolutePathName();
            // Activation dynamics.
            this->add_control(actuPath + "_excitation",
                              {actuator.get_min_control(),
                               actuator.get_max_control()});
            // TODO use activation bounds, not excitation bounds.
            this->add_state(actuPath + "_activation",
                            {actuator.get_min_control(),
                             actuator.get_max_control()});

            // Fiber dynamics.
            this->add_control(actuPath + "_norm_fiber_velocity", {-1, 1});
            // These bounds come from simtk.org/projects/optcntrlmuscle.
            // TODO our initial guess for fiber length does not obey
            // these bounds.
            this->add_state(actuPath + "_norm_fiber_length", {0.2, 1.8});
            this->add_path_constraint(
                    actuator.getAbsolutePathName() + "_equilibrium", 0);

            _muscleLabels.push_back(actuPath);
            _numMuscles++;
        }

        // Store muscle parameters.
        _max_isometric_force.resize(_numMuscles);
        _optimal_fiber_length.resize(_numMuscles);
        _tendon_slack_length.resize(_numMuscles);
        _max_contraction_velocity.resize(_numMuscles);
        _pennation_angle_at_optimal.resize(_numMuscles);
        int i_mus = 0;
        for (const auto& muscle : _model.getComponentList<Muscle>()) {
            if (!muscle.get_appliesForce()) continue;

            _max_isometric_force[i_mus] = muscle.get_max_isometric_force();
            _optimal_fiber_length[i_mus] = muscle.get_optimal_fiber_length();
            _tendon_slack_length[i_mus] = muscle.get_tendon_slack_length();
            _max_contraction_velocity[i_mus] =
                    muscle.get_max_contraction_velocity();
            _pennation_angle_at_optimal[i_mus] =
                    muscle.get_pennation_angle_at_optimal();

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
        auto* mutableThis = const_cast<MRSProblemSeparate<T>*>(this);

        // Evaluate inverse dynamics result at all mesh points.
        // ----------------------------------------------------
        // Store the desired joint moments in an Eigen matrix.
        Eigen::VectorXd times = (_finalTime - _initialTime) * mesh;
        // TODO probably has to be VectorX<T> to use with subtraction.
        mutableThis->_desiredMoments.resize(_inverseDynamics.getSize(),
                                            times.size());
        for (size_t i_time = 0; i_time < size_t(times.size()); ++i_time) {
            for (size_t i_dof = 0; i_dof < size_t(_inverseDynamics.getSize());
                 ++i_dof)
            {
                const double value = _inverseDynamics[i_dof].calcValue(
                        SimTK::Vector(1, times[i_time]));
                mutableThis->_desiredMoments(i_dof, i_time) = value;
            }
        }
        mesh::write(times, _desiredMoments, "DEBUG_desiredMoments.csv");


        // Muscle Analysis: muscle-tendon length.
        // --------------------------------------
        // TODO tailored to hanging mass: just use absoluate value of the
        // coordinate value.
        mutableThis->_muscleTendonLength.resize(1, times.size());
        const auto& kinematics = _mrs.getKinematicsData();
        const auto& muscleTendonLengthColumn =
                kinematics.getDependentColumn("joint/height/value");
        GCVSpline muscleTendonLengthData(5, kinematics.getNumRows(),
                &kinematics.getIndependentColumn()[0],
                &muscleTendonLengthColumn[0],
                "muscleTendonLength", 0);
        for (size_t i_mesh = 0; i_mesh < size_t(times.size()); ++i_mesh) {
            mutableThis->_muscleTendonLength(i_mesh) =
                    std::abs(muscleTendonLengthData.calcValue(
                            SimTK::Vector(1, times[i_mesh])));
        }

    }
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // TODO first solve a static optimization problem.


        // Actuator dynamics.
        // ==================
        // Parameters.
        // -----------
        static const double actTimeConst   = 0.015;
        static const double deactTimeConst = 0.060;
        static const double tanhSteepness  = 0.1;
        for (Eigen::Index i_act = 0; i_act < _numMuscles; ++i_act) {

            // Unpack variables.
            const T& excitation = controls[_numCoordActuators + 2 * i_act];
            const T& activation = states[2 * i_act];
            const T& normFibVel = controls[_numCoordActuators + 2 * i_act + 1];

            // Activation dynamics.
            //     f = 0.5 tanh(b(e - a))
            //     z = 0.5 + 1.5a
            // da/dt = [(f + 0.5)/(tau_a * z) + (-f + 0.5)*z/tau_d] * (e - a)
            const T timeConstFactor = 0.5 + 1.5 * activation;
            const T tempAct = 1.0 / (actTimeConst * timeConstFactor);
            const T tempDeact = timeConstFactor / deactTimeConst;
            const T f = 0.5 * tanh(tanhSteepness * (excitation - activation));
            const T timeConst = tempAct * (f + 0.5) + tempDeact * (-f + 0.5);
            derivatives[2 * i_act] = timeConst * (excitation - activation);

            // Fiber dynamics.
            derivatives[2 * i_act + 1] =
                    _max_contraction_velocity[i_act] * normFibVel;
        }

        // TODO std::cout << "DEBUG dynamics " << derivatives << std::endl;
    }
    void path_constraints(unsigned i_mesh,
                          const T& /*time*/,
                          const mesh::VectorX<T>& states,
                          const mesh::VectorX<T>& controls,
                          Eigen::Ref<mesh::VectorX<T>> constraints)
            const override {
        // Actuator equilibrium.
        // =====================
        // Parameters.
        // -----------
        // Tendon force-length curve.
        static const double kT = 35;
        static const double c1 = 0.200;
        static const double c2 = 0.995;
        static const double c3 = 0.250;

        // Active force-length curve.
        static const double b11 =  0.815;
        static const double b21 =  1.055;
        static const double b31 =  0.162;
        static const double b41 =  0.063;
        static const double b12 =  0.433;
        static const double b22 =  0.717;
        static const double b32 = -0.030;
        static const double b42 =  0.200;
        static const double b13 =  0.100;
        static const double b23 =  1.000;
        static const double b33 =  0.354;
        static const double b43 =  0.000;

        // Passive force-length curve.
        static const double kPE = 4.0;
        static const double e0  = 0.6;

        // Muscle force-velocity.
        static const double d1 = -0.318;
        static const double d2 = -8.149;
        static const double d3 = -0.374;
        static const double d4 =  0.886;

        auto gaussian = [](const T& x, const double& b1, const double& b2,
                           const double& b3, const double& b4) -> T {
            return b1 * exp((-0.5 * pow(x - b2, 2)) / (b3 + b4 * x));
        };

        // Assemble generalized forces to apply to the joints.
        mesh::VectorX<T> genForce(_numDOFs);
        genForce.setZero();

        // CoordinateActuators.
        // --------------------
        for (Eigen::Index i_act = 0; i_act < _numCoordActuators; ++i_act) {
            genForce[0] += _controlToMoment * controls[i_act];
        }

        // Muscles.
        // --------
        for (Eigen::Index i_act = 0; i_act < _numMuscles; ++i_act) {

            // Unpack variables.
            // -----------------
            const T& activation = states[2 * i_act];
            const T& normFibVel = controls[_numCoordActuators + 2 * i_act + 1];
            const T& normFibLen = states[2 * i_act + 1];

            // Intermediate quantities.
            // ------------------------
            const T fibLen = normFibLen * _optimal_fiber_length[i_act];
            // TODO cache this somewhere; this is constant.
            const double fibWidth = _optimal_fiber_length[i_act]
                             * sin(_pennation_angle_at_optimal[i_act]);
            // Tendon length.
            const T& musTenLen = _muscleTendonLength(i_act, i_mesh);
            // lT = lMT - sqrt(lM^2 - w^2)
            const T tenLen = musTenLen
                           - sqrt(fibLen*fibLen - fibWidth*fibWidth);
            const T normTenLen = tenLen / _tendon_slack_length[i_act];
            const T cosPenn = (musTenLen - tenLen) / fibLen;

            // Curves/multipliers.
            // -------------------
            // Tendon force-length curve.
            const T normTenForce = c1 * exp(kT * (normTenLen - c2)) - c3;

            // Active force-length curve.
            // Sum of 3 gaussians.
            const T activeForceLenMult =
                    gaussian(normFibLen, b11, b21, b31, b41) +
                    gaussian(normFibLen, b12, b22, b32, b42) +
                    gaussian(normFibLen, b13, b23, b33, b43);

            // Passive force-length curve.
            const T passiveFibForce = (exp(kPE * (normFibLen - 1)/ e0) - 1) /
                    (exp(kPE) - 1);

            // Force-velocity curve.
            const T tempV = d2 * normFibVel + d3;
            const T tempLogArg = tempV + sqrt(pow(tempV, 2) + 1);
            const T forceVelMult = d1 * log(tempLogArg) + d4;

            // Equilibrium constraint.
            // -----------------------
            const T normFibForce =
                    activation * activeForceLenMult * forceVelMult
                        + passiveFibForce;
            const T normFibForceAlongTen = normFibForce * cosPenn;

            constraints[i_act] = normFibForceAlongTen - normTenForce;

            // Used to evaluate the joint moment error.
            genForce[0] += -_max_isometric_force[i_act] * normTenForce;
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
        // Use a map to skip over fiber velocities.
        using ExcitationsVector = Eigen::Map<const mesh::VectorX<T>,
        /* pointer alignment: Unaligned */   0,
        /* pointer increment btn elements */ Eigen::InnerStride<2>>;
        ExcitationsVector muscleExcit(controls.data() + _numCoordActuators,
                                      _numMuscles);

        integrand = controls.head(_numCoordActuators).squaredNorm()
                  + muscleExcit.squaredNorm();
    }
    MuscleRedundancySolver::Solution interpret_solution(
            const mesh::OptimalControlSolution& ocp_sol) const
    {

        MuscleRedundancySolver::Solution sol;
        if (_numCoordActuators) {
            sol.other_controls.setColumnLabels(_otherControlsLabels);
        }
        if (_numMuscles) {
            sol.excitation.setColumnLabels(_muscleLabels);
            sol.activation.setColumnLabels(_muscleLabels);
            sol.norm_fiber_length.setColumnLabels(_muscleLabels);
            sol.norm_fiber_velocity.setColumnLabels(_muscleLabels);
        }

        for (int i_time = 0; i_time < ocp_sol.time.cols(); ++i_time) {
            const auto& time = ocp_sol.time[i_time];
            const auto& controls = ocp_sol.controls.col(i_time);
            const auto& states = ocp_sol.states.col(i_time);

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
                                        1 /* stride: skip over fiber vel. */,
                                        controls.data() + _numCoordActuators,
                                        true /* makes this a view */);
            sol.excitation.appendRow(time, excitation);

            SimTK::RowVector activation(_numMuscles,
                                        1 /* stride: skip over fiber length */,
                                        states.data(),
                                        true /* makes this a view */);
            sol.activation.appendRow(time, activation);

            SimTK::RowVector fiber_length(_numMuscles,
                                          1 /* stride: skip over activation */,
                                          states.data() + 1,
                                          true /* makes this a view */);
            sol.norm_fiber_length.appendRow(time, fiber_length);

            SimTK::RowVector fiber_velocity(_numMuscles,
                                       1 /* stride: skip over excit. */,
                                       controls.data() + _numCoordActuators + 1,
                                       true /* makes this a view */);
            sol.norm_fiber_velocity.appendRow(time, fiber_velocity);
        }
        return sol;
    }
private:
    const MuscleRedundancySolver& _mrs;
    Model _model;
    const GCVSplineSet& _inverseDynamics;
    double _initialTime = SimTK::NaN;
    double _finalTime = SimTK::NaN;

    // Bookkeeping.
    int _numDOFs;
    int _numCoordActuators;
    int _numMuscles;
    std::vector<std::string> _muscleLabels;
    std::vector<std::string> _otherControlsLabels;

    // Cached quantities to use during the optimization.
    Eigen::MatrixXd _desiredMoments;
    Eigen::MatrixXd _muscleTendonLength;
    // TODO make general (CoordinateActuator optimal force).
    double _controlToMoment;

    // Muscle parameters.
    Eigen::VectorXd _max_isometric_force;
    // Units: meters.
    Eigen::VectorXd _optimal_fiber_length;
    // Units: meters.
    Eigen::VectorXd _tendon_slack_length;
    // Units: optimal fiber lengths per second.
    Eigen::VectorXd _max_contraction_velocity;
    // Units: radians.
    Eigen::VectorXd _pennation_angle_at_optimal;
};

MuscleRedundancySolver::MuscleRedundancySolver() {
    //constructProperty_model_file("");
    //constructProperty_kinematics_file("");
}

GCVSplineSet
MuscleRedundancySolver::computeInverseDynamics() const {
    Model modelForID(_model);
    modelForID.finalizeFromProperties();
    // Disable all actuators in the model, as we don't want them to
    // contribute generalized forces that would reduce the inverse
    // dynamics moments to track.
    auto actuators = modelForID.updComponentList<Actuator>();
    for (auto& actuator : actuators) {
        actuator.set_appliesForce(false);
    }

    InverseDynamicsSolver invdyn(modelForID);
    SimTK::State state = modelForID.initSystem();

    // Assemble functions for coordinate values. Functions must be in the
    // same order as the joint moments (multibody tree order).
    auto coords = modelForID.getCoordinatesInMultibodyTreeOrder();
    std::vector<std::string> columnLabels(coords.size());
    for (size_t i = 0; i < coords.size(); ++i) {
        // The first state variable in the coordinate should be its value.
        // TODOosim make it easy to get the full name of a state variable
        // Perhaps expose the StateVariable class.
        //columnLabels[i] = coords[i]->getStateVariableNames()[0];
        // This will yield something like "knee/flexion/value".
        columnLabels[i] = ComponentPath(coords[i]->getAbsolutePathName())
                .formRelativePath(modelForID.getAbsolutePathName()).toString()
                + "/value";
    }
    FunctionSet coordFunctions =
            createGCVSplineSet(getKinematicsData(), columnLabels);

    // Convert normalized mesh points into times at which to evaluate
    // net joint moments.
    // TODO this variant ignores our data for generalized speeds.
    const auto& times = getKinematicsData().getIndependentColumn();
    // TODO avoid copy.
    SimTK::Array_<double> simtkTimes(times); // , SimTK::DontCopy());

    // Perform Inverse Dynamics.
    // -------------------------
    SimTK::Array_<SimTK::Vector> forceTrajectory;
    invdyn.solve(state, coordFunctions, simtkTimes, forceTrajectory);

    // Post-process Inverse Dynamics results.
    // --------------------------------------
    Storage forceTrajectorySto;
    const size_t numDOFs = forceTrajectory[0].size();
    OpenSim::Array<std::string> labels("", numDOFs);
    for (size_t i = 0; i < numDOFs; ++i) {
        labels[i] = "force" + std::to_string(i);
    }
    forceTrajectorySto.setColumnLabels(labels);
    for (size_t i_time = 0; i_time < forceTrajectory.size(); ++i_time) {
        forceTrajectorySto.append(times[i_time], forceTrajectory[i_time]);
    }
    // Filter; otherwise, inverse dynamics moments are too noisy.
    forceTrajectorySto.pad(forceTrajectorySto.getSize() / 2);
    // TODO make the filter frequency a parameter.
    forceTrajectorySto.lowpassIIR(6);
    return GCVSplineSet(5, &forceTrajectorySto);
}

MuscleRedundancySolver::Solution MuscleRedundancySolver::solve() {

    // Precompute quantities.
    // ----------------------
    std::vector<std::string> actuatorsToUse;
    auto actuators = _model.getComponentList<ScalarActuator>();
    for (const auto& actuator : actuators) {
        if (actuator.get_appliesForce()) {
            actuatorsToUse.push_back(actuator.getAbsolutePathName());
        }
    }

    // Run inverse dynamics.
    // ---------------------
    const auto forceTrajectory = computeInverseDynamics();


    // Solve the optimal control problem.
    // ----------------------------------
    auto ocp = std::make_shared<MRSProblemSeparate<adouble>>(*this,
                                                             forceTrajectory);
    ocp->print_description();
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt",
                                                  100);
    mesh::OptimalControlSolution ocp_solution = dircol.solve();

    // Return the solution.
    // --------------------
    // TODO remove
    ocp_solution.write("MuscleRedundancySolver_OCP_solution.csv");
    return ocp->interpret_solution(ocp_solution);
}
