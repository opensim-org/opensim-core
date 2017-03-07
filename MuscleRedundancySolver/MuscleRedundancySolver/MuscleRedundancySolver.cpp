

#include "MuscleRedundancySolver.h"

#include <mesh.h>
#include <OpenSim/OpenSim.h>
// TODO should not be needed/**/:
#include <OpenSim/Simulation/InverseDynamicsSolver.h>

using namespace OpenSim;

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
    MRSProblemSeparate(const MuscleRedundancySolver& mrs)
            : mesh::OptimalControlProblemNamed<T>("MRS"),
              _mrs(mrs), _model(mrs.getModel()) {
        SimTK::State state = _model.initSystem();

        // Set the time bounds.
        // TODO when time is a variable, this has to be more advanced:
        const auto& times = _mrs.getKinematicsData().getIndependentColumn();
        this->_initialTime = times.front();
        this->_finalTime = times.back();
        this->set_time({this->_initialTime}, {this->_finalTime});

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
            this->_controlToMoment = actuator.getOptimalForce();

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

            _numMuscles++;
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

        // Run inverse dynamics, evaluated at all mesh points.
        // ---------------------------------------------------

        // Disable all actuators in the model, as we don't want them to
        // contribute generalized forces that would reduce the inverse
        // dynamics moments to track.
        auto actuators = mutableThis->_model
                .template updComponentList<Actuator>();
        for (auto& actuator : actuators) {
            actuator.set_appliesForce(false);
        }

        InverseDynamicsSolver invdyn(_model);
        SimTK::State state = mutableThis->_model.initSystem();

        // Assemble functions for coordinate values. Functions must be in the
        // same order as the joint moments (multibody tree order).
        auto coords = _model.getCoordinatesInMultibodyTreeOrder();
        std::vector<std::string> columnLabels(coords.size());
        for (size_t i = 0; i < coords.size(); ++i) {
            // The first state variable in the coordinate should be its value.
            // TODOosim make it easy to get the full name of a state variable
            // Perhaps expose the StateVariable class.
            //columnLabels[i] = coords[i]->getStateVariableNames()[0];
            // This will yield something like "knee/flexion/value".
            columnLabels[i] = ComponentPath(coords[i]->getAbsolutePathName())
                    .formRelativePath(_model.getAbsolutePathName()).toString()
                    + "/value";
        }
        FunctionSet coordFunctions =
                createGCVSplineSet(_mrs.getKinematicsData(), columnLabels);

        // Convert normalized mesh points into times at which to evaluate
        // net joint moments.
        // TODO this variant ignores our data for generalized speeds.
        Eigen::VectorXd times = (_finalTime - _initialTime) * mesh;
        SimTK::Array_<double> simtkTimes(
                times.data(), times.data() + times.size(), SimTK::DontCopy());

        // Compute the desired joint moments.
        SimTK::Array_<SimTK::Vector> forceTrajectory;
        invdyn.solve(state, coordFunctions, simtkTimes, forceTrajectory);

        // Store the desired joint moments in an Eigen matrix.
        // TODO probably has to be VectorX<T> to use with subtraction.
        mutableThis->_desiredMoments.resize(forceTrajectory[0].size(),
                                            times.size());
        for (size_t i = 0; i < size_t(times.size()); ++i) {
            mutableThis->_desiredMoments.col(i) = Eigen::Map<Eigen::VectorXd>(
                    &forceTrajectory[i][0], forceTrajectory[i].size());
        }
        // TODO looks very noisy:
        //std::cout << "DEBUG " << this->_desiredMoments << std::endl;
        mesh::write(times, this->_desiredMoments,
                    "DEBUG_desiredMoments.csv",
                    columnLabels);


        // Muscle Analysis: muscle-tendon length.
        // --------------------------------------
        // TODO tailored to hanging mass: just use absoluate value of the
        // coordinate value.
        mutableThis->_muscleTendonLength.resize(1, times.size());
        const auto& kinematics = this->_mrs.getKinematicsData();
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
        for (Eigen::Index i_act = 0; i_act < this->_numMuscles; ++i_act)
        {
            const Eigen::Index i_mus = i_act + this->_numCoordActuators;

            // Unpack variables.
            const T& excitation = controls[2 * i_mus];
            const T& activation = states[2 * i_mus];
            const T& normFibVel = controls[2 * i_mus + 1];

            // Activation dynamics.
            //     f = 0.5 tanh(b(e - a))
            //     z = (0.5 + 1.5a)
            // da/dt = [(f + 0.5)/(tau_a * z) + (-f + 0.5)*z/tau_d] * (e - a)
            const T temp1 = 0.5 + 1.5 * activation;
            const T tempAct = 1.0 / (actTimeConst * temp1);
            const T tempDeact = temp1 / deactTimeConst;
            const T f = 0.5 * tanh(tanhSteepness * (excitation - activation));
            const T timeConst = tempAct * (f + 0.5) + tempDeact * (-f + 0.5);
            derivatives[2 * i_act] = timeConst * (excitation - activation);

            // Fiber dynamics.
            derivatives[2 * i_act + 1] =
                    this->_max_contraction_velocity * normFibVel;
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
        mesh::VectorX<T> genForce(this->_numDOFs);
        genForce.setZero();

        // CoordinateActuators.
        // --------------------
        for (Eigen::Index i_act = 0; i_act < this->_numCoordActuators; ++i_act)
        {
            genForce[0] += this->_controlToMoment * controls[i_act];
        }

        // Muscles.
        // --------
        for (Eigen::Index i_act = 0; i_act < this->_numMuscles; ++i_act) {
            const Eigen::Index i_mus = i_act + this->_numCoordActuators;

            // Unpack variables.
            // -----------------
            const T& activation = states[2 * i_mus];
            const T& normFibVel = controls[2 * i_mus + 1];
            const T& normFibLen = states[2 * i_mus + 1];

            // Intermediate quantities.
            // ------------------------
            const T fibLen = normFibLen * this->_optimal_fiber_length;
            // TODO cache this somewhere; this is constant.
            const T fibWidth = this->_optimal_fiber_length
                             * sin(this->_pennation_angle_at_optimal);
            // Tendon length.
            const T& musTenLen = this->_muscleTendonLength(i_act, i_mesh);
            // lT = lMT - sqrt(lM^2 - w^2)
            const T tenLen = musTenLen
                           - sqrt(fibLen*fibLen - fibWidth*fibWidth);
            const T normTenLen = tenLen / this->_tendon_slack_length;
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
            genForce[0] += -this->_max_isometric_force * normTenForce;
        }


        // Achieve the motion.
        // ===================
        // /*const TODO*/ auto generatedMoments = _mrs.controlToMoment * controls;
        // TODO constraints = _desiredMoments[index] - generatedMoments;
        constraints.segment(this->_numMuscles, this->_numDOFs)
                = this->_desiredMoments.col(i_mesh).template cast<adouble>()
                - genForce;

        // TODO std::cout << "DEBUG constraints " << constraints << std::endl;
    }
    void integral_cost(const T& /*time*/,
                       const mesh::VectorX<T>& /*states*/,
                       const mesh::VectorX<T>& controls,
                       T& integrand) const override {
        integrand = controls.squaredNorm();
    }
private:
    const MuscleRedundancySolver& _mrs;
    Model _model;
    double _initialTime = SimTK::NaN;
    double _finalTime = SimTK::NaN;
    int _numDOFs;
    int _numCoordActuators;
    int _numMuscles;
    Eigen::MatrixXd _desiredMoments;
    Eigen::MatrixXd _muscleTendonLength;
    Eigen::MatrixXd _muscleTendonVelocity;
    // TODO make general (CoordinateActuator optimal force).
    double _controlToMoment;

    // TODO muscle parameters.
    // Units: optimal fiber lengths per second.
    double _max_contraction_velocity = 10;
    // Units: meters.
    double _optimal_fiber_length = 0.10;
    // Units: radians. TODO
    double _pennation_angle_at_optimal = 0.1;
    double _tendon_slack_length = 0.10;
    double _max_isometric_force = 10;
};

MuscleRedundancySolver::MuscleRedundancySolver() {
    //constructProperty_model_file("");
    //constructProperty_kinematics_file("");
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

    // Solve the optimal control problem.
    // ----------------------------------
    auto ocp = std::make_shared<MRSProblemSeparate<adouble>>(*this);
    ocp->print_description();
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt",
                                                  100);
    mesh::OptimalControlSolution ocp_solution = dircol.solve();

    // Return the solution.
    // --------------------
    // TODO remove
    ocp_solution.write("MuscleRedundancySolver_OCP_solution.csv");
    Solution solution;
    solution.excitations.setColumnLabels(actuatorsToUse);
    solution.activations.setColumnLabels(actuatorsToUse);
    for (int i = 0; i < ocp_solution.time.cols(); ++i) {
        // Each column of controls is a different time.
        if (ocp_solution.controls.rows()) {
            SimTK::RowVector controls(ocp_solution.controls.rows(),
                                      ocp_solution.controls.col(i)[0]);
            solution.excitations.appendRow(ocp_solution.time[i], controls);
        }

        if (ocp_solution.states.rows()) {
            SimTK::RowVector states(ocp_solution.states.rows(),
                                    ocp_solution.states.col(i)[0]);
            solution.activations.appendRow(ocp_solution.time[i], states);
        }
    }
    return solution;
}
