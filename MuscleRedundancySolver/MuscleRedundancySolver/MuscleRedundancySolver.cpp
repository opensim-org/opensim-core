

#include "MuscleRedundancySolver.h"

#include <mesh.h>
#include <OpenSim/OpenSim.h>
// TODO should not be needed/**/:
#include <OpenSim/Simulation/InverseDynamicsSolver.h>

using namespace OpenSim; // TODO should not be necessary (see header).

GCVSplineSet createGCVSplineSet(const TimeSeriesTable& table,
                                const std::vector<std::string>& labels) {
    GCVSplineSet set;
    const int degree = 5;
    const double errorVariance = 0.0;
    const auto& time = table.getIndependentColumn();
    for (const auto& label : labels) {
        const auto& column = table.getDependentColumn(label);
        set.adoptAndAppend(new GCVSpline(degree, column.size(), time.data(),
                                         &column[0], label, errorVariance));
    }
    return set;
}

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

        // Add a control for each enabled actuator.
        auto actuators = _model.getComponentList<ScalarActuator>();
        for (const auto& actuator : actuators) {
            if (actuator.get_appliesForce()) {
                this->add_control(actuator.getAbsolutePathName(),
                                  {actuator.get_min_control(),
                                   actuator.get_max_control()});
            }
        }

        // Add a constraint for each joint moment.
        for (int i = 0; i < state.getNU(); ++i) {
            this->add_path_constraint("joint_moment_" + std::to_string(i), 0);
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
            if (auto* sa = dynamic_cast<const ScalarActuator*>(&actuator)) {
                // TODO temporary
                mutableThis->_controlToMoment = sa->getOptimalForce();
            }
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
        std::cout << "DEBUG " << this->_desiredMoments << std::endl;
        mesh::write(times, this->_desiredMoments,
                    "DEBUG_desiredMoments.csv",
                    columnLabels);
    }
    //void dynamics(const mesh::VectorX<T>& /*states*/,
    //              const mesh::VectorX<T>& controls,
    //              Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
    //}
    //virtual void path_constraints(const T& time,
    //                              const VectorX<T>& /*states*/,
    //                              const VectorX<T>& controls,
    //                              Eigen::Ref<VectorX<T>> constraints) const {
    //
    //}
    void path_constraints(unsigned i_mesh,
                          const T& /*time*/,
                          const mesh::VectorX<T>& /*states*/,
                          const mesh::VectorX<T>& controls,
                          Eigen::Ref<mesh::VectorX<T>> constraints)
            const override {
        // /*const TODO*/ auto generatedMoments = _mrs.controlToMoment * controls;
        // TODO constraints = _desiredMoments[index] - generatedMoments;
        // TODO constraints = this->_desiredMoments[i_mesh] - controls;
        constraints.setZero(); // TODO is this necessary?
        // TODO don't do subtractions individually...
        for (int i_con = 0; i_con < constraints.size(); ++i_con) {
            constraints[i_con] = this->_desiredMoments(i_con, i_mesh) -
                            this->_controlToMoment * controls[i_con];
        }
    }
    //virtual void path_constraints(const T& time,
    //                              const MatrixX<T>& /*states*/,
    //                              const MatrixX<T>& controls,
    //                              Eigen::Ref<MatrixX<T>> constraints) const {
    //    TODO desired_moments =;
    //    TODO generated_moments = TODO matrix multiplication controls;
    //    constraints = desired_moments - generated_moments;
    //}
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
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
    Eigen::MatrixXd _desiredMoments;
    // TODO make general.
    double _controlToMoment;
};

MuscleRedundancySolver::MuscleRedundancySolver() {
    //constructProperty_model_file("");
    //constructProperty_kinematics_file("");
}

MuscleRedundancySolver::Solution MuscleRedundancySolver::solve() {


    // Run muscle analysis.
    // --------------------
    // TODO

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
                                                  20);
    mesh::OptimalControlSolution ocp_solution = dircol.solve();

    // Return the solution.
    // --------------------
    // TODO remove
    ocp_solution.write("MuscleRedundancySolver_OCP_solution.csv");
    Solution solution;
    solution.excitations.setColumnLabels(actuatorsToUse);
    for (int i = 0; i < ocp_solution.time.cols(); ++i) {
        // Each column of controls is a different time.
        SimTK::RowVector controls(ocp_solution.controls.rows(),
                                  ocp_solution.controls.col(i)[0]);
        solution.excitations.appendRow(ocp_solution.time[i], controls);
    }
    return solution;
}
