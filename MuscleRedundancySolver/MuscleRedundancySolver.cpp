

#include "MuscleRedundancySolver.h"

#include <mesh.h>
#include <OpenSim/OpenSim.h>

using namespace OpenSim; // TODO should not be necessary (see header).

template<typename T>
class SlidingMassNew : public mesh::OptimalControlProblemNamed<T> {
public:
    SlidingMassNew() {
        // TODO when time is a variable, this has to be more advanced:
        this->set_time({0}, {2});
        this->add_state("x", {0, 2}, {0}, {1});
        this->add_state("u", {-10, 10}, {0}, {0});
        this->add_control("F", {-50, 50});
        // this->add_state("x", Bounds(  0,  2), InitialBounds(0), FinalBounds(1));
        // this->add_state("u", Bounds(-10, 10), InitialBounds(0), FinalBounds(0));
        // this->add_control("F", Bounds(-50, 50));
    }
    const double mass = 10.0;
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override
    {
        derivatives[0] = states[1];
        derivatives[1] = controls[0] / mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    void integral_cost(const T& /*time*/,
                       const mesh::VectorX<T>& /*states*/,
                       const mesh::VectorX<T>& controls,
                       T& integrand) const override {
        integrand = controls[0] * controls[0];
    }
};

MuscleRedundancySolver::MuscleRedundancySolver() {
    constructProperty_model_file("");
    constructProperty_coordinates_file("");
}

void MuscleRedundancySolver::run() const
{
    // Run inverse dynamics.
    // Run muscle analysis.
    OpenSim::Model model;
    auto ocp = std::make_shared<SlidingMassNew<adouble>>();
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt");
    mesh::OptimalControlSolution solution = dircol.solve();
}
