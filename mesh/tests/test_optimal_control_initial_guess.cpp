
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include "testing.h"

#include <mesh.h>

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::Matrix2d;

using namespace mesh;

template<typename T>
class FinalPositionLocalOptima : public mesh::OptimalControlProblemNamed<T> {
public:
    FinalPositionLocalOptima()
    {
        this->set_time(0, 1);
        this->add_state("x", {0, 3.5}, {0}, {0.5, 3.5});
        this->add_state("v", {-10, 10}, {0}, {0});
        this->add_control("F", {-50, 50});
    }
    void dynamics(const VectorX<T>& states, const VectorX<T>& controls,
                  Ref<VectorX<T>> derivatives) const override
    {
        derivatives[0] = states[1];
        derivatives[1] = controls[0];
    }
    void endpoint_cost(const T& /*final_time*/,
                       const VectorX<T>& final_states,
                       T& cost) const override
    {
        cost = pow(final_states[0] - 2, 2);
    }
};

TEST_CASE("Final position cost with two local optima", "[initial_guess]") {

    auto ocp = std::make_shared<FinalPositionLocalOptima<adouble>>();
    DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt", 100);
    OptimalControlSolution solution = dircol.solve();
    solution.write("final_position_local_optima_solution.csv");

}
