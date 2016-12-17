
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <mesh.h>

#include <Eigen/LU>

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::Matrix2d;

using namespace mesh;

template<typename T>
class DoublePendulum : public mesh::OptimalControlProblemNamed<T> {
public:
    constexpr static const double g = 9.81;
    double L0 = 1;
    double L1 = 1;
    double m0 = 1;
    double m1 = 1;

    void dynamics(const VectorX<T>& x, const VectorX<T>& tau,
            Ref<VectorX<T>> xdot) const override final
    {
        const auto& q0 = x[0];
        const auto& q1 = x[1];
        const auto& u0 = x[2];
        const auto& u1 = x[3];
        const auto& L0 = this->L0;
        const auto& L1 = this->L1;
        const auto& m0 = this->m0;
        const auto& m1 = this->m1;
        xdot[0] = u0;
        xdot[1] = u1;

        const T z0 = m1 * L0 * L1 * cos(q1);
        const T M01 = m1 * L1*L1 + z0;
        Matrix2<T> M;
        M << m0 * L0*L0 + m1 * (L0*L0 + L1*L1) + 2*z0,    M01,
             M01,                                         m1 * L1*L1;
        Vector2<T> V(-u1 * (2 * u0 + u1),
                     u0 * u0);
        V *= m1*L0*L1*sin(q1);
        Vector2<T> G(g * ((m0 + m1) * L0 * cos(q0) + m1 * L1 * cos(q0 + q1)),
                     g * m1 * L1 * cos(q0 + q1));

        //Vector2<T> drag(-10 * u0, -10 * u1);
        // TODO xdot.tail<2>() =
        xdot.tail(2) = M.inverse() * (tau - (V + G));
    }
};

template<typename T>
class DoublePendulumHorizontalVertical : public DoublePendulum<T> {
public:
    DoublePendulumHorizontalVertical()
    {
        this->set_time(0, 2); //{0, 5});
        // TODO fix allowing final bounds to be unconstrained.
        this->add_state("q0", {-10, 10}, {0});
        this->add_state("q1", {-10, 10}, {0});
        this->add_state("u0", {-20, 20}, {0}, {0});
        this->add_state("u1", {-20, 20}, {0}, {0});
        this->add_control("tau0", {-200, 200});
        this->add_control("tau1", {-200, 200});
    }
    //void terminal_cost(const double& final_time, const VectorX<T>&
    // final_states, const VectorX<T>& final_controls, T& cost)
    // TODO want to take const T& final_time.
    void endpoint_cost(const T& /*final_time*/,
            const VectorX<T>& final_states,
            T& cost) const override
    {
        // TODO a final state constraint probably makes more sense.
        const auto& q0 = final_states[0];
        const auto& q1 = final_states[1];
        const auto& L0 = this->L0;
        const auto& L1 = this->L1;
        Vector2<T> actual_location(L0 * cos(q0) + L1 * cos(q0 + q1),
                                   L0 * sin(q0) + L1 * sin(q0 + q1));
        const Vector2<T> desired_location(0, 2);
        cost = 10.0 * (actual_location - desired_location).squaredNorm();
        // cost = final_time;
    }
    void integral_cost(const T& /*time*/,
            const VectorX<T>& /*states*/,
            const VectorX<T>& controls,
            T& integrand) const override
    {
        // TODO different penalties for tau0 vs. tau1.
        integrand = 0.001 * controls.squaredNorm();
    }
};

TEST_CASE("Double pendulum horizontal to vertical", "[adolc][trapezoidal]")
{
    auto ocp = std::make_shared<DoublePendulumHorizontalVertical<adouble>>();
    DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt", 100);
    OptimalControlSolution solution = dircol.solve();
    solution.write("double_pendulum_horizontal_to_vertical_solution.csv");

}

