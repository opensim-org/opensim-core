
#include <casadi/casadi.hpp>

#include <tropter/tropter.h>

using namespace tropter;

using namespace casadi;
class MyCallback : public Callback {
    // Data members
    double d;
public:
    // Constructor
    MyCallback(const std::string& name, double d,
            const Dict& opts=Dict()) : d(d) {
        construct(name, opts);
    }

    // Destructor
    ~MyCallback() override {}

    // Number of inputs and outputs
    casadi_int get_n_in() override { return 2;}
    casadi_int get_n_out() override { return 1;}

    // Initialize the object
    void init() override {
        std::cout << "initializing object" << std::endl;
        d = 5;
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        DM f = arg.at(1) - arg.at(0) * arg.at(0);
        return {f};
    }
};

template<typename T>
class SlidingMass : public tropter::Problem<T> {
public:
    SlidingMass() {
        this->set_time({0}, {2});
        this->add_state("x", {0, 2}, {0}, {1});
        this->add_state("u", {-10, 10}, {0}, {0});
        this->add_control("F", {-50, 50});
    }
    const double mass = 10.0;
    void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const override {
        out.dynamics[0] = in.states[1];
        out.dynamics[1] = in.controls[0] / mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    void calc_integral_cost(const Input<T>& in, T& integrand) const override {
        const auto& controls = in.controls;
        integrand = controls[0] * controls[0];
    }
};


class EndpointCostFunction : public Callback {
public:
    EndpointCostFunction(const std::string& name, const Problem<double>& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~EndpointCostFunction() override {}
    casadi_int get_n_in() override { return p.get_num_states() + 1; }
    casadi_int get_n_out() override { return 1;}
    void init() override {
        std::cout << "initializing object" << std::endl;
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double final_time = double(arg.at(0));
        Eigen::VectorXd final_states(p.get_num_states());
        for (int i = 0; i < final_states.size(); ++i) {
            final_states[i] = double(arg.at(i + 1));
        }
        Eigen::VectorXd parameters;
        double cost;
        p.calc_cost(0, final_time, final_states);
        DM f = cost;
        return {f};
    }
private:
    const Problem<double>& p;
};

class IntegrandCostFunction : public Callback {
public:
    IntegrandCostFunction(const std::string& name, const Problem<double>& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~IntegrandCostFunction() override {}
    casadi_int get_n_in() override { return p.get_num_states() + p.get_num_controls() + 1; }
    casadi_int get_n_out() override { return 1;}
    void init() override {
        std::cout << "initializing object" << std::endl;
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        Eigen::VectorXd states(p.get_num_states());
        for (int i = 0; i < states.size(); ++i) {
            states[i] = double(arg.at(i + 1));
        }
        Eigen::VectorXd controls(p.get_num_controls());
        for (int i = 0; i < states.size(); ++i) {
            states[i] = double(arg.at(i + 1 + p.get_num_controls()));
        }
        double integrand;
        tropter::Input<double> in {
            0, time, states, controls, Eigen::VectorXd(), Eigen::VectorXd()};
        p.calc_cost_integrand(0, in, integrand);
        DM f = integrand;
        return {f};
    }
private:
    const Problem<double>& p;
};

class Defect : public Callback {
public:
    Defect(const std::string& name, const Problem<double>& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~IntegrandCost() override {}
    casadi_int get_n_in() override { return p.get_num_states() + p.get_num_controls() + 1; }
    casadi_int get_n_out() override { return 1;}
    void init() override {
        std::cout << "initializing object" << std::endl;
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        Eigen::VectorXd states(p.get_num_states());
        for (int i = 0; i < states.size(); ++i) {
            states[i] = double(arg.at(i + 1));
        }
        Eigen::VectorXd controls(p.get_num_controls());
        for (int i = 0; i < states.size(); ++i) {
            states[i] = double(arg.at(i + 1 + p.get_num_controls()));
        }
        double integrand;
        tropter::Input<double> in {
                0, time, states, controls, Eigen::VectorXd(), Eigen::VectorXd()};
        p.calc_differential_algebraic_equations(in, integrand);
        DM f = integrand;
        return {f};
    }
private:
    const Problem<double>& p;
};
class MucoCasADiSolver {
public:
    MucoCasADiSolver(const Problem<double>& problem) : p(problem) {}
    void solve() const {
        casadi::Opti opt;
        int N = 20;
        MX tf = opt.variable();
        MX states = opt.variable(p.get_num_states(), N);
        auto controls = opt.variable(p.get_num_controls(), N);
        EndpointCostFunction endpoint_cost_function("endpoint_cost", p, {{"enable_fd", true}});

        std::vector<MX> args;
        args.push_back(tf);
        for (int i = 0; i < p.get_num_states(); ++i) {
            args.push_back(states(i, N - 1));
        }

        auto endpoint_cost = endpoint_cost_function(args);

        IntegrandCostFunction integrand_cost("integrand", p, {{"enable_fd", true}});
        MX integral = opt.variable();
        integral = 0;
        for (int i = 0; i < N; ++i) {
            std::vector<MX> iargs;
            iargs.push_back(tf);
            for (int is = 0; is < p.get_num_states(); ++is) {
                iargs.push_back(states(is, i));
            }
            for (int ic = 0; ic < p.get_num_controls(); ++ic) {
                iargs.push_back(controls(ic, i));
            }
            integral += integrand_cost(iargs).at(0);
        }
        opt.solver("ipopt");
        opt.minimize(endpoint_cost.at(0) + integral);

        for (int itime = 0; itime < N; ++itime) {

            opt.subject_to();
        }
        auto solution = opt.solve();
    }
private:
    const Problem<double>& p;
};

int main() {
    MucoCasADiSolver solver(SlidingMass<double>{});
    solver.solve();
    /*
    casadi::Opti opti;
    auto x = opti.variable();
    auto y = opti.variable();
    auto callback = MyCallback("f", 5, {{ "enable_fd", true}});
    std::vector<MX> args;
    args.push_back(x);
    args.push_back(y);
    auto f = callback(args);
    opti.minimize( f.at(0) * f.at(0));
    opti.subject_to( x * x + y * y == 1 );
    opti.subject_to( x + y >= 1);
    opti.solver("ipopt");
    auto sol = opti.solve();
    sol.value(x);
    sol.value(y);
     */


    std::cout << "DEBUG " << std::endl;
    return 0;
}
