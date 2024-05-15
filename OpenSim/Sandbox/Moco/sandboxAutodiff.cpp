#include <iostream>
#include <casadi/casadi.hpp>

using namespace casadi;

class MyCallback : public Callback {
public:
    MyCallback(const std::string& name, double d, const Dict& opts = Dict()) : d(d) {
        construct(name, opts);
    }

    casadi_int get_n_in() override { return 1; }
    casadi_int get_n_out() override { return 1; }

    void init() override {
        std::cout << "initializing object" << std::endl;
    }

    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        DM x = arg[0];
        DM f = sin(d * x);
        return {f};
    }

private:
    double d;
};

class Example4To3 : public Callback {
public:
    Example4To3(const std::string& name, const Dict& opts = Dict()) {
        construct(name, opts);
    }

    casadi_int get_n_in() override { return 1; }
    casadi_int get_n_out() override { return 1; }

    Sparsity get_sparsity_in(casadi_int i) override {
        return Sparsity::dense(4, 1);
    }

    Sparsity get_sparsity_out(casadi_int i) override {
        return Sparsity::dense(3, 1);
    }

    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        DM a = arg[0](Slice(0, 1));
        DM b = arg[0](Slice(1, 2));
        DM c = arg[0](Slice(2, 3));
        DM d = arg[0](Slice(3, 4));
        DM ret = vertcat(sin(c)*d + d*d, 2*a + c, b*b + 5*c);
        return {ret};
    }
};

class ForwardFun : public Callback {
public:
    ForwardFun(const std::string& name, const Dict& opts = Dict()) {
        construct(name, opts);
    }

    casadi_int get_n_in() override { return 3; }
    casadi_int get_n_out() override { return 1; }

    Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0) return Sparsity::dense(4, 1);   // nominal input
        else if (i == 1) return Sparsity(3, 1);     // nominal output
        else return Sparsity::dense(4, 1);          // Forward seed
    }

    Sparsity get_sparsity_out(casadi_int i) override {
        return Sparsity::dense(3, 1);               // Forward sensitivity
    }

    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        DM a = arg[0](Slice(0, 1));
        DM b = arg[0](Slice(1, 2));
        DM c = arg[0](Slice(2, 3));
        DM d = arg[0](Slice(3, 4));
        DM a_dot = arg[2](Slice(0, 1));
        DM b_dot = arg[2](Slice(1, 2));
        DM c_dot = arg[2](Slice(2, 3));
        DM d_dot = arg[2](Slice(3, 4));

        std::cout << "Forward sweep with " << a_dot << b_dot << c_dot << d_dot << std::endl;

        DM w0 = sin(c);
        DM w0_dot = cos(c) * c_dot;
        DM w1 = w0 * d;
        DM w1_dot = w0_dot * d + w0 * d_dot;
        DM w2 = d * d;
        DM w2_dot = 2 * d_dot * d;
        DM r0 = w1 + w2;
        DM r0_dot = w1_dot + w2_dot;
        DM w3 = 2 * a;
        DM w3_dot = 2 * a_dot;
        DM r1 = w3 + c;
        DM r1_dot = w3_dot + c_dot;
        DM w4 = b * b;
        DM w4_dot = 2 * b_dot * b;
        DM w5 = 5 * w0;
        DM w5_dot = 5 * w0_dot;
        DM r2 = w4 + w5;
        DM r2_dot = w4_dot + w5_dot;

        return {vertcat(r0_dot, r1_dot, r2_dot)};
    }
};

class Example4To3_Fwd : public Example4To3 {
public:
    Example4To3_Fwd(const std::string& name, const Dict& opts = Dict()) : 
            Example4To3(name, opts) {}

    bool has_forward(casadi_int nfwd) const override { return nfwd == 1; }

    Function get_forward(casadi_int nfwd, const std::string& name, 
                const std::vector<std::string>& inames, 
                const std::vector<std::string>& onames, 
                const Dict& opts) const override {

        ForwardFun fwd(name, opts);
        return fwd;
    }

};


int main() {
    MyCallback f("f", 0.5);
    std::cout << f(DM(2)) << std::endl;

    MX x = MX::sym("x");
    std::cout << f(x) << std::endl;

    double eps = 1e-5;
    std::cout << (f(DM(2) + eps) - f(DM(2))) / eps << std::endl;

    MyCallback f_fd("f", 0.5, {{"enable_fd", true}});
    Function J = Function("J", {x}, {jacobian(f_fd(x)[0], x)});
    std::cout << J(DM(2)) << std::endl;

    Example4To3_Fwd f_fwd("f_fwd");
    std::cout << "f_fwd.has_forward(1)(): " << f_fwd.has_forward(1) << std::endl;
    x = MX::sym("x", 4);
    J = Function("J", {x}, {jacobian(f_fwd(x)[0], x)});
    std::cout << J(DM::vertcat({1, 2, 0, 3})) << std::endl;

    return 0;
}