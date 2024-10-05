#include <casadi/casadi.hpp>
#include <OpenSim/Moco/osimMoco.h>

// using namespace casadi;
using namespace OpenSim;


// Define the MyCallback class inheriting from casadi::Callback
class MyCallback : public casadi::Callback {
public:
    MyCallback(const std::string& name, double d, 
            const casadi::Dict& opts = casadi::Dict()) : d_(d) {
        this->construct(name, opts);
    }
    virtual ~MyCallback() = default;

protected:
    virtual casadi_int get_n_in() override final { return 1; }
    virtual casadi_int get_n_out() override final { return 1; }

    // virtual void init() override {
    //     std::cout << "initializing object" << std::endl;
    // }

    virtual casadi::DMVector eval(const casadi::DMVector& args) const override {
        auto x = args[0];
        double result = d_ * x.scalar();
        double f = casadi::sin(result);
        casadi::DMVector res((int)n_out());
        res[0] = casadi::DM(f);
        return res;
    }
private:
    double d_;

};

void exampleMyCallback() {
    // Use the function
    MyCallback f("f", 0.5);
    casadi::DM res = f(casadi::DM(2.0));
    std::cout << "result: " << res << std::endl;

    // You may call the Callback symbolically
    casadi::MX x = casadi::MX::sym("x");
    std::cout << "f(x): " << f(x) << std::endl;

    // Derivates OPTION 1: finite-differences
    casadi::DM eps = 1e-5;
    casadi::DM dfdx = (f(2+eps)-f(casadi::DM(2)))/eps;
    std::cout << "finite diff manual: " << dfdx << std::endl;

    casadi::Dict opts = {{"enable_fd", true}};
    MyCallback f_fd("f", 0.5, opts);
    casadi::MX dfdx_fd = jacobian(f_fd(x)[0], x);
    casadi::Function J("J",{x}, {dfdx_fd});
    std::cout << "finite diff: " << J(casadi::DM(2.0))[0] << std::endl;
}

// see https://groups.google.com/g/casadi-users/c/e663pbPqHLU
// see https://github.com/casadi/casadi/issues/2490
class Example4To3 : public casadi::Callback {
public:
    Example4To3(const std::string& name, 
            const casadi::Dict& opts = casadi::Dict()) {
        this->construct(name, opts);
    }
    virtual ~Example4To3() = default;

    virtual casadi_int get_n_in() override final { return 1; }
    virtual casadi_int get_n_out() override final { return 1; }

    casadi::Sparsity get_sparsity_in(casadi_int i) override final {
        return casadi::Sparsity::dense(4, 1);
    }

    casadi::Sparsity get_sparsity_out(casadi_int i) override final {
        return casadi::Sparsity::dense(3, 1);
    }

    virtual casadi::DMVector eval(const casadi::DMVector& arg) const override {
        casadi::DMVector split = vertsplit(arg[0]);
        casadi::DM a = split[0];
        casadi::DM b = split[1];
        casadi::DM c = split[2];
        casadi::DM d = split[3];
        casadi::DM ret = vertcat(sin(c)*d + d*d, 2*a + c, b*b + 5*c);
        return {ret};
    }

    bool has_jacobian() const override { return true; }
    casadi::Function get_jacobian(const std::string& name,
            const std::vector<std::string>& inames,
            const std::vector<std::string>& onames,
            const casadi::Dict& opts) const override {
        
        casadi::MX x = casadi::MX::sym("x",4,1);
        casadi::MXVector split = vertsplit(x);
        casadi::MX a = split[0];
        casadi::MX b = split[1];
        casadi::MX c = split[2];
        casadi::MX d = split[3];

        casadi::MX f = casadi::MX::sym("f",3,1);

        casadi::MX res(3, 4);
        res(0, 2) = d*cos(c);
        res(0, 3) = sin(c) + 2*d;
        res(1, 0) = 2;
        res(1, 2) = 1;
        res(2, 1) = 2*b;
        res(2, 2) = 5;
        casadi::Function jac_f = casadi::Function(name,{x,f},{res});

        return jac_f;
    }
};


void example4to3() {

    Example4To3 f("f");
    casadi::MX x = casadi::MX::sym("x", 4);
    std::cout << "f(x): " << f(x)[0] << std::endl;

    casadi::DM y = casadi::DM::ones(4, 1);
    std::cout << "f(y): " << f(y)[0] << std::endl;

    casadi::MX dfdx = jacobian(f(x)[0], x);
    std::cout << "dfdx: " << dfdx << std::endl;

    casadi::Function J("J", {x}, {jacobian(f(x)[0], x)});
    casadi::DM a = 1.0;
    casadi::DM b = 2.0;
    casadi::DM c = 0.0;
    casadi::DM d = 3.0;
    std::cout << J(vertcat(a, b, c, d)) << std::endl;
}

int main() {
    example4to3();
    return 0;
}
