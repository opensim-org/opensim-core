
#include <casadi/casadi.hpp>

#include "casadi/casadi.hpp"
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

int main() {
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


    std::cout << "DEBUG " << std::endl;
    return 0;
}
