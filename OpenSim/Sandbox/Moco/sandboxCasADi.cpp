#include <casadi/casadi.hpp>

// #include <tropter/tropter.h>

// #include <Moco/osimMoco.h>

// using namespace tropter;

using namespace casadi;

// using namespace OpenSim;

// class EndpointCostFunction : public Callback {
// public:
//     EndpointCostFunction(const std::string& name, const MocoProblemRep& problem,
//             const Dict& opts=Dict()) : p(problem) {
//         construct(name, opts);
//     }
//     ~EndpointCostFunction() override {}
//     casadi_int get_n_in() override { return 1 + p.getNumStates(); }
//     casadi_int get_n_out() override { return 1;}
//     void init() override {
//         std::cout << "initializing object endpointCost" << std::endl;
//     }
//     Sparsity get_sparsity_in(casadi_int i) override {
//         // TODO fix when using a matrix as input for states.
//         // TODO detect this sparsity.
//         if (i == 0) return Sparsity::scalar();
//         else return Sparsity(1, 1);
//     }

//     // Evaluate numerically
//     std::vector<DM> eval(const std::vector<DM>& arg) const override {
//         double time = double(arg.at(0));
//         auto state = p.getModel().getWorkingState();
//         state.setTime(time);
//         for (int i = 0; i < state.getNY(); ++i) {
//             state.updY()[i] = double(arg.at(1 + i));
//         }
//         // TODO parameters.
//         DM cost = p.calcEndpointCost(state);
//         // TODO std::cout << "DEBUG endpointCost " << cost.rows() << " " << cost.columns() << std::endl;
//         return {cost};
//     }
// private:
//     const MocoProblemRep& p;
// };

// class IntegrandCostFunction : public Callback {
// public:
//     IntegrandCostFunction(const std::string& name, const MocoProblemRep& problem,
//             const Dict& opts=Dict()) : p(problem) {
//         construct(name, opts);
//     }
//     ~IntegrandCostFunction() override {}
//     casadi_int get_n_in() override { return 1 + p.getNumStates() + p.getNumControls(); }
//     casadi_int get_n_out() override { return 1;}
//     void init() override {
//         std::cout << "initializing object" << std::endl;
//     }

//     Sparsity get_sparsity_out(casadi_int i) override {
//         // TODO fix when we have an actual integral cost!
//         if (i == 0) return Sparsity(1, 1);
//         else return Sparsity(0, 0);
//     }
//     // Evaluate numerically
//     std::vector<DM> eval(const std::vector<DM>& arg) const override {
//         double time = double(arg.at(0));
//         auto state = p.getModel().getWorkingState();
//         state.setTime(time);
//         for (int i = 0; i < state.getNY(); ++i) {
//             state.updY()[i] = double(arg.at(1 + i)); // (i));
//         }
//         auto& controls = p.getModel().updControls(state);
//         for (int i = 0; i < controls.size(); ++i) {
//             controls[i] = double(arg.at(1 + state.getNY() + i)); // 2)(i));
//         }
//         return {p.calcIntegralCost(state)};
//     }
// private:
//     const MocoProblemRep& p;
// };

// class DynamicsFunc : public Callback {
// public:
//     DynamicsFunc(const std::string& name, const MocoProblemRep& problem,
//             const Dict& opts=Dict()) : p(problem) {
//         construct(name, opts);
//     }
//     ~DynamicsFunc() override {}
//     casadi_int get_n_in() override { return p.getNumStates() + p.getNumControls() + 1; }
//     casadi_int get_n_out() override { return p.getNumStates();}
//     void init() override {
//         std::cout << "initializing object" << std::endl;
//     }

//     // Evaluate numerically
//     std::vector<DM> eval(const std::vector<DM>& arg) const override {
//         double time = double(arg.at(0));
//         auto state = p.getModel().getWorkingState();
//         state.setTime(time);
//         for (int i = 0; i < state.getNY(); ++i) {
//             state.updY()[i] = double(arg.at(1 + i)); // (i));
//         }
//         auto& controls = p.getModel().updControls(state);
//         for (int i = 0; i < controls.size(); ++i) {
//             controls[i] = double(arg.at(1 + state.getNY() + i)); // 2)(i));
//         }
//         p.getModel().realizeVelocity(state);
//         p.getModel().setControls(state, controls);
//         p.getModel().realizeAcceleration(state);
//         //DM deriv(state.getNY(), 1);
//         std::vector<DM> deriv(state.getNY());
//         for (int i = 0; i < state.getNY(); ++i) {
//             deriv[i] = state.getYDot()[i];
//         }
//         return deriv;
//     }
// private:
//     const MocoProblemRep& p;
// };

// class MocoCasADiSolver : public MocoSolver {
//     OpenSim_DECLARE_CONCRETE_OBJECT(MocoCasADiSolver, MocoSolver);
// public:
//     void resetProblemImpl(const MocoProblemRep&) const override {}
//     MocoSolution solve() const { return Super::solve(); }
//     MocoSolution solveImpl() const override {
//         const MocoProblemRep& rep = getProblemRep();

//         casadi::Opti opt;
//         int N = 20;

//         const auto& model = rep.getModel();

//         // Add this as a method to MocoProblemRep.
//         const auto svNamesInSysOrder =
//                 createStateVariableNamesInSystemOrder(model);
//         const int numStates = (int)svNamesInSysOrder.size();
//         // TODO SX t0
//         MX tf = opt.variable();

//         {
//             const auto& finalBounds = rep.getTimeFinalBounds();
//             opt.subject_to(finalBounds.getLower() <= tf <= finalBounds.getUpper());
//         }

//         // TODO create mesh times.
//         MX states = opt.variable(numStates, N);

//         for (int is = 0; is < numStates; ++is) {
//             const auto& info = rep.getStateInfo(svNamesInSysOrder[is]);
//             const auto& bounds = info.getBounds();
//             const auto& initialBounds = info.getInitialBounds();
//             const auto& finalBounds = info.getFinalBounds();
//             // All time except initial and final time??
//             // opt.subject_to(bounds.getLower() <= states(is, Slice(1, -1)) <= bounds.getUpper());
//             opt.subject_to(bounds.getLower() <= states(is, Slice()) <= bounds.getUpper());
//             if (initialBounds.isSet()) {
//                 opt.subject_to(initialBounds.getLower() <= states(is, 0) <= initialBounds.getUpper());
//             }
//             if (finalBounds.isSet()) {
//                 // Last state can be obtained via -1.
//                 opt.subject_to(finalBounds.getLower() <= states(is, -1) <= finalBounds.getUpper());
//             }
//         }

//         const int numControls = [&]() {
//             int count = 0;
//             for (const auto& actuator : model.getComponentList<Actuator>()) {
//                 // TODO check if it's enabled.
//                 actuator.getName();
//                 ++count;
//             }
//             return count;
//         }();

//         MX controls = opt.variable(numControls, N);

//         std::vector<std::string> controlNames;
//         int ic = 0;
//         for (const auto& actuator : model.getComponentList<Actuator>()) {
//             const auto actuPath = actuator.getAbsolutePathString();
//             controlNames.push_back(actuPath);
//             const auto& info = rep.getControlInfo(actuPath);
//             const auto& bounds = info.getBounds();
//             const auto& initialBounds = info.getInitialBounds();
//             const auto& finalBounds = info.getFinalBounds();
//             opt.subject_to(bounds.getLower() <= controls(ic, Slice()) <= bounds.getUpper());
//             if (initialBounds.isSet()) {
//                 opt.subject_to(initialBounds.getLower() <= controls(ic, 0) <= initialBounds.getUpper());
//             }
//             if (finalBounds.isSet()) {
//                 // Last state can be obtained via -1.
//                 opt.subject_to(finalBounds.getLower() <= controls(ic, -1) <= finalBounds.getUpper());
//             }
//             ++ic;
//         }

//         auto h = tf / (N - 1);
//         DynamicsFunc dynamics("dynamics", rep, {{"enable_fd", true}});

//         // Defects.

//         std::vector<MX> dynArgs;
//         dynArgs.push_back(tf);
//         for (int istate = 0; istate < numStates; ++istate) {
//             dynArgs.push_back(states(istate, 0));
//         }
//         for (int icontrol = 0; icontrol < numControls; ++icontrol) {
//             dynArgs.push_back(controls(icontrol, 0));
//         }
//         auto xdot_im1 = dynamics(dynArgs);
//         for (int itime = 1; itime < N; ++itime) {
//             const auto t = itime * h;
//             auto x_i = states(Slice(), itime);
//             auto x_im1 = states(Slice(), itime - 1);
//             // dynArgs = std::vector<MX>{t, states(Slice(), itime), controls(Slice(), itime)};
//             dynArgs.clear();
//             dynArgs.push_back(tf);
//             for (int istate = 0; istate < numStates; ++istate) {
//                 dynArgs.push_back(states(istate, itime));
//             }
//             for (int icontrol = 0; icontrol < numControls; ++icontrol) {
//                 dynArgs.push_back(controls(icontrol, itime));
//             }
//             auto xdot_i = dynamics(dynArgs);
//             //std::cout << "DEBUG dyn " << OpenSim::format("%i %i", xdot_i.rows(), xdot_i.columns()) << std::endl;
//             for (int istate = 0; istate < numStates; ++istate) {
//                 opt.subject_to(x_i(istate) == (x_im1(istate) + 0.5 * h * (xdot_i.at(istate) + xdot_im1.at(istate))));
//             }
//             xdot_im1 = xdot_i;
//         }

//         EndpointCostFunction endpoint_cost_function("endpoint_cost", rep, {{"enable_fd", true}});

//         std::vector<MX> args;
//         args.push_back(tf);
//         for (int i = 0; i < numStates; ++i) {
//             args.push_back(states(i, N-1));
//         }
//         // TODO args.push_back(states(Slice(), -1));


//         auto endpoint_cost = endpoint_cost_function(args);

//         IntegrandCostFunction integrand_cost("integrand", rep, {{"enable_fd", true}});
//         MX integral = opt.variable();
//         integral = 0;
//         for (int i = 0; i < N; ++i) {
//             std::vector<MX> args;
//             args.push_back(i * h);
//             for (int istate = 0; istate < numStates; ++istate) {
//                 args.push_back(states(istate, i));
//             }
//             for (int icontrol = 0; icontrol < numControls; ++icontrol) {
//                 args.push_back(controls(icontrol, i));
//             }
//             //const auto out = integrand_cost({i * h, states(Slice(), i), controls(Slice(), i)});
//             const auto out = integrand_cost(args);
//             integral += out.at(0);
//         }
//         opt.minimize(endpoint_cost.at(0) + integral);
//         opt.disp(std::cout, true);
//         opt.solver("ipopt", {}, {{"hessian_approximation", "limited-memory"}});
//         try {
//             auto casadiSolution = opt.solve();
//             const auto statesValue = opt.value(states);
//             SimTK::Matrix simtkStates(N, numStates);
//             for (int istate = 0; istate < numStates; ++istate) {
//                 for (int itime = 0; itime < N; ++itime) {
//                     simtkStates(itime, istate) = double(statesValue(istate, itime));
//                 }
//             }
//             const auto controlsValue = opt.value(controls);
//             SimTK::Matrix simtkControls(N, numControls);
//             for (int icontrol = 0; icontrol < numControls; ++icontrol) {
//                 for (int itime = 0; itime < N; ++itime) {
//                     simtkControls(itime, icontrol) = double(controlsValue(icontrol, itime));
//                 }
//             }

//             MocoSolution mucoSolution(
//                     OpenSim::createVectorLinspace(N, 0, double(opt.value(tf))),
//                     svNamesInSysOrder, controlNames, {}, {},
//                     simtkStates, simtkControls, {}, {});
//             return mucoSolution;


//         } catch (...) {}
//         DM statesValues = opt.debug().value(states);
//         std::cout << "DEBUG states values " << statesValues << std::endl;
//         DM controlValues = opt.debug().value(controls);
//         std::cout << "DEBUG control values " << controlValues << std::endl;

//         // opt.debug().show_infeasibilities();
//         // std::cout << "DEBUGg43 " << opt.debug().g_describe(43) << std::endl;
//         // std::cout << "DEBUGg44 " << opt.debug().g_describe(44) << std::endl;
//         // std::cout << "DEBUGg43 " << opt.g()(43) << std::endl;
//         // std::cout << "DEBUGg43 " << opt.x() << std::endl;
//         // std::cout << "DEBUGg43 " << opt.g()(44) << std::endl;

//         return {};
//     }
// private:
// };

// std::unique_ptr<Model> createSlidingMassModel() {
//     auto model = make_unique<Model>();
//     model->setName("sliding_mass");
//     model->set_gravity(SimTK::Vec3(0, 0, 0));
//     auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
//     model->addComponent(body);

//     // Allows translation along x.
//     auto* joint = new SliderJoint("slider", model->getGround(), *body);
//     auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
//     coord.setName("position");
//     model->addComponent(joint);

//     auto* actu = new CoordinateActuator();
//     actu->setCoordinate(&coord);
//     actu->setName("actuator");
//     actu->setOptimalForce(1);
//     actu->setMinControl(-10);
//     actu->setMaxControl(10);
//     model->addComponent(actu);

//     body->attachGeometry(new Sphere(0.05));

//     return model;
// }

// int main() {
//     MocoProblem problem;
//     problem.setModel(createSlidingMassModel());
//     problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0, 10));
//     problem.setStateInfo("/slider/position/value", MocoBounds(0, 1),
//             MocoInitialBounds(0), MocoFinalBounds(1));
//     problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
//     problem.addGoal<MocoFinalTimeGoal>();

//     MocoCasADiSolver solver;
//     solver.resetProblem(problem);
//     MocoSolution solution = solver.solve();
//     OpenSim::visualize(problem.getPhase().getModel(),
//             solution.exportToStatesStorage());
//     return 0;
// }

/* User-defined Jacobian in Python.

from casadi import *
class MyCallback(Callback):
    def __init__(self, name, d, opts={}):
        Callback.__init__(self)
        self.d = d
        self.construct(name, opts)
    # Number of inputs and outputs
    def get_n_in(self): return 1
    def get_n_out(self): return 1
    # Initialize the object
    def init(self):
        print('initializing object')
    # Evaluate numerically
    def eval(self, arg):
        x = arg[0]
        f = sin(self.d*x)
        return [f]
    def has_jacobian(self):
        return True
    def get_jacobian(self, name, inames, onames, opts):
        x = MX.sym(inames[0])
        J = MX.sym(onames[0])
        J = self.d * cos(self.d * x)

        dummy = MX.sym(inames[1])

        return Function(name, [x, dummy], [J], opts)


fd = MyCallback('f', 0.5, {'enable_fd': True, 'fd_options': {'h_min': 1e-3}})
f = MyCallback('f', 0.5)
ynum = fd(2)
print(fd(2))
x = MX.sym('x')



yd = fd(x)
y = f(x) # sin(0.5 * x) # f(x)

Jd = jacobian(yd, x)
J = jacobian(y, x)

Jdf = Function('Jdf', [x], [Jd])
print(Jdf(2))
Jf = Function('Jf', [x], [J])
print(Jf(2))
*/

#include <vector>
#include <iostream>

void hello_fatrop() {
    double T = 10.0; // Time horizon
    int N = 10; // Number of control intervals

    // Declare model variables
    MX x1 = MX::sym("x1");
    MX x2 = MX::sym("x2");
    MX x = vertcat(x1, x2);
    MX u = MX::sym("u");
    MX p = MX::sym("p");

    // Model equations
    MX xdot = vertcat((1-pow(x2, 2))*x1 - x2 + u + p, x1);

    // Create an integrator
    Function F = integrator("F", "rk", {{"x", x}, {"p", p}, {"u", u}, {"ode", xdot}}, {{"tf", 1.0}, {"simplify", true}, {"number_of_finite_elements", 1}});

    // Start with an empty NLP
    std::vector<MX> w;
    std::vector<double> w0;
    std::vector<double> lbw;
    std::vector<double> ubw;
    MX J = 0;
    std::vector<MX> g;
    std::vector<double> lbg;
    std::vector<double> ubg;

    // "Lift" initial conditions
    MX Xk = MX::sym("X0", 2);
    w.push_back(Xk);
    lbw.push_back(0.0);
    lbw.push_back(1.0);
    ubw.push_back(0.0);
    ubw.push_back(1.0);
    w0.push_back(0.1);
    w0.push_back(0.2);

    // Formulate the NLP
    for (int k = 0; k < N; ++k) {
        // New NLP variable for the control
        MX Uk = MX::sym("U_" + std::to_string(k));
        w.push_back(Uk);
        lbw.push_back(-1.0);
        ubw.push_back(1.0);
        w0.push_back(0.3);

        // Integrate till the end of the interval
        auto Fk = F({{"x0", Xk}, {"p", p}, {"u", Uk}});
        MX Xk_end = Fk.at("xf");
        J = J + sumsqr(Xk) + sumsqr(Uk);

        // New NLP variable for state at end of interval
        Xk = MX::sym("X_" + std::to_string(k+1), 2);
        w.push_back(Xk);
        lbw.push_back(-0.25);
        lbw.push_back(-inf);
        ubw.push_back(inf);
        ubw.push_back(inf);
        w0.push_back(0.1);
        w0.push_back(0.2);

        // Add equality constraint
        g.push_back(Xk - Xk_end);
        lbg.push_back(0.0);
        lbg.push_back(0.0);
        ubg.push_back(0.0);
        ubg.push_back(0.0);
    }

    J = J + pow(Xk(0), 2);

    // Create an NLP solver
    MX w_cat = vertcat(w);
    MX g_cat = vertcat(g);
    Function solver = nlpsol("solver", "fatrop", {{"f", J}, {"x", w_cat}, {"g", g_cat}, {"p", p}});

    // Solve the NLP
    DMDict arg = {{"x0", DM(w0)}, {"lbx", DM(lbw)}, {"ubx", DM(ubw)}, {"lbg", DM(lbg)}, {"ubg", DM(ubg)}, {"p", 0.0}};
    auto sol = solver(arg);
    std::vector<double> w_opt = sol.at("x").nonzeros();

    // Print the solution
    for (auto &val : w_opt) {
        std::cout << val << " ";
    }
    std::cout << std::endl;
}

void sliding_mass() {
    double T = 0.5; // Time horizon
    int N = 10; // Number of control intervals
    double h = T / double(N); // Length of a control interval

    // Start with an empty NLP
    std::vector<MX> w;
    std::vector<double> w0;
    std::vector<double> lbw;
    std::vector<double> ubw;
    std::vector<MX> integrand;
    std::vector<MX> quadrature;
    std::vector<MX> g;
    std::vector<double> lbg;
    std::vector<double> ubg;

    // Variables
    std::vector<MX> X;
    std::vector<MX> U;
    for (int k = 0; k < N; ++k) {

        // States
        MX Xk = MX::sym("X" + std::to_string(k), 2);
        MX Xkp1 = MX::sym("X" + std::to_string(k+1), 2);
        X.push_back(Xk);
        X.push_back(Xkp1);

        w.push_back(Xk);
        w.push_back(Xkp1);
        if (k == 0) {
            lbw.push_back(0.0);
            lbw.push_back(0.0);
            ubw.push_back(0.0);
            ubw.push_back(0.0);
            w0.push_back(0.0);
            w0.push_back(0.0);
            lbw.push_back(-5.0);
            lbw.push_back(-50.0);
            ubw.push_back(5.0);
            ubw.push_back(50.0);
            w0.push_back(0.0);
            w0.push_back(0.0);
        } else {
            lbw.push_back(-5.0);
            lbw.push_back(-50.0);
            ubw.push_back(5.0);
            ubw.push_back(50.0);
            w0.push_back(0.0);
            w0.push_back(0.0);
            lbw.push_back(-5.0);
            lbw.push_back(-50.0);
            ubw.push_back(5.0);
            ubw.push_back(50.0);
            w0.push_back(0.0);
            w0.push_back(0.0);
        }

        // Controls
        MX Uk = MX::sym("U" + std::to_string(k));
        MX Ukp1 = MX::sym("U" + std::to_string(k+1));
        U.push_back(Uk);
        U.push_back(Ukp1);
        w.push_back(Uk);
        w.push_back(Ukp1);
        lbw.push_back(-50.0);
        ubw.push_back(50.0);
        w0.push_back(0.0);
        lbw.push_back(-50.0);
        ubw.push_back(50.0);
        w0.push_back(0.0);
    }

    // Final state
    MX Xf = MX::sym("X" + std::to_string(N), 2);
    X.push_back(Xf);
    w.push_back(Xf);
    lbw.push_back(1.0);
    lbw.push_back(0.0);
    ubw.push_back(1.0);
    ubw.push_back(0.0);
    w0.push_back(0.0);
    w0.push_back(0.0);

    // 0 1
    //   2 3
    //     4 5

    // Trapezoidal transcription
    for (int k = 0; k < N; ++k) {
        MX Xk = X[2*k];
        MX Xkp1 = X[2*k+1];

        // Gap closing constraint
        g.push_back(X[2*k+2] - X[2*k+1]);
        lbg.push_back(0.0);
        lbg.push_back(0.0);
        ubg.push_back(0.0);
        ubg.push_back(0.0);

        // Model dynamics
        MX Xdotk = vertcat(X[2*k](1), U[2*k]);
        MX Xdotkp1 = vertcat(X[2*k+1](1), U[2*k+1]);

        // Defect constraint
        g.push_back(X[2*k+1] - (X[2*k] + 0.5 * h * (Xdotk + Xdotkp1)));
        lbg.push_back(0.0);
        lbg.push_back(0.0);
        ubg.push_back(0.0);
        ubg.push_back(0.0);
    }

    // Integrate the cost
    MX J = 0;
    // for (int i = 0; i < (int)integrand.size(); ++i) {
    //     J += quadrature[i] * integrand[i];
    // }
    // J *= T;

    // Create an NLP solver
    MX w_cat = vertcat(w);
    MX g_cat = vertcat(g);
    casadi::Dict options;
    // options["structure_detection"] = "auto";
    // std::vector<bool> equality;
    // for (int i = 0; i < 4*N; ++i) {
    //     equality.push_back(true);
    // }
    // options["equality"] = equality;

    options["structure_detection"] = "manual";
    options["N"] = N;

    std::vector<int> nx;
    std::vector<int> nu;
    std::vector<int> ng;
    nx.reserve(N+1);
    nu.reserve(N+1);
    ng.reserve(N+1);
    for (int i = 0; i < N; ++i) {
        nx.push_back(4);
        nu.push_back(2);
        ng.push_back(0);
    }
    nx.push_back(2);
    nu.push_back(0);
    ng.push_back(0);
    options["nx"] = nx;
    options["nu"] = nu;
    options["ng"] = ng;

    options["debug"] = true;
    auto jacobian = casadi::MX::jacobian(g_cat, w_cat);
    jacobian.sparsity().to_file("sliding_mass_constraint_Jacobian_sparsity.mtx");
    Function solver = nlpsol("solver", "fatrop", {{"f", J}, {"x", w_cat}, {"g", g_cat}}, options);

    // Solve the NLP
    DMDict arg = {{"x0", DM(w0)}, {"lbx", DM(lbw)}, {"ubx", DM(ubw)}, {"lbg", DM(lbg)}, {"ubg", DM(ubg)}};
    auto sol = solver(arg);
    std::vector<double> w_opt = sol.at("x").nonzeros();
}

void dircol() {
    // Degree of interpolating polynomial
    int d = 3;

    casadi::DM C;
    casadi::DM D;
    casadi::DM B;

    // Get collocation points
    std::vector<double> tau_root = casadi::collocation_points(d, "legendre");
    casadi::collocation_coeff(tau_root, C, D, B);
    std::cout << "C: " << C << std::endl;
    std::cout << "D: " << D << std::endl;
    std::cout << "B: " << B << std::endl;

    // Time horizon
    double T = 10.0;

    // Control discretization
    int N = 20; // number of control intervals
    double h = T / N;

    // Start with an empty NLP
    std::vector<MX> w;
    std::vector<double> w0;
    std::vector<double> lbw;
    std::vector<double> ubw;
    std::vector<MX> g;
    std::vector<double> lbg;
    std::vector<double> ubg;
    MX J = 0;

    // Slider starts at rest at x1 = 0
    MX Xk = MX::sym("X0", 2);
    w.push_back(Xk);
    lbw.push_back(0.0);
    lbw.push_back(0.0);
    ubw.push_back(0.0);
    ubw.push_back(0.0);
    w0.push_back(0.0);
    w0.push_back(0.0);

    for (int k = 0; k < N; ++k) {

        // New NLP variable for the control
        MX Uk = MX::sym("U_" + std::to_string(k));
        w.push_back(Uk);
        lbw.push_back(-50.0);
        ubw.push_back(50.0);
        w0.push_back(0.0);

        std::vector<MX> Z;
        Z.push_back(Xk);
        std::vector<MX> Xc;
        for (int j = 0; j < d; ++j) {
            MX Xkj = MX::sym("C_" + std::to_string(k) + "_" + std::to_string(j), 2);
            Z.push_back(Xkj);
            Xc.push_back(Xkj);
            w.push_back(Xkj);
            lbw.push_back(-5);
            lbw.push_back(-50);
            ubw.push_back(5);
            ubw.push_back(50);
            w0.push_back(0.0);
            w0.push_back(0.0);
        }

        // Defects and cost
        MX Pidot = MX::mtimes(horzcat(Z), C);
        std::vector<MX> cost;
        for (int j = 0; j < d; ++j) {
            MX Xdot = vertcat(Xc[j](1), Uk);
            g.push_back(h * Xdot - Pidot(j));
            lbg.push_back(0.0);
            lbg.push_back(0.0);
            ubg.push_back(0.0);
            ubg.push_back(0.0);
            cost.push_back(pow(Uk, 2));
        }

        // New NLP variables end of interval
        MX Xkp1 = MX::sym("X_" + std::to_string(k+1), 2);
        w.push_back(Xkp1);
        if (k == N - 1) {
            // Slider ends at rest at x1 = 1
            lbw.push_back(1.0);
            lbw.push_back(0.0);
            ubw.push_back(1.0);
            ubw.push_back(0.0);
            w0.push_back(0.0);
            w0.push_back(0.0);
        } else {
            lbw.push_back(-5.0);
            lbw.push_back(-50.0);
            ubw.push_back(5.0);
            ubw.push_back(50.0);
            w0.push_back(0.0);
            w0.push_back(0.0);
        }

        MX Xk_end = MX::mtimes(horzcat(Z), D);

        // Continuity constraints
        g.push_back(Xkp1 - Xk_end);
        lbg.push_back(0.0);
        lbg.push_back(0.0);
        ubg.push_back(0.0);
        ubg.push_back(0.0);

        // Integral cost
        J += h*mtimes(horzcat(cost), B);

        // Update variables for next interval
        Xk = Xkp1;
    }

    std::cout << "w.size(): " << w.size() << std::endl;
    std::cout << "g.size(): " << g.size() << std::endl;

    // Concatenate vectors
    MX w_conc = vertcat(w);
    MX g_conc = vertcat(g);
    std::vector<double> w0_conc(w0);
    std::vector<double> lbw_conc(lbw);
    std::vector<double> ubw_conc(ubw);
    std::vector<double> lbg_conc(lbg);
    std::vector<double> ubg_conc(ubg);

    std::cout << "w_conc.size(): " << w_conc.size() << std::endl;
    std::cout << "g_conc.size(): " << g_conc.size() << std::endl;
    std::cout << "w0_conc.size(): " << w0_conc.size() << std::endl;
    std::cout << "lbw_conc.size(): " << lbw_conc.size() << std::endl;
    std::cout << "ubw_conc.size(): " << ubw_conc.size() << std::endl;
    std::cout << "lbg_conc.size(): " << lbg_conc.size() << std::endl;
    std::cout << "ubg_conc.size(): " << ubg_conc.size() << std::endl;

    // Create an NLP solver
    Dict opts;
    // opts["structure_detection"] = "auto";
    // std::vector<bool> equality;
    // for (int i = 0; i < 8*N; ++i) {
    //     equality.push_back(true);
    // }
    // opts["equality"] = equality;

    opts["structure_detection"] = "manual";
    opts["N"] = N;
    std::vector<int> nx;
    std::vector<int> nu;
    std::vector<int> ng;
    for (int i = 0; i < N+1; ++i) {
        nx.push_back(2*d +1);
        nu.push_back(1);
        ng.push_back(0);
    }
    opts["nx"] = nx;
    opts["nu"] = nu;
    opts["ng"] = ng;
    opts["debug"] = true;
    Function solver = nlpsol("solver", "fatrop", {{"f", J}, {"x", w_conc}, {"g", g_conc}}, opts);

    // Solve the NLP
    DMDict arg = {{"x0", w0_conc}, {"lbx", lbw_conc}, {"ubx", ubw_conc}, {"lbg", lbg_conc}, {"ubg", ubg_conc}};
    DMDict sol = solver(arg);

}


int main() {
    // hello_fatrop();
    sliding_mass();
    // dircol();

    return 0;
}