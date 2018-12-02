#include <casadi/casadi.hpp>

#include <tropter/tropter.h>

#include <Muscollo/osimMuscollo.h>

using namespace tropter;

using namespace casadi;

using namespace OpenSim;

class EndpointCost : public Callback {
public:
    EndpointCost(const std::string& name, const MucoProblemRep& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~EndpointCost() override {}
    casadi_int get_n_in() override { return 1 + p.getNumStates(); }
    casadi_int get_n_out() override { return 1;}
    void init() override {
        std::cout << "initializing object endpointCost" << std::endl;
    }
    Sparsity get_sparsity_in(casadi_int i) override {
        // TODO fix when using a matrix as input for states.
        // TODO detect this sparsity.
        if (i == 0) return Sparsity::scalar();
        else return Sparsity(1, 1);
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        auto state = p.getModel().getWorkingState();
        state.setTime(time);
        for (int i = 0; i < state.getNY(); ++i) {
            state.updY()[i] = double(arg.at(1 + i));
        }
        // TODO parameters.
        DM cost = p.calcEndpointCost(state);
        // TODO std::cout << "DEBUG endpointCost " << cost.rows() << " " << cost.columns() << std::endl;
        return {cost};
    }
private:
    const MucoProblemRep& p;
};

class IntegrandCost : public Callback {
public:
    IntegrandCost(const std::string& name, const MucoProblemRep& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~IntegrandCost() override {}
    casadi_int get_n_in() override { return 1 + p.getNumStates() + p.getNumControls(); }
    casadi_int get_n_out() override { return 1;}
    void init() override {
        std::cout << "initializing object" << std::endl;
    }

    Sparsity get_sparsity_out(casadi_int i) override {
        // TODO fix when we have an actual integral cost!
        if (i == 0) return Sparsity(1, 1);
        else return Sparsity(0, 0);
    }
    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        auto state = p.getModel().getWorkingState();
        state.setTime(time);
        for (int i = 0; i < state.getNY(); ++i) {
            state.updY()[i] = double(arg.at(1 + i)); // (i));
        }
        auto& controls = p.getModel().updControls(state);
        for (int i = 0; i < controls.size(); ++i) {
            controls[i] = double(arg.at(1 + state.getNY() + i)); // 2)(i));
        }
        return {p.calcIntegralCost(state)};
    }
private:
    const MucoProblemRep& p;
};

class DynamicsFunc : public Callback {
public:
    DynamicsFunc(const std::string& name, const MucoProblemRep& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~DynamicsFunc() override {}
    casadi_int get_n_in() override { return p.getNumStates() + p.getNumControls() + 1; }
    casadi_int get_n_out() override { return p.getNumStates();}
    void init() override {
        std::cout << "initializing object" << std::endl;
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        auto state = p.getModel().getWorkingState();
        state.setTime(time);
        for (int i = 0; i < state.getNY(); ++i) {
            state.updY()[i] = double(arg.at(1 + i)); // (i));
        }
        auto& controls = p.getModel().updControls(state);
        for (int i = 0; i < controls.size(); ++i) {
            controls[i] = double(arg.at(1 + state.getNY() + i)); // 2)(i));
        }
        p.getModel().realizeVelocity(state);
        p.getModel().setControls(state, controls);
        p.getModel().realizeAcceleration(state);
        //DM deriv(state.getNY(), 1);
        std::vector<DM> deriv(state.getNY());
        for (int i = 0; i < state.getNY(); ++i) {
            deriv[i] = state.getYDot()[i];
        }
        return deriv;
    }
private:
    const MucoProblemRep& p;
};

class MucoCasADiSolver : public MucoSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoCasADiSolver, MucoSolver);
public:
    void resetProblemImpl(const MucoProblemRep&) const override {}
    MucoSolution solve() const { return Super::solve(); }
    MucoSolution solveImpl() const override {
        const MucoProblemRep& rep = getProblemRep();

        casadi::Opti opt;
        int N = 20;

        const auto& model = rep.getModel();

        // Add this as a method to MucoProblemRep.
        const auto svNamesInSysOrder =
                createStateVariableNamesInSystemOrder(model);
        const int numStates = (int)svNamesInSysOrder.size();
        // TODO SX t0
        MX tf = opt.variable();

        {
            const auto& finalBounds = rep.getTimeFinalBounds();
            opt.subject_to(finalBounds.getLower() <= tf <= finalBounds.getUpper());
        }

        // TODO create mesh times.
        MX states = opt.variable(numStates, N);

        for (int is = 0; is < numStates; ++is) {
            const auto& info = rep.getStateInfo(svNamesInSysOrder[is]);
            const auto& bounds = info.getBounds();
            const auto& initialBounds = info.getInitialBounds();
            const auto& finalBounds = info.getFinalBounds();
            // All time except initial and final time??
            // opt.subject_to(bounds.getLower() <= states(is, Slice(1, -1)) <= bounds.getUpper());
            opt.subject_to(bounds.getLower() <= states(is, Slice()) <= bounds.getUpper());
            if (initialBounds.isSet()) {
                opt.subject_to(initialBounds.getLower() <= states(is, 0) <= initialBounds.getUpper());
            }
            if (finalBounds.isSet()) {
                // Last state can be obtained via -1.
                opt.subject_to(finalBounds.getLower() <= states(is, -1) <= finalBounds.getUpper());
            }
        }

        const int numControls = [&]() {
            int count = 0;
            for (const auto& actuator : model.getComponentList<Actuator>()) {
                // TODO check if it's enabled.
                actuator.getName();
                ++count;
            }
            return count;
        }();

        MX controls = opt.variable(numControls, N);

        std::vector<std::string> controlNames;
        int ic = 0;
        for (const auto& actuator : model.getComponentList<Actuator>()) {
            const auto actuPath = actuator.getAbsolutePathString();
            controlNames.push_back(actuPath);
            const auto& info = rep.getControlInfo(actuPath);
            const auto& bounds = info.getBounds();
            const auto& initialBounds = info.getInitialBounds();
            const auto& finalBounds = info.getFinalBounds();
            opt.subject_to(bounds.getLower() <= controls(ic, Slice()) <= bounds.getUpper());
            if (initialBounds.isSet()) {
                opt.subject_to(initialBounds.getLower() <= controls(ic, 0) <= initialBounds.getUpper());
            }
            if (finalBounds.isSet()) {
                // Last state can be obtained via -1.
                opt.subject_to(finalBounds.getLower() <= controls(ic, -1) <= finalBounds.getUpper());
            }
            ++ic;
        }

        auto h = tf / (N - 1);
        DynamicsFunc dynamics("dynamics", rep, {{"enable_fd", true}});

        // Defects.

        std::vector<MX> dynArgs;
        dynArgs.push_back(tf);
        for (int istate = 0; istate < numStates; ++istate) {
            dynArgs.push_back(states(istate, 0));
        }
        for (int icontrol = 0; icontrol < numControls; ++icontrol) {
            dynArgs.push_back(controls(icontrol, 0));
        }
        auto xdot_im1 = dynamics(dynArgs);
        for (int itime = 1; itime < N; ++itime) {
            const auto t = itime * h;
            auto x_i = states(Slice(), itime);
            auto x_im1 = states(Slice(), itime - 1);
            // dynArgs = std::vector<MX>{t, states(Slice(), itime), controls(Slice(), itime)};
            dynArgs.clear();
            dynArgs.push_back(tf);
            for (int istate = 0; istate < numStates; ++istate) {
                dynArgs.push_back(states(istate, itime));
            }
            for (int icontrol = 0; icontrol < numControls; ++icontrol) {
                dynArgs.push_back(controls(icontrol, itime));
            }
            auto xdot_i = dynamics(dynArgs);
            //std::cout << "DEBUG dyn " << OpenSim::format("%i %i", xdot_i.rows(), xdot_i.columns()) << std::endl;
            for (int istate = 0; istate < numStates; ++istate) {
                opt.subject_to(x_i(istate) == (x_im1(istate) + 0.5 * h * (xdot_i.at(istate) + xdot_im1.at(istate))));
            }
            xdot_im1 = xdot_i;
        }

        EndpointCost endpoint_cost_function("endpoint_cost", rep, {{"enable_fd", true}});

        std::vector<MX> args;
        args.push_back(tf);
        for (int i = 0; i < numStates; ++i) {
            args.push_back(states(i, N-1));
        }
        // TODO args.push_back(states(Slice(), -1));


        auto endpoint_cost = endpoint_cost_function(args);

        IntegrandCost integrand_cost("integrand", rep, {{"enable_fd", true}});
        MX integral = opt.variable();
        integral = 0;
        for (int i = 0; i < N; ++i) {
            std::vector<MX> args;
            args.push_back(i * h);
            for (int istate = 0; istate < numStates; ++istate) {
                args.push_back(states(istate, i));
            }
            for (int icontrol = 0; icontrol < numControls; ++icontrol) {
                args.push_back(controls(icontrol, i));
            }
            //const auto out = integrand_cost({i * h, states(Slice(), i), controls(Slice(), i)});
            const auto out = integrand_cost(args);
            integral += out.at(0);
        }
        opt.minimize(endpoint_cost.at(0) + integral);
        opt.disp(std::cout, true);
        opt.
        opt.solver("ipopt", {}, {{"hessian_approximation", "limited-memory"}});
        try {
            auto casadiSolution = opt.solve();
            const auto statesValue = opt.value(states);
            SimTK::Matrix simtkStates(N, numStates);
            for (int istate = 0; istate < numStates; ++istate) {
                for (int itime = 0; itime < N; ++itime) {
                    simtkStates(itime, istate) = double(statesValue(istate, itime));
                }
            }
            const auto controlsValue = opt.value(controls);
            SimTK::Matrix simtkControls(N, numControls);
            for (int icontrol = 0; icontrol < numControls; ++icontrol) {
                for (int itime = 0; itime < N; ++itime) {
                    simtkControls(itime, icontrol) = double(controlsValue(icontrol, itime));
                }
            }

            MucoSolution mucoSolution(
                    OpenSim::createVectorLinspace(N, 0, double(opt.value(tf))),
                    svNamesInSysOrder, controlNames, {}, {},
                    simtkStates, simtkControls, {}, {});
            return mucoSolution;


        } catch (...) {}
        DM statesValues = opt.debug().value(states);
        std::cout << "DEBUG states values " << statesValues << std::endl;
        DM controlValues = opt.debug().value(controls);
        std::cout << "DEBUG control values " << controlValues << std::endl;

        // opt.debug().show_infeasibilities();
        // std::cout << "DEBUGg43 " << opt.debug().g_describe(43) << std::endl;
        // std::cout << "DEBUGg44 " << opt.debug().g_describe(44) << std::endl;
        // std::cout << "DEBUGg43 " << opt.g()(43) << std::endl;
        // std::cout << "DEBUGg43 " << opt.x() << std::endl;
        // std::cout << "DEBUGg43 " << opt.g()(44) << std::endl;

        return {};
    }
private:
};

std::unique_ptr<Model> createSlidingMassModel() {
    auto model = make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    actu->setMinControl(-10);
    actu->setMaxControl(10);
    model->addComponent(actu);

    body->attachGeometry(new Sphere(0.05));

    return model;
}

int main() {
    MucoProblem problem;
    problem.setModel(createSlidingMassModel());
    problem.setTimeBounds(MucoInitialBounds(0), MucoFinalBounds(0, 10));
    problem.setStateInfo("/slider/position/value", MucoBounds(0, 1),
            MucoInitialBounds(0), MucoFinalBounds(1));
    problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
    problem.addCost<MucoFinalTimeCost>();

    MucoCasADiSolver solver;
    solver.resetProblem(problem);
    MucoSolution solution = solver.solve();
    OpenSim::visualize(problem.getPhase().getModel(),
            solution.exportToStatesStorage());

    std::cout << "DEBUG " << std::endl;
    return 0;
}
