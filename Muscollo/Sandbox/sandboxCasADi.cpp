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
    casadi_int get_n_in() override { return p.getNumStates() + 1; }
    casadi_int get_n_out() override { return 1;}
    void init() override {
        std::cout << "initializing object" << std::endl;
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        auto state = p.getModel().getWorkingState();
        state.setTime(time);
        for (int i = 0; i < state.getNY(); ++i) {
            state.updY()[i] = double(arg.at(i + 1));
        }
        // TODO parameters.
        return {p.calcEndpointCost(state)};
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
    casadi_int get_n_in() override { return p.getNumStates() + p.getNumControls() + 1; }
    casadi_int get_n_out() override { return 1;}
    void init() override {
        std::cout << "initializing object" << std::endl;
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        auto state = p.getModel().getWorkingState();
        state.setTime(time);
        for (int i = 0; i < state.getNY(); ++i) {
            state.updY()[i] = double(arg.at(i + 1));
        }
        auto& controls = p.getModel().updControls(state);
        for (int i = 0; i < controls.size(); ++i) {
            controls[i] = double(arg.at(i + 1 + p.getNumStates()));
        }
        return {p.calcIntegralCost(state)};
    }
private:
    const MucoProblemRep& p;
};

/*
class Defect : public Callback {
public:
    Defect(const std::string& name, const Problem<double>& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~Defect() override {}
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
*/

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
            opt.subject_to(initialBounds.getLower() <= states(is, 0) <= initialBounds.getUpper());
            // Last state can be obtained via -1.
            opt.subject_to(finalBounds.getLower() <= states(is, -1) <= finalBounds.getUpper());
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

        int ic = 0;
        for (const auto& actuator : model.getComponentList<Actuator>()) {
            const auto actuPath = actuator.getAbsolutePathString();
            const auto& info = rep.getControlInfo(actuPath);
            const auto& bounds = info.getBounds();
            const auto& initialBounds = info.getInitialBounds();
            const auto& finalBounds = info.getFinalBounds();
            opt.subject_to(bounds.getLower() <= controls(ic, Slice()) <= bounds.getUpper());
            opt.subject_to(initialBounds.getLower() <= controls(ic, 0) <= initialBounds.getUpper());
            // Last state can be obtained via -1.
            opt.subject_to(finalBounds.getLower() <= controls(ic, -1) <= finalBounds.getUpper());
            ++ic;
        }

        EndpointCost endpoint_cost_function("endpoint_cost", rep, {{"enable_fd", true}});

        std::vector<MX> args;
        args.push_back(tf);
        for (int i = 0; i < numStates; ++i) {
            args.push_back(states(i, N - 1));
        }

        auto endpoint_cost = endpoint_cost_function(args);

        IntegrandCost integrand_cost("integrand", rep, {{"enable_fd", true}});
        MX integral = opt.variable();
        integral = 0;
        for (int i = 0; i < N; ++i) {
            std::vector<MX> iargs;
            iargs.push_back(tf);
            for (int is = 0; is < numStates; ++is) {
                iargs.push_back(states(is, i));
            }
            for (int ic = 0; ic < numControls; ++ic) {
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
