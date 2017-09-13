#include <OpenSim/OpenSim.h>
#include <tropter/tropter.h>

using tropter::VectorX;

namespace OpenSim {

class MucoContinuousVariableInfo : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoContinuousVariableInfo, Object);
public:
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(initial_bounds, double, 2, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(final_bounds, double, 2, "TODO");
    MucoContinuousVariableInfo() {
        constructProperties();
    }
    MucoContinuousVariableInfo(const std::string& name,
            const tropter::Bounds& bounds,
            const tropter::InitialBounds& initialBounds = {},
            const tropter::InitialBounds& finalBounds = {})
            : MucoContinuousVariableInfo() {
        setName(name);
        if (bounds.is_set()) {
            append_bounds(bounds.lower);
            if (bounds.lower != bounds.upper) {
                append_bounds(bounds.upper);
            }
        }
        if (initialBounds.is_set()) {
            append_initial_bounds(initialBounds.lower);
            if (initialBounds.lower != initialBounds.upper) {
                append_initial_bounds(initialBounds.upper);
            }
        }
        if (finalBounds.is_set()) {
            append_final_bounds(finalBounds.lower);
            if (finalBounds.lower != finalBounds.upper) {
                append_final_bounds(finalBounds.upper);
            }
        }
    }
    tropter::Bounds getBounds() const {
        tropter::Bounds bounds;
        const auto& pBounds = getProperty_bounds();
        if (pBounds.size() >= 1) {
            bounds.lower = pBounds[0];
            if (pBounds.size() == 2) bounds.upper = pBounds[1];
            else                     bounds.upper = pBounds[0];
        }
        return bounds;
    }
    tropter::InitialBounds getInitialBounds() const {
        tropter::InitialBounds initial;
        const auto& pInitial = getProperty_initial_bounds();
        if (pInitial.size() >= 1) {
            initial.lower = pInitial[0];
            if (pInitial.size() == 2) initial.upper = pInitial[1];
            else                      initial.upper = pInitial[0];
        }
        return initial;
    }
    tropter::FinalBounds getFinalBounds() const {
        tropter::FinalBounds final;
        const auto& pFinal = getProperty_final_bounds();
        if (pFinal.size() >= 1) {
            final.lower = pFinal[0];
            if (pFinal.size() == 2) final.upper = pFinal[1];
            else                    final.upper = pFinal[0];
        }
        return final;
    }
private:
    void constructProperties() {
        constructProperty_bounds();
        constructProperty_initial_bounds();
        constructProperty_final_bounds();
    }
};

class MucoCost : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(MucoCost, Object);
public:
    OpenSim_DECLARE_PROPERTY(weight, double, "TODO");
    MucoCost() {
        constructProperties();
    }
    // TODO allow alternate interface that does not require creating a SimTK
    // state.
    /// This includes the weight.
    double calcEndpointCost(const SimTK::State& finalState) const {
        return get_weight() * calcEndpointCostImpl(finalState);
    }
    // TODO avoid this weirdness.
    void setModel(const Model& model) const { m_model.reset(&model); }
protected:
    virtual double calcEndpointCostImpl(const SimTK::State& finalState)
            const = 0;
    const Model& getModel() const { return m_model.getRef(); }
private:
    void constructProperties() {
        constructProperty_weight(1);
    }
    mutable SimTK::ReferencePtr<const Model> m_model;
};

class MucoFinalTimeCost : public MucoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoFinalTimeCost, MucoCost);
protected:
    double calcEndpointCostImpl(const SimTK::State& finalState) const override {
        return finalState.getTime();
    }
};

// TODO should these be added to the model? we might need access to
// components, etc. Maybe these should have a method where they get to add
// arbitrary components to the model.
// TODO should also have an initialization routine to cache quantities.
class MucoMarkerEndpointCost : public MucoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoMarkerEndpointCost, MucoCost);
public:
    OpenSim_DECLARE_PROPERTY(frame_name, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(point_on_frame, SimTK::Vec3, "TODO");
    OpenSim_DECLARE_PROPERTY(point_to_track, SimTK::Vec3,
            "TODO Expressed in ground.");
    MucoMarkerEndpointCost() {
        constructProperties();
    }
protected:
    double calcEndpointCostImpl(const SimTK::State& finalState) const override {
        getModel().realizePosition(finalState);
        const auto& frame = getModel().getComponent<Frame>(get_frame_name());
        auto actualLocation =
                frame.findStationLocationInGround(finalState,
                        get_point_on_frame());
        return (actualLocation - get_point_to_track()).normSqr();
    }
private:
    void constructProperties() {
        constructProperty_frame_name("");
        constructProperty_point_on_frame(SimTK::Vec3(0));
        constructProperty_point_to_track(SimTK::Vec3(0));
    }
};

class MucoProblem : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoProblem, Object);
public:

    OpenSim_DECLARE_PROPERTY(model_file, std::string, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(time_initial_bounds, double, 2,
            "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(time_final_bounds, double, 2, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(state_info,
            MucoContinuousVariableInfo, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(control_info,
            MucoContinuousVariableInfo, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(costs, MucoCost, "TODO");

    MucoProblem() {
        constructProperties();
    }

    void setModel(const Model& model) { m_model = model; }
    const Model& getModel() const { return m_model; }

    void setTimeBounds(
            const tropter::InitialBounds& initial,
            const tropter::FinalBounds& final) {
        if (initial.is_set()) {
            append_time_initial_bounds(initial.lower);
            if (initial.lower != initial.upper) {
                append_time_initial_bounds(initial.upper);
            }
        }
        if (final.is_set()) {
            append_time_final_bounds(final.lower);
            if (final.lower != final.upper) {
                append_time_final_bounds(final.upper);
            }
        }
    }

    tropter::InitialBounds getTimeInitialBounds() const {
        tropter::InitialBounds initial;
        const auto& pInitial = getProperty_time_initial_bounds();
        if (pInitial.size() >= 1) {
            initial.lower = pInitial[0];
            if (pInitial.size() == 2) initial.upper = pInitial[1];
            else                      initial.upper = pInitial[0];
        }
        return initial;
    }
    tropter::FinalBounds getTimeFinalBounds() const {
        tropter::FinalBounds final;
        const auto& pFinal = getProperty_time_final_bounds();
        if (pFinal.size() >= 1) {
            final.lower = pFinal[0];
            if (pFinal.size() == 2) final.upper = pFinal[1];
            else                    final.upper = pFinal[0];
        }
        return final;
    }

    const MucoContinuousVariableInfo& getStateInfo(
            const std::string& name) const {
        int idx = getProperty_state_info().findIndexForName(name);
        OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
                "No info provided for state '" + name + "'.");
        return get_state_info(idx);
    }
    const MucoContinuousVariableInfo& getControlInfo(
            const std::string& name) const {
        int idx = getProperty_control_info().findIndexForName(name);
        OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
                "No info provided for control for '" + name + "'.");
        return get_control_info(idx);
    }
private:
    void constructProperties() {
        constructProperty_model_file("");
        constructProperty_time_initial_bounds();
        constructProperty_time_final_bounds();
        constructProperty_state_info();
        constructProperty_control_info();
        constructProperty_costs();
    }
    Model m_model;

};


class MucoSolver : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoSolver, Object);
public:
    // TODO should be a copy or reference to MucoProblem?
    MucoSolver(const MucoProblem& problem) {
        m_problem.reset(&problem);
    }

    void solve() const;

    const MucoProblem& getProblem() const { return m_problem.getRef(); }

    template<typename T>
    class OptimalControlProblem;
private:
    SimTK::ReferencePtr<const MucoProblem> m_problem;
};

/** The map provides the index of each state variable in
 SimTK::State::getY() from its each state variable path string. */
std::vector<std::string> createStateVariableNamesInSystemOrder(
        const Model& model) {
    std::vector<std::string> svNamesInSysOrder;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                svNamesInSysOrder.push_back(svNames[isv]);
                s.updY()[iy] = 0;
                break;
            }
        }
    }
    SimTK_ASSERT2_ALWAYS((size_t)svNames.size() == svNamesInSysOrder.size(),
            "Expected to get %i state names but found %i.", svNames.size(),
            svNamesInSysOrder.size());
    return svNamesInSysOrder;
}

// TODO should these be copyable to support parallelization with finite
// differences?
template<typename T>
class MucoSolver::OptimalControlProblem : public
                                          tropter::OptimalControlProblem<T> {
public:
    OptimalControlProblem(const MucoSolver& solver)
            : m_mucoSolver(solver),
              m_mucoProb(solver.getProblem()) {
        m_model = m_mucoProb.getModel();
        m_state = m_model.initSystem();

        this->set_time(m_mucoProb.getTimeInitialBounds(),
                       m_mucoProb.getTimeFinalBounds());
        auto svNamesInSysOrder = createStateVariableNamesInSystemOrder(m_model);
        for (const auto& svName : svNamesInSysOrder) {
            const auto& info = m_mucoProb.getStateInfo(svName);
            this->add_state(svName, info.getBounds(),
                    info.getInitialBounds(), info.getFinalBounds());
        }
        for (const auto& actu : m_model.getComponentList<Actuator>()) {
            // TODO handle a variable number of control signals.
            const auto& actuName = actu.getName();
            const auto& info = m_mucoProb.getControlInfo(actuName);
            this->add_control(actuName, info.getBounds(),
                    info.getInitialBounds(), info.getFinalBounds());
        }
        const auto& costs = m_mucoProb.getProperty_costs();
        // TODO avoid this weirdness; add the costs directly to the model.
        for (int ic = 0; ic < costs.size(); ++ic) {
            costs[ic].setModel(m_model);
        }
    }
    // TODO rename argument "states" to "state".
    void calc_differential_algebraic_equations(
            const tropter::DAEInput<T>& in,
            tropter::DAEOutput<T> out) const override {

        // TODO convert to implicit formulation.

        const auto& states = in.states;
        const auto& controls = in.controls;

        m_state.setTime(in.time);

        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        //
        // TODO do not copy? I think this will still make a copy:
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        auto& osimControls = m_model.updControls(m_state);
        std::copy(controls.data(), controls.data() + controls.size(),
                &osimControls[0]);

        m_model.realizeVelocity(m_state);
        m_model.setControls(m_state, osimControls);

        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.
        m_model.realizeAcceleration(m_state);
        std::copy(&m_state.getYDot()[0], &m_state.getYDot()[0] + states.size(),
                out.dynamics.data());

    }
    //void calc_integral_cost(const double& /*time*/, const VectorXd& /*states*/,
    //        const VectorXd& controls, double& integrand) const override {
    //    integrand = controls[0] * controls[0];
    //}
    void calc_endpoint_cost(const double& final_time, const VectorX<T>& states,
            double& cost) const override {
        m_state.setTime(final_time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        // TODO cannot use control signals...
        for (int i = 0; i < m_mucoProb.getProperty_costs().size(); ++i) {
            cost += m_mucoProb.get_costs(i).calcEndpointCost(m_state);
        }
    }

private:
    const MucoSolver& m_mucoSolver;
    const MucoProblem& m_mucoProb;
    Model m_model;
    mutable SimTK::State m_state;
};

void MucoSolver::solve() const {
    // TODO
    auto ocp = std::make_shared<OptimalControlProblem<double>>(*this);
    ocp->print_description();
    int N = 100;
    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt",
            N);
    dircol.get_optimization_solver().set_hessian_approximation("limited-memory");

    tropter::OptimalControlIterate guess;
    guess.time.setLinSpaced(N, 0, 1);
    const double pi = 3.14159;
    // Give a hint (not the exact final state, but something close to it).
    // I tried giving a guess where the final state guess was from the
    // solution (-3/2pi, -2pi), but then Ipopt incorrectly thought the
    // solution was all zeros.
    ocp->set_state_guess(guess, "j0/q0/value",
            Eigen::RowVectorXd::LinSpaced(N, 0, -pi));
    ocp->set_state_guess(guess, "j1/q1/value",
            Eigen::RowVectorXd::LinSpaced(N, 0, 2*pi));
    ocp->set_state_guess(guess, "j0/q0/speed", Eigen::RowVectorXd::Zero(N));
    ocp->set_state_guess(guess, "j1/q1/speed", Eigen::RowVectorXd::Zero(N));
    ocp->set_control_guess(guess, "tau0", Eigen::RowVectorXd::Zero(N));
    ocp->set_control_guess(guess, "tau1", Eigen::RowVectorXd::Zero(N));
    auto solution = dircol.solve(guess);

    //auto solution = dircol.solve();
    solution.write("DEBUG_sandboxSlidingMass.csv");
    dircol.print_constraint_values(solution);
}

} // namespace OpenSim

using namespace OpenSim;

Model createModel() {
    // Generate motion.
    Model model;
    model.setName("sliding_mass");
    model.set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 2.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model.addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model.addComponent(actu);

    return model;
}

Model createDoublePendulum() {
    Model model;
    model.setName("dp");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* b0 = new OpenSim::Body("b0", 1, Vec3(0), Inertia(1));
    model.addBody(b0);
    auto* b1  = new OpenSim::Body("b1", 1, Vec3(0), Inertia(1));
    model.addBody(b1);

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    auto* j0 = new PinJoint("j0", model.getGround(), Vec3(0), Vec3(0),
            *b0, Vec3(-1, 0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    auto* j1 = new PinJoint("j1",
            *b0, Vec3(0), Vec3(0), *b1, Vec3(-1, 0, 0), Vec3(0));
    auto& q1 = j1->updCoordinate();
    q1.setName("q1");
    model.addJoint(j0);
    model.addJoint(j1);

    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    model.addComponent(tau0);

    auto* tau1 = new CoordinateActuator();
    tau1->setCoordinate(&j1->updCoordinate());
    tau1->setName("tau1");
    tau1->setOptimalForce(1);
    model.addComponent(tau1);

    return model;
}

SimTK::Vector linspace(int length, double start, double end) {
    SimTK::Vector v(length);
    for (int i = 0; i < length; ++i) {
        v[i] = start + i * (end - start) / (length - 1);
    }
    return v;
}

int main() {

    /*
    auto ocp = std::make_shared<OptimalControlProblem>();
    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal",
            "ipopt", 20);
    dircol.get_optimization_solver().set_hessian_approximation("limited-memory");
    auto solution = dircol.solve();
    solution.write("DEBUG_sandboxSlidingMass.csv");
    dircol.print_constraint_values(solution);
     */

    /*{
        MucoProblem mp;
        mp.setModel(createModel());
        mp.setTimeBounds(0, {0, 5});
        mp.append_state_info({"joint/position/value", {-10, 10}, {0}, {1}});
        mp.append_state_info({"joint/position/speed", {-10, 10}, {0}, {0}});
        mp.append_control_info({"actuator", {-50, 50}});
        //mp.setStateInfo("joint/position/value", {-10, 10}, {0}, {1});
        //mp.setStateInfo("joint/position/speed", {-10, 10}, {0}, {0});
        //mp.setControlInfo("actuator", {-50, 50});

        mp.print("DEBUG_MucoProblem.xml");

        // TODO specify guess.

        mp.append_costs(MucoFinalTimeCost());

        MucoSolver ms(mp);
        ms.solve();
    }*/


    // TODO setting an initial guess.

    {
        MucoProblem mp;
        mp.setModel(createDoublePendulum());
        mp.setTimeBounds(0, {0, 5});
        //mp.append_state_info({"j0/q0/value", {-10, 10}, {0}});
        mp.append_state_info({"j0/q0/value", {-10, 10}, {0}});
        mp.append_state_info({"j0/q0/speed", {-50, 50}, {0}, {0}});
        mp.append_state_info({"j1/q1/value", {-10, 10}, {0}});
        mp.append_state_info({"j1/q1/speed", {-50, 50}, {0}, {0}});
        mp.append_control_info({"tau0", {-50, 50}});
        mp.append_control_info({"tau1", {-50, 50}});


        //int N = 100;
        //mp.setTimeGuess(linspace(N, 0, 1));
        //const double pi = 3.14159;
        //// Give a hint (not the exact final state, but something close to it).
        //// I tried giving a guess where the final state guess was from the
        //// solution (-3/2pi, -2pi), but then Ipopt incorrectly thought the
        //// solution was all zeros.
        //mp.setStateGuess("j0")
        //ocp->set_state_guess(guess, "j0/q0/value",
        //        Eigen::RowVectorXd::LinSpaced(N, 0, -pi));
        //ocp->set_state_guess(guess, "j1/q1/value",
        //        Eigen::RowVectorXd::LinSpaced(N, 0, 2*pi));
        //ocp->set_state_guess(guess, "j0/q0/speed", Eigen::RowVectorXd::Zero(N));
        //ocp->set_state_guess(guess, "j1/q1/speed", Eigen::RowVectorXd::Zero(N));
        //ocp->set_control_guess(guess, "tau0", Eigen::RowVectorXd::Zero(N));
        //ocp->set_control_guess(guess, "tau1", Eigen::RowVectorXd::Zero(N));

        mp.print("DEBUG_MucoProblemDoublePendulumSwingUp.xml");

        MucoMarkerEndpointCost endpointCost;
        endpointCost.set_frame_name("b1");
        endpointCost.set_weight(1000.0);
        endpointCost.set_point_on_frame(SimTK::Vec3(0));
        endpointCost.set_point_to_track(SimTK::Vec3(0, 2, 0));
        mp.append_costs(endpointCost);

        MucoFinalTimeCost ftCost;
        ftCost.set_weight(0.001);
        mp.append_costs(ftCost);

        MucoSolver ms(mp);
        ms.solve();


        // TODO get solution, use it to solve the problem on a finer grid.
    }

    return 0;
}


