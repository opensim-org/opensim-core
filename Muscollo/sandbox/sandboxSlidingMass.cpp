#include <OpenSim/OpenSim.h>
#include <tropter/tropter.h>

using namespace OpenSim;

using Eigen::VectorXd;

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

class SlidingMassMinEffort : public
                             tropter::OptimalControlProblem<double> {
public:
    SlidingMassMinEffort() {
        set_time(0, {0, 5});
        for
        add_state("q", {-10, 10}, {0}, {1});
        add_state("u", {-50, 50}, {0}, {0});
        add_control("F", {-50, 50});
        m_model = createModel();
        m_state = m_model.initSystem();
    }
    // TODO rename argument "states" to "state".
    void dynamics(const VectorXd& states, const VectorXd& controls,
            Eigen::Ref<VectorXd> deriv) const override {

        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);

        auto& osimControls = m_model.updControls(m_state);
        std::copy(controls.data(), controls.data() + controls.size(),
                &osimControls[0]);

        m_model.realizeVelocity(m_state);
        m_model.setControls(m_state, osimControls);

        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.
        m_model.realizeAcceleration(m_state);
        std::copy(&m_state.updYDot()[0], &m_state.updYDot()[0] + states.size(),
                deriv.data());
    }
    //void integral_cost(const double& /*time*/, const VectorXd& /*states*/,
    //        const VectorXd& controls, double& integrand) const override {
    //    integrand = controls[0] * controls[0];
    //}
    void endpoint_cost(const double& final_time, const VectorXd&, double&
    cost)
    const override
    {
        cost = final_time;
    }

private:
    Model m_model;
    mutable SimTK::State m_state;
};

int main() {

    auto ocp = std::make_shared<SlidingMassMinEffort>();
    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal",
            "ipopt", 20);
    dircol.optimization_solver().set_hessian_approximation("limited-memory");
    auto solution = dircol.solve();
    solution.write("DEBUG_sandboxSlidingMass.csv");
    dircol.print_constraint_values(solution);

    return 0;
}

