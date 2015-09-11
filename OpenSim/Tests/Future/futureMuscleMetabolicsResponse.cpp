#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

class ComplexResponse : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ComplexResponse, ModelComponent);
public:
    OpenSim_DECLARE_PROPERTY(strength, double, "per-coord param.");
    ComplexResponse() {
        constructInfrastructure();
        constructProperty_strength(3.0);
    }
    double getTerm1(const State& s) const {
        if (!isCacheVariableValid(s, "term_1")) {
            const auto& coord = getConnectee<Coordinate>("coord");
            const auto value = coord.getValue(s);
            setCacheVariableValue(s, "term_1", get_strength() * value);
            markCacheVariableValid(s, "term_1");
        }
        return getCacheVariableValue<double>(s, "term_1");
    }
    double getTerm2(const State& s) const {
        if (!isCacheVariableValid(s, "term_2")) {
            const auto& coord = getConnectee<Coordinate>("coord");
            const auto speed = coord.getSpeedValue(s);
            setCacheVariableValue(s, "term_2", 2.0 * speed);
            markCacheVariableValid(s, "term_2");
        }
        return getCacheVariableValue<double>(s, "term_2");
    }
    double getTotal(const State& s) const {
        if (!isCacheVariableValid(s, "sum")) {
            const auto& coord = getConnectee<Coordinate>("coord");
            const auto speed = coord.getSpeedValue(s);
            setCacheVariableValue(s, "sum", getTerm1(s) + getTerm2(s));
            markCacheVariableValid(s, "sum");
        }
        return getCacheVariableValue<double>(s, "sum");
    }
private:
    void constructConnectors() override {
        constructConnector<Coordinate>("coord");
    }
    void constructOutputs() override {
        constructOutput<double>("term_1",
                &ComplexResponse::getTerm1, SimTK::Stage::Position);
        constructOutput<double>("term_2",
                &ComplexResponse::getTerm2, SimTK::Stage::Velocity);
        constructOutput<double>("sum", &ComplexResponse::getTotal,
                SimTK::Stage::Time);
    }
    void extendAddToSystem(MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addCacheVariable<double>("term_1",
                0.0, SimTK::Stage::Velocity);
        addCacheVariable<double>("term_2",
                0.0, SimTK::Stage::Velocity);
        addCacheVariable<double>("sum",
                0.0, SimTK::Stage::Velocity);
    }
};

//class AggregateResponse : public ModelComponent {
//    OpenSim_DECLARE_LIST_PROPERTY(responses, ComplexResponse,
//            "for individual coordinates.");
//    OpenSim_DECLARE_PROPERTY(scaling_factor, double, "Affects each coord.");
//};

template <typename T>
class ConsoleReporter : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ConsoleReporter, Component);
public:
    ConsoleReporter() {
        constructInfrastructure();
    }
private:
    void constructInputs() override {
        constructInput<T>("input", SimTK::Stage::Acceleration);
        // multi input: constructMultiInput<T>("input", SimTK::Stage::Acceleration);
    }
    void extendRealizeReport(const State& state) const override {
        // multi input: loop through multi-inputs.
        std::cout << std::setw(10) << state.getTime() << ": " <<
            getInputValue<T>(state, "input") << std::endl;
    }
};

void integrate(const System& system, Integrator& integrator,
        const State& initialState,
        Real finalTime) {
    TimeStepper ts(system, integrator);
    ts.initialize(initialState);
    ts.setReportAllSignificantStates(true);
    integrator.setReturnEveryInternalStep(true); 
    while (ts.getState().getTime() < finalTime) {
        ts.stepTo(finalTime);
        system.realize(ts.getState(), SimTK::Stage::Report);
    }
}

void testComplexResponse() {
    Model model;
    auto b1 = new OpenSim::Body("b1", 1, Vec3(0), Inertia(0));
    auto b2 = new OpenSim::Body("b2", 1, Vec3(0), Inertia(0));
    auto b3 = new OpenSim::Body("b3", 1, Vec3(0), Inertia(0));
    auto j1 = new PinJoint("j1", model.getGround(), Vec3(0), Vec3(0),
            *b1, Vec3(0, 1, 0), Vec3(0));
    auto j2 = new PinJoint("j2", *b1, Vec3(0), Vec3(0),
               *b2, Vec3(0, 1, 0), Vec3(0));
    auto j3 = new PinJoint("j3", *b2, Vec3(0), Vec3(0),
               *b3, Vec3(0, 1, 0), Vec3(0));

    auto cr = new ComplexResponse();
    cr->setName("complex_response");
    cr->updConnector<Coordinate>("coord").set_connectee_name("j1_coord_0");

    auto reporter = new ConsoleReporter<double>();
    reporter->setName("reporter");
    reporter->getInput("input").connect(cr->getOutput("sum"));
    // TODO connect by path: reporter->getInput("input").connect("/complex_response/sum");
    // multi input: reporter->getMultiInput("input").append_connect(cr->getOutput("sum"));

    model.addBody(b1);
    model.addBody(b2);
    model.addBody(b3);
    model.addJoint(j1);
    model.addJoint(j2);
    model.addJoint(j3);
    model.addModelComponent(cr); 
    model.addModelComponent(reporter);

    State& state = model.initSystem();

    model.updCoordinateSet().get("j1_coord_0").setValue(state, 0.5 * Pi);

    RungeKuttaMersonIntegrator integrator(model.getSystem());
    integrate(model.getSystem(), integrator, state, 1);
}

int main() {
    SimTK_START_TEST("futureMuscleMetabolicsResponse");
        SimTK_SUBTEST(testComplexResponse);
    SimTK_END_TEST();
}
