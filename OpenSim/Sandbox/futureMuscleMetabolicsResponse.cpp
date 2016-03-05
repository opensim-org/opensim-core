#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

class ComplexResponse : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ComplexResponse, ModelComponent);
public:
    OpenSim_DECLARE_PROPERTY(strength, double, "per-coord param.");
    ComplexResponse() {
        constructInfrastructure();
    }
    double getTerm1(const State& s) const {
        if (!isCacheVariableValid(s, "term_1")) {
            const auto& coord = getConnectee<Coordinate>("coord");
            const auto value = coord.getValue(s);
            setCacheVariableValue(s, "term_1", get_strength() * value);
        }
        return getCacheVariableValue<double>(s, "term_1");
    }
    double getTerm2(const State& s) const {
        if (!isCacheVariableValid(s, "term_2")) {
            const auto& coord = getConnectee<Coordinate>("coord");
            const auto speed = coord.getSpeedValue(s);
            setCacheVariableValue(s, "term_2", 2.0 * speed);
        }
        return getCacheVariableValue<double>(s, "term_2");
    }
    double getSum(const State& s) const {
        if (!isCacheVariableValid(s, "sum")) {
            setCacheVariableValue(s, "sum", getTerm1(s) + getTerm2(s));
        }
        return getCacheVariableValue<double>(s, "sum");
    }
private:
    void constructProperties() override {
        constructProperty_strength(3.0);
    }
    void constructConnectors() override {
        constructConnector<Coordinate>("coord");
    }
    void constructOutputs() override {
        constructOutput<double>("term_1",
                &ComplexResponse::getTerm1, SimTK::Stage::Position);
        constructOutput<double>("term_2",
                &ComplexResponse::getTerm2, SimTK::Stage::Velocity);
        constructOutput<double>("sum", &ComplexResponse::getSum,
                SimTK::Stage::Velocity);
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

class AggregateResponse : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(AggregateResponse, ModelComponent);
public:
    AggregateResponse() { constructInfrastructure(); }

    /* TODO would prefer to use this, but waiting until the function within
     outputs is copied correctly.
     */
    OpenSim_DECLARE_LIST_PROPERTY(responses, ComplexResponse,
            "for individual coordinates.");
     
    void adopt(ComplexResponse* resp) {
        updProperty_responses().adoptAndAppendValue(resp);
        finalizeFromProperties();
    }
    
    // TODO propagate this scaling_factor to the ComplexResponses using
    // Component::getParent.
    OpenSim_DECLARE_PROPERTY(scaling_factor, double, "Affects each coord.");

    double getTotalSum(const State& s) const {
        const double basalRate(1.0);
        double totalSum = basalRate;
        for (int ir = 0; ir < getProperty_responses().size(); ++ir) {
            const auto& response = get_responses(ir);
            totalSum += response.getOutputValue<double>(s, "sum");
        }
        return totalSum;
    }

    double getTotalTerm1(const State& s) const {
        double totalTerm1=0;
        for (int ir = 0; ir < getProperty_responses().size(); ++ir) {
            const auto& response = get_responses(ir);
            totalTerm1 += response.getOutputValue<double>(s, "term_1");
        }
        return totalTerm1;
    }

    double getTotalTerm2(const State& s) const {
        double totalTerm2=0;
        for (int ir = 0; ir < getProperty_responses().size(); ++ir) {
            const auto& response = get_responses(ir);
            totalTerm2 += response.getOutputValue<double>(s, "term_2");
        }
        return totalTerm2;
    }
    
    /*
    ComplexResponse cresponse1 { constructSubcomponent<ComplexResponse>("complex_response_j1"); }
     */

private:
    /*void extendFinalizeFromProperties() {
        Super::extendFinalizeFromProperties();
        for (auto& response : responses) {
            markAsSubcomponent(response.get());
        }
    }
    */
    
    void constructProperties() override {
        constructProperty_responses();
        constructProperty_scaling_factor(1.0);
    }
    void constructOutputs() override {
        constructOutput<double>("total_sum",
                &AggregateResponse::getTotalSum, SimTK::Stage::Position);
        constructOutput<double>("total_term_1",
                &AggregateResponse::getTotalTerm1, SimTK::Stage::Velocity);
        constructOutput<double>("total_term_2",
                &AggregateResponse::getTotalTerm2, SimTK::Stage::Velocity);
    }
};

template <typename T>
class ConsoleReporter : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ConsoleReporter, Component);
public:
    ConsoleReporter() {
        constructInfrastructure();
    }
private:
    void constructProperties() override {}
    void constructInputs() override {
        constructInput<T>("input1", SimTK::Stage::Acceleration);
        // constructInput<T>("input2", SimTK::Stage::Acceleration);
        constructInput<T>("input3", SimTK::Stage::Acceleration);
        // multi input: constructMultiInput<T>("input", SimTK::Stage::Acceleration);
    }
    void extendRealizeReport(const State& state) const override {
        // multi input: loop through multi-inputs.
        // Output::getNumberOfSignificantDigits().
        std::cout << std::setw(10) << state.getTime() << ": " <<
            getInputValue<T>(state, "input1") << " " <<
            // getInputValue<T>(state, "input2") << " " <<
            getInputValue<T>(state, "input3") << std::endl;
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

    auto aggregate = new AggregateResponse();
    aggregate->setName("aggregate_response");
    // TODO must fix copying of an output's function first.
    auto* complexResponse1 = new ComplexResponse();
    complexResponse1->setName("complex_response_j1");
    complexResponse1->updConnector<Coordinate>("coord").set_connectee_name("j1_coord_0");
    aggregate->adopt(complexResponse1);
    auto* complexResponse2 = new ComplexResponse();
    complexResponse2->setName("complex_response_j2");
    complexResponse2->updConnector<Coordinate>("coord").set_connectee_name("j2_coord_0");
    aggregate->adopt(complexResponse2);
     

    auto reporter = new ConsoleReporter<double>();
    reporter->setName("reporter");
    reporter->getInput("input1").connect(aggregate->get_responses(0).getOutput("sum"));
    //reporter->getInput("input2").connect(aggregate->responses[1]->getOutput("sum"));
    reporter->getInput("input3").connect(aggregate->getOutput("total_sum"));
    // TODO connect by path: reporter->getInput("input").connect("/complex_response/sum");
    // multi input: reporter->getMultiInput("input").append_connect(cr->getOutput("sum"));

    model.addBody(b1);
    model.addBody(b2);
    model.addBody(b3);
    model.addJoint(j1);
    model.addJoint(j2);
    model.addJoint(j3);
    model.addModelComponent(aggregate); 
    model.addModelComponent(reporter);

    State& state = model.initSystem();

    model.updCoordinateSet().get("j1_coord_0").setValue(state, 0.5 * Pi);

    RungeKuttaMersonIntegrator integrator(model.getSystem());
    integrate(model.getSystem(), integrator, state, 1);
}

int main() {
    // TODO SimTK_START_TEST("futureMuscleMetabolicsResponse");
        SimTK_SUBTEST(testComplexResponse);
    //SimTK_END_TEST();
}
