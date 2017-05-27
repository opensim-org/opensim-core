#include <OpenSim/OpenSim.h>
#include <OpenSim/Common/Reporter.h>

using namespace OpenSim;
using namespace SimTK;

class ComplexResponse : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ComplexResponse, ModelComponent);
public:
    OpenSim_DECLARE_PROPERTY(strength, double, "per-coord param.");
    OpenSim_DECLARE_SOCKET(coord, Coordinate, "");

    OpenSim_DECLARE_OUTPUT(term_1, double, getTerm1, SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(term_2, double, getTerm2, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(sum, double, getSum, SimTK::Stage::Velocity);

    ComplexResponse() {
        constructProperties();
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
    void constructProperties() {
        constructProperty_strength(3.0);
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
    AggregateResponse() { constructProperties(); }

    // TODO propagate this scaling_factor to the ComplexResponses using
    // Component::getParent.
    OpenSim_DECLARE_PROPERTY(scaling_factor, double, "Affects each coord.");
    
   

    OpenSim_DECLARE_OUTPUT(total_sum, double, getTotalSum,
            SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(total_term_1, double, getTotalTerm1,
            SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(total_term_2, double, getTotalTerm2,
            SimTK::Stage::Velocity);



    double getTotalSum(const State& s) const {
        const double basalRate(1.0);
        double totalSum = basalRate;
        for (const auto& response : getComponentList<ComplexResponse>()) {
            totalSum += response.getOutputValue<double>(s, "sum");
        }
        return totalSum;
    }

    double getTotalTerm1(const State& s) const {
        double totalTerm1=0;
        for (const auto& response : getComponentList<ComplexResponse>()) {
            totalTerm1 += response.getOutputValue<double>(s, "term_1");
        }
        return totalTerm1;
    }

    double getTotalTerm2(const State& s) const {
        double totalTerm2=0;
        for (const auto& response : getComponentList<ComplexResponse>()) {
            totalTerm2 += response.getOutputValue<double>(s, "term_2");
        }
        return totalTerm2;
    }
    
    /*
    ComplexResponse cresponse1 { constructSubcomponent<ComplexResponse>("complex_response_j1"); }
     */

private:
    
    void constructProperties() {
        constructProperty_scaling_factor(1.0);
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
    
    // Bodies and joints.
    auto b1 = new OpenSim::Body("b1", 1, Vec3(0), Inertia(0));
    auto b2 = new OpenSim::Body("b2", 1, Vec3(0), Inertia(0));
    auto b3 = new OpenSim::Body("b3", 1, Vec3(0), Inertia(0));
    auto j1 = new PinJoint("j1", model.getGround(), Vec3(0), Vec3(0),
            *b1, Vec3(0, 1, 0), Vec3(0));
    auto j2 = new PinJoint("j2", *b1, Vec3(0), Vec3(0),
               *b2, Vec3(0, 1, 0), Vec3(0));
    auto j3 = new PinJoint("j3", *b2, Vec3(0), Vec3(0),
               *b3, Vec3(0, 1, 0), Vec3(0));

    // Aggregate calculator.
    auto aggregate = new AggregateResponse();
    aggregate->setName("aggregate_response");
    // Add individual responses to the aggregate response
    auto response1 = new ComplexResponse();
    response1->setName("complex_response_j1");
    response1->updSocket<Coordinate>("coord").connect(
        j1->getCoordinate(PinJoint::Coord::RotationZ));
    // add to aggregate which takes ownership
    aggregate->addComponent(response1);

    // now create response 2
    auto response2 = new ComplexResponse();
    response2->setName("complex_response_j2");
    response2->updSocket<Coordinate>("coord").connect(
        j2->getCoordinate(PinJoint::Coord::RotationZ));
    // add to aggregate which takes ownership
    aggregate->addComponent(response2);

    // Add in the components to the model.
    model.addBody(b1);
    model.addBody(b2);
    model.addBody(b3);
    model.addJoint(j1);
    model.addJoint(j2);
    model.addJoint(j3);
    model.addModelComponent(aggregate);
    
    auto* reporter = new ConsoleReporter();
    reporter->setName("reporter");
    reporter->updInput("inputs").connect(response1->getOutput("sum"));
    reporter->updInput("inputs").connect(response2->getOutput("sum"));
    reporter->updInput("inputs").connect(aggregate->getOutput("total_sum"));
    reporter->updInput("inputs").connect(
            j1->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"));
    // TODO connect by path: reporter->getInput("input").connect("/complex_response/sum");
    // multi input: reporter->getMultiInput("input").append_connect(cr->getOutput("sum"));
 
    model.addComponent(reporter);

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
