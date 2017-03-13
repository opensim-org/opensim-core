
#include <OpenSim/OpenSim.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>
#include <MuscleRedundancySolver.h>

using namespace OpenSim;

/// This system only contains a a single CoordinateActuator; no muscles. This is
/// moreso a test of the inverse dynamics we perform.
void testHangingMassRoundtrip() {
    // Generate motion.
    Model model;
    model.setName("hanging_mass");
    TimeSeriesTable states;
    {
        model.set_gravity(SimTK::Vec3(9.81, 0, 0));
        auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
        model.addComponent(body);

        // Allows translation along x.
        auto* joint = new SliderJoint("joint", model.getGround(), *body);
        auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
        coord.setName("height");
        model.addComponent(joint);

        auto* actu = new CoordinateActuator("height");
        actu->setName("actuator");
        actu->set_optimal_force(10);
        model.addComponent(actu);

        auto* contr = new PrescribedController();
        contr->setName("controller");
        contr->addActuator(*actu);
        contr->prescribeControlForActuator("actuator",
                                           new Sine(-1, 2*SimTK::Pi, 0));
        model.addComponent(contr);

        auto* rep = new ConsoleReporter();
        rep->setName("reporter");
        rep->set_report_time_interval(0.1);
        rep->addToReport(coord.getOutput("value"), "height");
        rep->addToReport(actu->getOutput("actuation"), "applied_force");
        model.addComponent(rep);

        auto* statesRep = new StatesTrajectoryReporter();
        statesRep->setName("states_reporter");
        // This small interval is helpful for obtaining accurate estimates of
        // generalized accelerations, which are needed for inverse dynamics.
        statesRep->set_report_time_interval(0.001);
        model.addComponent(statesRep);

        // Simulate!
        SimTK::State state = model.initSystem();
        Manager manager(model);
        manager.integrate(state, 1.0);

        // Print the model and states trajectory to files.
        model.print("testHangingMassRoundtrip_model.osim");
        states = statesRep->getStates().exportToTable(model);
        STOFileAdapter_<double>::write(states,
                                       "testHangingMassRoundtrip_states.sto");
    }

    // Reconstruct actuation.
    {
        MuscleRedundancySolver mrs;
        mrs.setModel(model);
        mrs.setKinematicsData(states);
        MuscleRedundancySolver::Solution solution = mrs.solve();

        const auto& actual = solution.other_controls.getDependentColumn(
                "/hanging_mass/actuator");

        const auto& timeVec = solution.other_controls.getIndependentColumn();
        SimTK::Vector expected(timeVec.size(), &timeVec[0]);
        for (int i = 0; i < expected.size(); ++i) {
            expected[i] = -std::sin(2 * SimTK::Pi * expected[i]);
            //std::cout << "DEBUG " << actual[i]
            //          << " "      << expected[i] << std::endl;
        }
        SimTK_TEST_EQ_TOL(actual, expected, 0.1);
    }
}

int main() {
    SimTK_START_TEST("testHangingMassRoundtrip");
        SimTK_SUBTEST(testHangingMassRoundtrip);
    SimTK_END_TEST();
}
