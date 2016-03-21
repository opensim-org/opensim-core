#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

StatesTrajectory simulate(const Model& model, const State& initState,
        double finalTime) {

    StatesTrajectory states;

    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    SimTK::TimeStepper ts(model.getSystem(), integrator);
    ts.initialize(initState);
    ts.setReportAllSignificantStates(true);
    integrator.setReturnEveryInternalStep(true);
    while (ts.getState().getTime() < finalTime) {
        ts.stepTo(finalTime);
        // StatesTrajectory API for appending states:
        states.append(ts.getState());
    }
    return states;

}

int main() {
    Model model("gait10dof18musc_subject01.osim");

    // Create a StatesTrajectory from a forward simulation.
    // ----------------------------------------------------
    // To assist with creating interesting (non-zero) coordinate values:
    model.updCoordinateSet().get("pelvis_ty").setDefaultLocked(true);
    auto& initState = model.initSystem();
    StatesTrajectory states = simulate(model, initState, 0.05);

    // Use trajectory.
    // ---------------
    for (const auto& state : states) {
        std::cout << state.getTime() << "s: "
                  << model.getStateVariableValue(state,
                                                 "ankle_r/ankle_angle_r/value")
                  << std::endl;
    }

    // Export to data table.
    // ---------------------
    auto table = states.exportToTable(model);

    return 0;
}
