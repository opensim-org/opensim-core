#include <iostream>
#include <OpenSim/OpenSim.h>

void getAnswer();

using namespace SimTK;
void integrate(const System& system, Integrator& integrator,
        const State& initialState,
        SimTK::Real finalTime) {
    SimTK::TimeStepper ts(system, integrator);
    ts.initialize(initialState);
    ts.setReportAllSignificantStates(true);
    integrator.setReturnEveryInternalStep(true); 
    while (ts.getState().getTime() < finalTime) {
        ts.stepTo(finalTime);
        system.realize(ts.getState(), SimTK::Stage::Report);
    }
}

int main() {
    OpenSim::Model luxo("Luxo_Myo.osim");
    std::cout << "Loaded model." << std::endl;
    luxo.setUseVisualizer(true);
    

    SimTK::State& s = luxo.initSystem();
        
    // Configure the 3D visualizer environment
    luxo.updMatterSubsystem().setShowDefaultGeometry(false);
    Visualizer& viz = luxo.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.GroundAndSky);
    viz.setShowSimTime(true);
    
    // TODO replace with a driver / time step advancer.
    SimTK::RungeKuttaMersonIntegrator integrator(luxo.getSystem());
    integrate(luxo.getSystem(), integrator, s, 5.0);

    getAnswer();

    return 0;
}
