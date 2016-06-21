
#include <OpenSim/OpenSim.h>
#include <iostream>

// Remnants from trying to use the Luxo_Myo.osim model
//#define LUXO 0
//// or: #define LUXO 1
//
//#if LUXO
//#define OPTIMAL_FORCE 2000
//#define MASS 0.5
//#define GAIN 2
//#define LOAD 10
//#define SPRINGSTIFF 5
//#define SIGNALGEN 1
//#else
//#define OPTIMAL_FORCE 4000
//#define GAIN 1
//#define LOAD 2500
//#define SPRINGSTIFF 5000
//#define SIGNALGEN 0.33
//#define REPORTING_INTERVAL 0.2
//#endif



void getAnswer() {
    std::cout << "NOTE: Answers are here to help you finish the example." 
        << std::endl;
}

namespace OpenSim {

/*************** HELPER FUNCTIONS: Implemented in answers.cpp ****************/
// Simple utility to load and draw a model in a refresh loop
// so that edits to the modelFile can be visualized immediately.
void refreshModel(const std::string& modelFile) {
    // Simulate in a replay loop if necessary
    char refresh = 'r';

    while (refresh == 'r') {
        OpenSim::Model model(modelFile);
        // make sure the model is set to visualize
        model.setUseVisualizer(true);
        // get the default state containing the default pose
        SimTK::State& state = model.initSystem();

        // Make teh visualization look nice!
        model.updMatterSubsystem().setShowDefaultGeometry(false);
        SimTK::Visualizer& viz =
            model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.GroundAndSky);

        // Draw the initial state now
        viz.drawFrameNow(state);

        // wait for user input to proceed.
        std::cout <<
            "Press 'r' to refresh model or any other key to continue."
            << std::endl;
        std::cin >> refresh;
    }
}

// Simulate any model from an initial state
void simulate(Model& model, SimTK::State& state) {
    SimTK::State s0 = state;
    // Configure the 3D visualizer environment
    if (model.getUseVisualizer()) {
        model.updMatterSubsystem().setShowDefaultGeometry(false);
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.GroundAndSky);
        viz.setShowSimTime(true);
        viz.drawFrameNow(state);
        // wait for user input to start the simulation.
        std::cout << "Press enter/return to continue:" << std::endl;
        std::cin.get();
    }

    // Simulate in a replay loop if necessary
    char replay = 'r';
    while (replay == 'r') {
        state = s0;
        SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
        OpenSim::Manager manager(model, integrator);
        manager.setInitialTime(0);
        manager.setFinalTime(5.0);
        manager.integrate(state);
        // generate states output debugging
        manager.getStateStorage().print("exampleHopperStates.sto");

        // wait for user input to proceed.
        std::cout << "Press 'r' to replay or any other key to continue." << std::endl;
        std::cin >> replay;
    }
}

// Take care of wrap objects in the model that device should also wrap over
void handlePathWrapping(ModelComponent* device, Model& model) {
    if (model.getName() != "testbed") {
        const OpenSim::Body* link1 = model.findComponent<OpenSim::Body>("link1");
        // get the knee wrap object from the model
        const OpenSim::WrapObject* wrap = link1->getWrapObject("patella");
        if (wrap) {
            auto& body = model.
                updComponent<OpenSim::Body>(link1->getFullPathName());
            auto& patella = body.upd_WrapObjectSet().get(wrap->getName());
            auto& actuator = device->
                updComponent<OpenSim::PathActuator>("cableAtoB");
            // bug in GeometryPath that needs the model to be set
            // this happens in connect
            actuator.connect(model);
            body.connect(model);
            actuator.updGeometryPath().addPathWrap(patella);
        }
    }
}

}