
#include <OpenSim/OpenSim.h>
#include <iostream>
void getAnswer() {
    std::cout << "HELLO! I'm here to help you finish the example." << std::endl;
}

void editModel(const std::string& modelFile) {
    // Simulate in a replay loop if necessary
    char edit = 'e';

    while (edit == 'e') {
        OpenSim::Model model(modelFile);

        model.setUseVisualizer(true);
        SimTK::State& state = model.initSystem();
        model.updMatterSubsystem().setShowDefaultGeometry(false);
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.GroundAndSky);
        viz.setShowSimTime(true);
        viz.drawFrameNow(state);
        // wait for user input to proceed.
        std::cout << "Press enter/return to continue OR 'e' to edit model." << std::endl;
        std::cin >> edit;
    }
}