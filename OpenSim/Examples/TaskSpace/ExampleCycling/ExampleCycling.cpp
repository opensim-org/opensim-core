/**
 * @file ExampleCycling.cpp
 *
 * \brief Simulation of cycling using task space control of the gear rotation.
 *
  * @authors Dimitar Stanev, Nathan Pickle, Aravind Sundararajan
 *
 * @see <a href="http://ieeexplore.ieee.org/document/8074739/">[Publication]</a>
 */
#include "OpenSim/Common/osimCommon.h"
#include "OpenSim/Simulation/osimSimulation.h"
#include "OpenSim/Analyses/osimAnalyses.h"
#include "OpenSim/Tools/osimTools.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

#define USE_VISUALIZER 1

Vec3 fromVectorToVec3(const Vector& v) { return Vec3(v[0], v[1], v[2]); }

SimTK::State simulate2(Model& model, const SimTK::State& initialState,
        double finalTime, bool saveStatesFile = false) {
    // Returned/ state begins as a copy of the initial state
    SimTK::State state = initialState;
    SimTK::Visualizer::InputSilo* silo;

    bool simulateOnce = true;

    // Configure the visualizer.
    if (model.getUseVisualizer() && model.hasVisualizer()) {
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        // We use the input silo to get key presses.
        silo = &model.updVisualizer().updInputSilo();

        SimTK::DecorativeText help("Press any key to start a new simulation; "
                                   "ESC to quit.");
        help.setIsScreenText(true);
        viz.addDecoration(SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), help);

        viz.setShowSimTime(true);
        viz.drawFrameNow(state);
        std::cout << "A visualizer window has opened." << std::endl;

        // if visualizing enable replay
        simulateOnce = false;
    }

    // Simulate until the user presses ESC (or enters 'q' if visualization has
    // been disabled).
    do {
        if (model.getUseVisualizer() && model.hasVisualizer()) {
            // Get a key press.
            silo->clear(); // Ignore any previous key presses.
            unsigned key, modifiers;
            silo->waitForKeyHit(key, modifiers);
            if (key == SimTK::Visualizer::InputListener::KeyEsc) { break; }
        }

        // reset the state to the initial state
        state = initialState;
        // Set up manager and simulate.
        Manager manager(model);
        state.setTime(0.0);
        manager.setIntegratorAccuracy(
                1e-1); // model is a bit unstable due to constraints
        manager.initialize(state);
        manager.integrate(finalTime);

        // Save the states to a storage file (if requested).
        if (saveStatesFile) {
            manager.getStateStorage().print(model.getName() + "_states.sto");
        }
    } while (!simulateOnce);

    return state;
}

void cyclingSimulation() {
    const string example = "ExampleCycling";

    ModelVisualizer::addDirToGeometrySearchPaths(OPENSIM_GEOMETRY_DIR);

    // load model
    Model model("bicycle_v10.osim");
    model.set_assembly_accuracy(0.1);

    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    // define the controller
    TaskSpaceTorqueController* controller = new TaskSpaceTorqueController();
    controller->set_ConstraintModel(NoConstraintModel());
    model.addController(controller);

    // disable any actuators when computing the total force to improve
    // simulation time
    auto& ms = model.updMuscles();
    for (int i = 0; i < ms.getSize(); i++) { ms[i].set_appliesForce(false); }

    // build and initialize model
    model.finalizeFromProperties();
    auto& state = model.initSystem();

    // gears initial configuration
    auto& gearsBody = model.updBodySet().get("gears");
    Vec3 initialOrientation = gearsBody.getRotationInGround(state)
                                      .convertRotationToBodyFixedXYZ();

    Vec3 direction(1.0, 0.0, 0.0);
    double mag = 2 * Pi;

    // Set up orientation tracking for the humerus
    auto gearsTask = new OrientationTask();
    gearsTask->setName("gears_task");

    gearsTask->set_kp(Array<double>(500, 3));
    gearsTask->set_kv(Array<double>(50, 3));

    gearsTask->set_direction_vectors(0, SimTK::Vec3(1, 0, 0));
    gearsTask->set_direction_vectors(1, SimTK::Vec3(0, 1, 0));
    gearsTask->set_direction_vectors(2, SimTK::Vec3(0, 0, 1));

    auto x_desired = LinearFunction(mag * direction[0], initialOrientation[0]);
    auto y_desired = LinearFunction(mag * direction[1], initialOrientation[1]);
    auto z_desired = LinearFunction(mag * direction[2], initialOrientation[2]);

    gearsTask->set_position_functions(0, x_desired);
    gearsTask->set_position_functions(1, y_desired);
    gearsTask->set_position_functions(2, z_desired);
    gearsTask->set_wrt_body("gears");

    controller->upd_TaskSpaceTaskSet().adoptAndAppend(gearsTask);

#if USE_VISUALIZER == 1
    model.setUseVisualizer(true);
#endif
    model.initSystem();

    // configure visualizer
#if USE_VISUALIZER == 1
    if (model.hasVisualizer()) {
        model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(
                Vec3(0));
        model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
                Visualizer::BackgroundType::SolidColor);
        model.updMatterSubsystem().setShowDefaultGeometry(false);
    }
#endif

    // print the complete model
    model.print(example + ".osim");

    // simulate we have to relax the tolerance because of the constraints
    //  simulate(model, state, 2, true);
    simulate2(model, state, 4.0, true);

    // export results
    controller->printResults(example, ".");
    bodyKinematics->printResults(example, ".");
}

int main(int argc, char* argv[]) {
    try {
        cyclingSimulation();
    } catch (exception& e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        getchar();
        return -1;
    }
    return 0;
}
