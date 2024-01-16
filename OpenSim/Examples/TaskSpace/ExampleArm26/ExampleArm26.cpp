/**
 * @file ExampleArm26.cpp
 *
 * \brief This example demonstrates multi-tasking control of a musculoskeletal
 * system. Two tasks are used in prioritized scheme and an optimization is
 * performed for mapping the task space generalized forces to muscle
 * excitations.
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

Vec3 fromVectorToVec3(const Vector& v) { return Vec3(v[0], v[1], v[2]); }

void arm26Simulation() {
    const string example = "ExampleArm26";

    ModelVisualizer::addDirToGeometrySearchPaths(OPENSIM_GEOMETRY_DIR);

    const int kp = 100;
    const int kv = 20;

    // load model
    Model model("arm26.osim");
    model.setName(example);

    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    // define the controller
    TaskSpaceTorqueController* controller = new TaskSpaceTorqueController();
    controller->set_ConstraintModel(NoConstraintModel());
    controller->set_control_strategy("force");
    model.addController(controller);

    // build and initialize model so we can retrieve its initial configuration
    auto& state = model.initSystem();
    model.realizePosition(state);

    double initial_shoulder_elevation = 0.1;
    model.updCoordinateSet()
            .get("r_shoulder_elev")
            .setValue(state, initial_shoulder_elevation);

    // humerus initial configuration
    auto& humerusBody = model.updBodySet().get("r_humerus");
    Vec3 initialOrientation_hum = humerusBody.getRotationInGround(state)
                                          .convertRotationToBodyFixedXYZ();

    // Set up orientation tracking for the humerus
    auto humerusTask = new OrientationTask();
    humerusTask->setName("r_humerus");
    humerusTask->set_priority(0);

    humerusTask->set_kp(Array<double>(kp, 3));
    humerusTask->set_kv(Array<double>(kv, 3));
    humerusTask->set_weight(Array<double>(1.0, 3));

    auto x_desired_hum = Constant(initialOrientation_hum[0]);
    auto y_desired_hum = Constant(initialOrientation_hum[1]);
    auto z_desired_hum = Constant(initialOrientation_hum[2]);

    humerusTask->set_position_functions(0, x_desired_hum);
    humerusTask->set_position_functions(1, y_desired_hum);
    humerusTask->set_position_functions(2, z_desired_hum);
    humerusTask->set_wrt_body("r_humerus");

    controller->upd_TaskSpaceTaskSet().adoptAndAppend(humerusTask);

    // ulna initial configuration
    auto& ulnaBody = model.getBodySet().get("r_ulna_radius_hand");
    Vec3 initialOrientation_uln =
            ulnaBody.getRotationInGround(state).convertRotationToBodyFixedXYZ();

    // Set up orientation tracking for the ulna
    auto ulnaTask = new OrientationTask();
    ulnaTask->setName("r_ulna_radius_hand");
    ulnaTask->set_priority(0);
    
    ulnaTask->set_kp(Array<double>(kp, 3));
    ulnaTask->set_kv(Array<double>(kv, 3));
    humerusTask->set_weight(Array<double>(1.0, 3));

    auto x_desired_uln = Constant(0.0);
    auto y_desired_uln = Constant(0.0);
    auto z_desired_uln =
            Sine(Pi / 4.0, 1 * Pi, 0, initialOrientation_uln[2] + 1);

    ulnaTask->set_position_functions(0, x_desired_uln);
    ulnaTask->set_position_functions(1, y_desired_uln);
    ulnaTask->set_position_functions(2, z_desired_uln);
    ulnaTask->set_wrt_body("r_ulna_radius_hand");
    ulnaTask->set_express_body("ground");

    controller->upd_TaskSpaceTaskSet().adoptAndAppend(ulnaTask);

    // build and initialize model to initialize the tasks. also add a visualizer
    model.setUseVisualizer(true);
    state = model.initSystem();
    model.updCoordinateSet()
            .get("r_shoulder_elev")
            .setValue(state, initial_shoulder_elevation);

    // configure visualizer
    if (model.hasVisualizer()) {
        model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(
                Vec3(0));
        model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
                Visualizer::BackgroundType::SolidColor);
        model.updMatterSubsystem().setShowDefaultGeometry(false);
    }

    // print the complete model
    model.print(example + ".osim");

    // simulate
    simulate(model, state, 4.0, true);

    // export results
    controller->printResults(example, ".");
    bodyKinematics->printResults(example, ".");
}

int main(int argc, char* argv[]) {
    try {
        arm26Simulation();
    } catch (exception& e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();
        return -1;
    }
    return 0;
}
