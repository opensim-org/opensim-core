/**
 * @file ExampleUpperLimb.cpp
 *
 * \brief Control of the MoBL 2016 upper limb model.
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

void predictiveSimulation() {
    const string example = "ExampleUpperLimb";

    const double kp = 100;
    const double kv = 20;

    ModelVisualizer::addDirToGeometrySearchPaths(OPENSIM_GEOMETRY_DIR);

    // load model
    Model model("mobl_2016_ideal_muscles.osim");

    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    // construct a torque controller and supply the control strategy
    // define the controller
    TaskSpaceTorqueController* controller = new TaskSpaceTorqueController();
    controller->set_ConstraintModel(AghiliModel());
    controller->set_control_strategy("force");
    model.addController(controller);

    auto marker = model.getMarkerSet().get("end_effector");
    // here we just have to find the body to which the marker is
    // attached, but due to OpenSim's Frame it is more difficult now
    auto markerFrame = marker.getParentFrameName();
    auto handBodyName = markerFrame.substr(markerFrame.find("/", 1) + 1);

    // build and initialize model
    auto& state = model.initSystem();
    model.realizePosition(state);

    model.updCoordinateSet()
            .get("elv_angle")
            .setValue(state, convertDegreesToRadians(50));
    model.updCoordinateSet()
            .get("shoulder_elv")
            .setValue(state, convertDegreesToRadians(50));
    model.updCoordinateSet()
            .get("shoulder_rot")
            .setValue(state, convertDegreesToRadians(-20));
    model.updCoordinateSet()
            .get("elbow_flexion")
            .setValue(state, convertDegreesToRadians(95));
    model.updCoordinateSet().get("pro_sup").setValue(
            state, convertDegreesToRadians(75));
    model.updCoordinateSet().get("flexion").setValue(
            state, convertDegreesToRadians(0));

    // hand initial configuration
    auto& handBody = model.getBodySet().get(handBodyName);
    Vec3 initialOrientation =
            handBody.getRotationInGround(state).convertRotationToBodyFixedXYZ();
    Vec3 initialPosition = handBody.getPositionInGround(state);

    // Set up spatial tracking for the hand
    auto hand_position = new StationTask();
    hand_position->setName(handBodyName);
    hand_position->set_kp(Array<double>(kp, 3));
    hand_position->set_kv(Array<double>(kv, 3));
    auto xpos_desired = Constant(initialPosition[0]);
    // FOR REVIEW: The original code specified these positions as a function of
    // two sinusoids multiplied together, which does not seem to be possible
    // using built-in OpenSim Functions. Is this required functionality? If so,
    // is there a way to implement it as a Function?
    auto ypos_desired = Sine(0.1, Pi, 0, initialPosition[1]);
    auto zpos_desired = Sine(0.1, 2 * Pi, 0, initialPosition[2]);
    hand_position->set_position_functions(0, xpos_desired);
    hand_position->set_position_functions(1, ypos_desired);
    hand_position->set_position_functions(2, zpos_desired);
    hand_position->set_wrt_body(handBodyName);
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(hand_position);

    auto hand_orientation = new OrientationTask();
    hand_orientation->setName(handBodyName);
    hand_orientation->set_kp(Array<double>(kp, 3));
    hand_orientation->set_kv(Array<double>(kv, 3));
    auto xorient_desired = Constant(initialOrientation[0]);
    auto yorient_desired = Constant(initialOrientation[1]);
    auto zorient_desired = Constant(initialOrientation[2]);
    hand_orientation->set_position_functions(0, xorient_desired);
    hand_orientation->set_position_functions(1, yorient_desired);
    hand_orientation->set_position_functions(2, zorient_desired);
    hand_orientation->set_wrt_body(handBodyName);
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(hand_orientation);

    // build and initialize model to initialize the tasks. also add a visualizer
    model.setUseVisualizer(true);
    state = model.initSystem();

    // configure visualizer
    model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(Vec3(0));
    model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
            Visualizer::BackgroundType::SolidColor);
    model.updMatterSubsystem().setShowDefaultGeometry(false);

    // put the model back in initial position
    model.realizePosition(state);
    model.updCoordinateSet()
            .get("elv_angle")
            .setValue(state, convertDegreesToRadians(50));
    model.updCoordinateSet()
            .get("shoulder_elv")
            .setValue(state, convertDegreesToRadians(50));
    model.updCoordinateSet()
            .get("shoulder_rot")
            .setValue(state, convertDegreesToRadians(-20));
    model.updCoordinateSet()
            .get("elbow_flexion")
            .setValue(state, convertDegreesToRadians(95));
    model.updCoordinateSet().get("pro_sup").setValue(
            state, convertDegreesToRadians(75));
    model.updCoordinateSet().get("flexion").setValue(
            state, convertDegreesToRadians(0));

    // print the complete model
    model.print("results/" + example + ".osim");

    // simulate
    simulate(model, state, 2.0, true);

    // export results
    controller->printResults(example, "results");
    bodyKinematics->printResults(example, "results");
}

int main(int argc, char* argv[]) {
    try {
        predictiveSimulation();
    } catch (exception& e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();
        return -1;
    }
    return 0;
}
