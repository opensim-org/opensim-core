/**
 * @file ExamplePerreault2001Experiment.cpp
 *
 * \brief Control of the MoBL 2016 upper limb model.
 *
 * [1] Perreault, E. J., Kirsch, R. F., & Crago, P. E. (2001). Effects of
 * voluntary force generation on the elastic components of endpoint stiffness.
 * Experimental Brain Research, 141(3), 312--323.
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

void findExperimentConfiguration(State& state, Model& model,
        double thetaShoulderDeg, double thetaElbowDeg) {
    // add orientation osensor
    SimTK::OrientationSensors* imus = new SimTK::OrientationSensors();
    auto humerusMX = imus->addOSensor("humerus",
            model.updBodySet().get("humerus").getMobilizedBodyIndex(),
            SimTK::Rotation(), 1);

    auto radiusMX = imus->addOSensor("radius",
            model.updBodySet().get("radius").getMobilizedBodyIndex(),
            SimTK::Rotation(), 1);

    // finalize observation order (to allocate ObservationIx)
    static const int OSENSORS = 2;
    static const char* osensor_observation_order[2] = {"humerus", "radius"};
    imus->defineObservationOrder(OSENSORS, osensor_observation_order);

    // get all ObservationIx
    auto humerusOX = imus->getObservationIxForOSensor(humerusMX);
    auto radiusOX = imus->getObservationIxForOSensor(radiusMX);

    // move to initial target
    SimTK::Assembler ik(model.updMultibodySystem());
    ik.setAccuracy(1e-5);
    ik.adoptAssemblyGoal(imus);
    imus->moveOneObservation(humerusOX,
            SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                    convertDegreesToRadians(-60), SimTK::XAxis,
                    convertDegreesToRadians(thetaShoulderDeg), SimTK::YAxis,
                    convertDegreesToRadians(0), SimTK::ZAxis));
    imus->moveOneObservation(radiusOX,
            SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                    convertDegreesToRadians(-90), SimTK::XAxis,
                    convertDegreesToRadians(thetaShoulderDeg + thetaElbowDeg),
                    SimTK::YAxis, convertDegreesToRadians(0), SimTK::ZAxis));

    // setup inverse kinematics
    state.setTime(0);
    ik.initialize(state);
    ik.assemble(state);
}

void perreault2001Experiment() {

    const string example = "ExamplePerreault2001Experiment";

    const double kp = 100;
    const double kv = 20;

    ModelVisualizer::addDirToGeometrySearchPaths(OPENSIM_GEOMETRY_DIR);

    // load model
    Model model("mobl_2016_ideal_muscles.osim");

    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    // define the controller
    TaskSpaceTorqueController* controller = new TaskSpaceTorqueController();
    controller->set_ConstraintModel(AghiliModel());
    controller->set_control_strategy("velocity");
    model.addController(controller);

    auto& marker = model.getMarkerSet().get("end_effector");
    // here we just have to find the body to which the marker is
    // attached, but due to OpenSim's Frame it is more difficult now
    auto markerFrame = marker.getParentFrameName();
    auto handBodyName = markerFrame.substr(markerFrame.find("/", 1) + 1);

    // build and initialize model
    auto& state = model.initSystem();
    findExperimentConfiguration(state, model, 60, 120);
    model.realizePosition(state);

    // hand initial configuration
    auto& handBody = model.getBodySet().get(handBodyName);
    Vec3 initialOrientation =
            handBody.getRotationInGround(state).convertRotationToBodyFixedXYZ();
    Vec3 initialPosition = marker.getLocationInGround(state);

    log_debug("initial position: {}", initialPosition);

    // Set up spatial tracking for the hand
    auto hand_orientation = new OrientationTask();
    hand_orientation->set_priority(0);
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

    auto hand_position = new StationTask();
    hand_position->set_priority(0);
    hand_position->set_point(marker.get_location());
    hand_position->set_kp(Array<double>(kp, 3));
    hand_position->set_kv(Array<double>(kv, 3));
    //No acceleration error term
    hand_position->set_ka(Array<double>(0.0, 3));
    auto xpos_desired = Constant(initialPosition[0]);
    auto ypos_desired = Constant(initialPosition[1]);
    auto zpos_desired = Sine(0.1, 2 * Pi, 0, initialPosition[2]);
    //auto zpos_desired = Constant(initialPosition[2]);
    hand_position->set_position_functions(0, xpos_desired);
    hand_position->set_position_functions(1, ypos_desired);
    hand_position->set_position_functions(2, zpos_desired);
    //Don't use derivative of position function
    hand_position->set_velocity_functions(0, Constant(0));
    hand_position->set_velocity_functions(1, Constant(0));
    hand_position->set_velocity_functions(2, Constant(0));
    hand_position->set_wrt_body(handBodyName);
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(hand_position);

    

#if USE_VISUALIZER == 1
    // build and initialize model to initialize the tasks. also add a visualizer
    model.setUseVisualizer(true);
    state = model.initSystem();

    // put the model back in the correct initial configuration
    findExperimentConfiguration(state, model, 60, 120);
    model.realizePosition(state);

    for (int i = 0; i < model.getCoordinateSet().getSize(); i++) {
        if (!model.getCoordinateSet()[i].isConstrained(state))
            cout << model.getCoordinateSet()[i].getName() << " "
                 << convertRadiansToDegrees(
                            model.getCoordinateSet()[i].getValue(state))
                 << endl;
    }

    // configure visualizer
    if (model.hasVisualizer()) {
        model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(
                Vec3(0));
        model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
                Visualizer::BackgroundType::SolidColor);
        model.updMatterSubsystem().setShowDefaultGeometry(false);
    }
#endif
    IO::makeDir("results");

    // print the complete model
    model.print("results/" + example + ".osim");

    // simulate
    //simulate(model, state, 3.0, true);

    state.setTime(0.0);
    model.realizeDynamics(state);
    Manager manager(model);
    manager.setIntegratorMaximumStepSize(1e-3);
    manager.initialize(state);
    controller->computeControls(state, controller->_tau);
    double end_time = 1;
    log_debug("#####################################################");
    log_info("\nIntegrating from {} to {}", 0.0, end_time);
    state = manager.integrate(end_time);

    // export results
    controller->printResults(example, "results");
    bodyKinematics->printResults(example, "results");
}

int main(int argc, char* argv[]) {
    Logger::setLevel(Logger::Level::Info);
    try {
        perreault2001Experiment();
    } catch (exception& e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        getchar();
        return -1;
    }
    return 0;
}
