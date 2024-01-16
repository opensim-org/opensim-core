/**
 * @file ExampleClosedKinematicChain.cpp
 *
 * \brief This example demonstrating working example of controlling (task
 * space) a model of a closed kinematic chain topology that is build using
 * absolute (Cartesian) coordinates. Since the underlying dynamics use
 * constraint projection, constraints are implicitly accounted. 
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

void closedKinematicChain() {
    // parameters
    string taskBodyName = "body1";
    const double q1 = -0.0;
    double body3_length = 1;
    const string example = "ExampleClosedKinematicChain";

    // create model
    Model model;
    model.setName(example);
    auto& ground = model.getGround();

    // construct body-joints
    // body1
    double body1_m = 1, body1_length = 1, body1_radius = 0.03;
    Vec3 body1_com = Vec3(0);
    Inertia body1_I =
            body1_m * Inertia::cylinderAlongY(body1_radius, body1_length);
    auto body1_body = new OpenSim::Body("body1", body1_m, body1_com, body1_I);
    auto body1_geom = new OpenSim::Cylinder(body1_radius, body1_length / 2);
    body1_geom->setName("body1_cylinder");
    body1_body->attachGeometry(body1_geom);
    Vec3 body1_distal(0, -body1_length / 2, 0);
    Vec3 body1_proximal(0, body1_length / 2, 0);
    auto ground_body1 = new OpenSim::PinJoint("ground_body1", ground, Vec3(0),
            Vec3(0), *body1_body, body1_distal, Vec3(0));
    ground_body1->upd_coordinates(0).setDefaultValue(
            SimTK::convertDegreesToRadians(q1));
    model.addBody(body1_body);
    model.addJoint(ground_body1);

    // body2
    double body2_m = 1, body2_length = 1, body2_radius = 0.03;
    Vec3 body2_com = Vec3(0);
    Inertia body2_I =
            body2_m * Inertia::cylinderAlongY(body2_radius, body2_length);
    auto body2_body = new OpenSim::Body("body2", body2_m, body2_com, body2_I);
    auto body2_geom = new OpenSim::Cylinder(body2_radius, body2_length / 2);
    body2_geom->setName("body2_cylinder");
    body2_body->attachGeometry(body2_geom);
    Vec3 body2_distal(0, -body2_length / 2, 0);
    Vec3 body2_proximal(0, body2_length / 2, 0);
    auto ground_body2 = new OpenSim::PinJoint("body1_body2", ground,
            Vec3(body3_length, 0, 0), Vec3(0), *body2_body, body2_distal,
            Vec3(0));
    ground_body2->upd_coordinates(0).setDefaultValue(
            SimTK::convertDegreesToRadians(q1));
    model.addBody(body2_body);
    model.addJoint(ground_body2);

    // body3
    double body3_m = 1, body3_radius = 0.03;
    Vec3 body3_com = Vec3(0);
    Inertia body3_I =
            body3_m * Inertia::cylinderAlongY(body3_radius, body3_length);
    auto body3_body = new OpenSim::Body("body3", body3_m, body3_com, body3_I);
    auto body3_geom = new OpenSim::Cylinder(body3_radius, body3_length / 2);
    body3_geom->setName("body3_cylinder");
    body3_body->attachGeometry(body3_geom);
    Vec3 body3_distal(0, -body3_length / 2, 0);
    Vec3 body3_proximal(0, body3_length / 2, 0);
    auto ground_body3 = new OpenSim::FreeJoint("body1_body3", ground, Vec3(0),
            Vec3(0), *body3_body, body3_distal, Vec3(0));
    model.addBody(body3_body);
    model.addJoint(ground_body3);

    // connect the two free bodies with constraints
    auto pointConstraint1 = new PointConstraint(
            *body1_body, body1_proximal, *body3_body, body3_distal);
    pointConstraint1->setName("pc1");
    model.addConstraint(pointConstraint1);
    auto pointConstraint2 = new PointConstraint(
            *body2_body, body2_proximal, *body3_body, body3_proximal);
    pointConstraint2->setName("pc2");
    model.addConstraint(pointConstraint2);

    auto& state = model.initSystem();

    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    // define the controller
    TaskSpaceTorqueController* controller = new TaskSpaceTorqueController();
    controller->set_ConstraintModel(AghiliModel());
    controller->set_control_strategy("force");
    model.addController(controller);

    // build and initialize model
    state = model.initSystem();

    // initial configuration (pose)
    model.updCoordinateSet()[0].setValue(state, convertDegreesToRadians(q1));
    model.updCoordinateSet()[1].setValue(state, convertDegreesToRadians(q1));
    auto& taskBody = model.updBodySet().get(taskBodyName);
    Vec3 initialOrientation =
            taskBody.getRotationInGround(state).convertRotationToBodyFixedXYZ();

    // Set up orientation tracking
    auto task = new OrientationTask();
    task->setName(taskBodyName);
    task->set_priority(0);

    task->set_kp(Array<double>(100, 3));
    task->set_kv(Array<double>(20, 3));
    task->set_weight(Array<double>(1, 3));
    
    task->set_direction_vectors(0, SimTK::Vec3(1, 0, 0));
    task->set_direction_vectors(1, SimTK::Vec3(0, 1, 0));
    task->set_direction_vectors(2, SimTK::Vec3(0, 0, 1));
    // task->set_active(Array<bool>(true));
    auto x_desired = Constant(initialOrientation[0]);
    auto y_desired = Constant(initialOrientation[1]);
    auto z_desired = Sine(0.9*Pi / 2, 2 * Pi, 0, initialOrientation[2]);

    task->set_position_functions(0, x_desired);
    task->set_position_functions(1, y_desired);
    task->set_position_functions(2, z_desired);

    task->set_wrt_body(taskBodyName);

    controller->upd_TaskSpaceTaskSet().adoptAndAppend(task);

    // build and initialize model to initialize the tasks. also add a visualizer
    model.setUseVisualizer(true);
    state = model.initSystem();

    // configure visualizer
    if (model.hasVisualizer()) {
        model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(
                Vec3(0));
        model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
                Visualizer::BackgroundType::SolidColor);
        model.updMatterSubsystem().setShowDefaultGeometry(true);
    }

    // simulate
    simulate(model, state, 2, true);

    // export results
    controller->printResults(example, "/results");
    bodyKinematics->printResults(example, "/results");

    model.print("ExampleClosedKinematicChain.osim");
}

int main(int argc, char* argv[]) {
    Logger::setLevel(Logger::Level::Info);

    try {
        closedKinematicChain();
    } catch (exception& e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        // getchar();
        return -1;
    }
    cout << "done" << endl;
    return 0;
}
