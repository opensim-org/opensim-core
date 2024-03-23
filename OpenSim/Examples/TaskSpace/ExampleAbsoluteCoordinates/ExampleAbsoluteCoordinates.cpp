/**
 * @file ExampleAbsoluteCoordinates.cpp
 *
 * \brief This example demonstrating working example of controlling (task space)
 * a model that is build using absolute (Cartesian) coordinates. Since the
 * underlying dynamics use constraint projection, constraints are implicitly
 * accounted. 
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

Model constructModel(const std::string& name) 
{
    // model
    Model model;
    model.setName(name);

    // construct model
    auto ground = &model.updGround();
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
    auto ground_body1 = new OpenSim::FreeJoint("ground_body1", *ground, Vec3(0),
            Vec3(0), *body1_body, body1_distal, Vec3(0));
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
    auto body1_body2 = new OpenSim::FreeJoint("body1_body2", *body1_body,
            body1_proximal, Vec3(0), *body2_body, body2_distal, Vec3(0));
    model.addBody(body2_body);
    model.addJoint(body1_body2);

    // connect the two free bodies
    auto pointConstraint1 =
            new PointConstraint(*ground, Vec3(0), *body1_body, body1_distal);
    pointConstraint1->setName("pc1");
    model.addConstraint(pointConstraint1);
    auto pointConstraint2 = new PointConstraint(
            *body1_body, body2_proximal, *body2_body, body2_distal);
    pointConstraint2->setName("pc2");
    model.addConstraint(pointConstraint2);

    return model;
}

void setInitialConfiguration(Model& model, const double& q1, const double& q2) {
    auto* ground_body1 = FreeJoint::safeDownCast(&model.updJointSet().get("ground_body1"));
    auto* body1_body2 = FreeJoint::safeDownCast(&model.updJointSet().get("body1_body2"));

    auto& state = model.updWorkingState();

    // initial configuration (pose)
    ground_body1->upd_coordinates(2).setValue(
            state, convertDegreesToRadians(q1));
    body1_body2->upd_coordinates(2).setValue(
            state, convertDegreesToRadians(q2));
}

void absoluteCoordinates() {
    // parameters
    const double q1 = -45.0;
    const double q2 = 90;
    const string taskBodyName = "body2";
    const string example = "ExampleAbsoluteCoordinates";

    auto model = constructModel(example);

    // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    // define the controller
    TaskSpaceTorqueController* controller = new TaskSpaceTorqueController();
    controller->set_ConstraintModel(DeSapioModel());
    controller->set_control_strategy("force");
    // model takes ownership
    model.addController(controller);

    // build and initialize model
    model.setUseVisualizer(false);
    model.initSystem();

    setInitialConfiguration(model, q1, q2);
    
    auto& taskBody = model.updBodySet().get(taskBodyName);
    auto point = SimTK::Vec3(0, 0.5, 0);
    auto& state = model.updWorkingState();
    Vec3 initialPosition = taskBody.findStationLocationInGround(state, point);

    // Set up position tracking for the block
    auto task = new StationTask();
    task->setName(taskBodyName+"task");
    task->set_priority(0);
    task->set_point(point);

    task->set_kp(Array<double>(100, 3));
    task->set_kv(Array<double>(20, 3));
    task->set_weight(Array<double>(1,3));
    
    auto x_desired = Constant(initialPosition[0]);
    auto y_desired = Sine(0.2, 2 * Pi, 0, initialPosition[1]);
    auto z_desired = Constant(initialPosition[2]);
    
    task->set_position_functions(0, x_desired);
    task->set_position_functions(1, y_desired);
    task->set_position_functions(2, z_desired);
    task->set_wrt_body(taskBodyName);

    controller->upd_TaskSpaceTaskSet().adoptAndAppend(task);

    // reinitialize to set up the task
    model.setUseVisualizer(true);
    state = model.initSystem();

    //reapply the initial configuration
    setInitialConfiguration(model, q1, q2);

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
    IO::makeDir("results");
    controller->printResults(example, IO::getCwd()+"/results");
    bodyKinematics->printResults(example, IO::getCwd()+"/results");

    model.print(IO::getCwd()+"/results/"+example+".osim");
}

int main(int argc, char* argv[]) {
    Logger::setLevel(Logger::Level::Info);
    try {
        absoluteCoordinates();
    } catch (exception& e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        return -1;
    }
    cout << "Example absolute coordinates finished." << endl;
    return 0;
}
