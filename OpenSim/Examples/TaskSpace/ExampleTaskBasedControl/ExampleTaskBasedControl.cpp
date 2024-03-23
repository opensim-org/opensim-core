/**
 * @file ExampleTaskBasedControl.cpp
 *
 * \brief An example of utilizing the task based projection in order to control
 * a model in task space. In this example a block is created and a position task
 * is assigned. The goal is prescribed using a PD tracking controller and a
 * forward simulation is performed.
 *
 * @authors Dimitar Stanev, Nathan Pickle, Aravind Sundararajan
 *
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
    // create model
    Model model;
    model.setName(name);
    model.setUseVisualizer(true);

    Vec3 halfLength(0.2, 0.2, 0.2);
    auto block =
            new OpenSim::Body("block", 1, Vec3(0), Inertia::brick(halfLength));
    auto blockGeom = new OpenSim::Brick(halfLength);
    block->attachGeometry(blockGeom);
    model.addBody(block);
    auto joint = new FreeJoint("joint", model.getGround(), Vec3(0), Vec3(0),
            *block, Vec3(0), Vec3(0));
    model.addJoint(joint);

    // add coordinate console reporter
    auto reporter = new ConsoleReporter();
    reporter->setName("reporter");
    reporter->set_report_time_interval(0.1);
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::TranslationX)
                                  .getOutput("value"),
            "X");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::TranslationY)
                                  .getOutput("value"),
            "Y");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::TranslationZ)
                                  .getOutput("value"),
            "Z");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::Rotation1X)
                                  .getOutput("value"),
            "thetaX");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::Rotation2Y)
                                  .getOutput("value"),
            "thetaY");
    reporter->addToReport(joint->getCoordinate(FreeJoint::Coord::Rotation3Z)
                                  .getOutput("value"),
            "thetaZ");
    model.addComponent(reporter);

    return model;
}

void taskBasedControl() {
    const string example = "ExampleTaskBasedControl";

    auto model = constructModel(example);

    // construct a torque controller and supply the control strategy
    TaskSpaceTorqueController* controller = new TaskSpaceTorqueController();
    controller->set_ConstraintModel(NoConstraintModel());
    model.addController(controller);

    // initial configuration
    Vec3 initialPosition(0, 0.5, 0);

    // Position tracking
    auto station_task = new StationTask();
    station_task->setName("block_position");
    station_task->set_wrt_body("block");
    auto x_desired = Sine(1.0, 2 * Pi, 0, initialPosition[0]);
    auto y_desired = Constant(initialPosition[1]);
    auto z_desired = Constant(initialPosition[2]);
    station_task->set_position_functions(0, x_desired);
    station_task->set_position_functions(1, y_desired);
    station_task->set_position_functions(2, z_desired);
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(station_task);

    // Orientation tracking
    auto orientation_task = new OrientationTask();
    orientation_task->setName("block_orientation");
    orientation_task->set_wrt_body("block");
    auto ox_desired = Constant(0);
    auto oy_desired = Constant(0);
    auto oz_desired = Sine(0.2, 2*Pi, 0, 0);
    orientation_task->set_position_functions(0, ox_desired);
    orientation_task->set_position_functions(1, oy_desired);
    orientation_task->set_position_functions(2, oz_desired);
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(orientation_task);

    // Build and initialize model. Note this must be done AFTER adding the
    // tasks to ensure proper initialization.
    auto& state = model.initSystem();

    // Set the model pose to the initial configuration.
    auto joint = FreeJoint::safeDownCast(&model.updJointSet().get("joint"));
    joint->updCoordinate(FreeJoint::Coord::TranslationX)
            .setValue(state, initialPosition[0]);
    joint->updCoordinate(FreeJoint::Coord::TranslationY)
            .setValue(state, initialPosition[1]);
    joint->updCoordinate(FreeJoint::Coord::TranslationZ)
            .setValue(state, initialPosition[2]);

    // configure visualizer
    if (model.hasVisualizer()) {
        model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(
                Vec3(0));
        model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
                Visualizer::BackgroundType::SolidColor);
        model.updMatterSubsystem().setShowDefaultGeometry(true);
    }

    // print the complete model
    IO::makeDir("results");
    model.print(IO::getCwd()+"/results/"+example+".osim");

    // simulate
    simulate(model, state, 2, true);

    // export results
    controller->printResults(example, IO::getCwd()+"/results");
    auto reporter = model.getComponent<ConsoleReporter>("reporter");
    reporter.print("results/" + example + "_Reporter.sto");
}

int main(int argc, char* argv[]) {
    Logger::setLevel(Logger::Level::Info);
    try {
        taskBasedControl();
    } catch (exception& e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        return -1;
    }
    return 0;
}