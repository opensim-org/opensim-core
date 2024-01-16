/* -------------------------------------------------------------------------- *
 * OpenSim TaskSpace: ExampleStandingBalance2D.cpp                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 CFD Resarch Corporation and the Authors                 *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Nathan Pickle, Ryan Middle, Garrett Tuer  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/**
 * @file ExampleStandingBalance2D.cpp
 *
 * \brief
 *
 * @authors Nathan Pickle, Aravind Sundararajan, Garrett Tuer, Ryan Middle
 *
 */
#include "OpenSim/Common/osimCommon.h"
#include "OpenSim/Simulation/osimSimulation.h"
#include "OpenSim/Analyses/osimAnalyses.h"
#include "OpenSim/Tools/osimTools.h"
#include "OpenSim/Moco/MocoUtilities.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

#define USE_VISUALIZER 1

void balanceSimulation() {
    const string example = "ExampleStandingBalance2D";

    ModelVisualizer::addDirToGeometrySearchPaths(OPENSIM_GEOMETRY_DIR);

    // load model
    Model model("gait2d.osim");

    // // body kinematics
    auto bodyKinematics = new BodyKinematics(&model);
    bodyKinematics->setInDegrees(false);
    model.addAnalysis(bodyKinematics);

    auto kinematics = new StatesReporter(&model);
    kinematics->setInDegrees(false);
    model.addAnalysis(kinematics);

    // define the controller
    auto controller = new TaskSpaceTorqueController();
    // Specify the constraint model to use
    controller->set_ConstraintModel(SupportModel());

    model.addController(controller);

    // Specify unactuated coordinates in the model
    Array<string> excluded_coords;
    excluded_coords.append("rz");
    excluded_coords.append("tx");
    excluded_coords.append("ty");
    controller->set_excluded_coords(excluded_coords);

    // build and initialize model
    auto& state = model.initSystem();

    // initial configuration
    auto& torso = model.updBodySet().get("torso");
    auto initialOrientation_torso = torso.getRotationInGround(state).convertRotationToBodyFixedXYZ();
    auto initialPosition_torso = torso.findStationLocationInGround(state, torso.getMassCenter());

    auto& pelvis = model.updBodySet().get("pelvis");
    auto initialOrientation_pelvis = pelvis.getRotationInGround(state).convertRotationToBodyFixedXYZ();
    auto initialPosition_pelvis = pelvis.findStationLocationInGround(state, pelvis.getMassCenter());

    auto& hand_r = model.updBodySet().get("hand_r");
    auto initialOrientation_hand_r = hand_r.getRotationInGround(state).convertRotationToBodyFixedXYZ();
    auto initialPosition_hand_r = hand_r.findStationLocationInGround(state, hand_r.getMassCenter());

    auto& hand_l = model.updBodySet().get("hand_l");
    auto initialOrientation_hand_l = hand_l.getRotationInGround(state).convertRotationToBodyFixedXYZ();
    auto initialPosition_hand_l = hand_l.findStationLocationInGround(state, hand_l.getMassCenter());

    Array<double> weights(1.0, 3);

    //========================
    // TORSO
    //========================
    // Torso orientation
    auto torso_orientation_task = new OrientationTask();
    torso_orientation_task->setName("torso_orientation_task");
    torso_orientation_task->set_priority(0);
    torso_orientation_task->set_kp(Array<double>(100, 3));
    torso_orientation_task->set_kv(Array<double>(20, 3));
    weights[0] = 0.0;
    weights[1] = 0.0;
    weights[2] = 1;
    torso_orientation_task->set_weight(weights);
    auto tx_desired_com_2 = Constant(initialOrientation_torso[0]);
    auto ty_desired_com_2 = Constant(initialOrientation_torso[1]);
    auto tz_desired_com_2 = Constant(initialOrientation_torso[2]);
    torso_orientation_task->set_position_functions(0, tx_desired_com_2);
    torso_orientation_task->set_position_functions(1, ty_desired_com_2);
    torso_orientation_task->set_position_functions(2, tz_desired_com_2);
    torso_orientation_task->set_wrt_body("torso");
    torso_orientation_task->set_express_body("ground");
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(torso_orientation_task);

    //========================
    // PELVIS
    //========================

    // Pelvis forward progression tracking
    auto pelvis_position_task = new StationTask();
    pelvis_position_task->setName("pelvis_position");
    pelvis_position_task->set_priority(0);
    pelvis_position_task->set_kp(Array<double>(100, 3));
    pelvis_position_task->set_kv(Array<double>(20, 3));
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 0.0;
    pelvis_position_task->set_weight(weights);
    pelvis_position_task->set_point(pelvis.getMassCenter());
    auto tx_desired_pelvis = Constant(initialPosition_pelvis[0]);
    auto ty_desired_pelvis = Constant(initialPosition_pelvis[1]);
    auto tz_desired_pelvis = Constant(0);
    pelvis_position_task->set_position_functions(0, tx_desired_pelvis);
    pelvis_position_task->set_position_functions(1, ty_desired_pelvis);
    pelvis_position_task->set_position_functions(2, tz_desired_pelvis);
    pelvis_position_task->set_wrt_body("pelvis");
    pelvis_position_task->set_express_body("ground");
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(pelvis_position_task);

    // Pelvis orientation
    auto pelvis_orientation_task = new OrientationTask();
    pelvis_orientation_task->setName("pelvis_orientation_task");
    pelvis_orientation_task->set_priority(0);
    pelvis_orientation_task->set_kp(Array<double>(100, 3));
    pelvis_orientation_task->set_kv(Array<double>(20, 3));
    weights[0] = 0.0;
    weights[1] = 0.0;
    weights[2] = 1.0;
    pelvis_orientation_task->set_weight(weights);
    auto rx_desired_pelvis = Constant(initialOrientation_pelvis[0]);
    auto ry_desired_pelvis = Constant(initialOrientation_pelvis[1]);
    auto rz_desired_pelvis = Constant(initialOrientation_pelvis[2]);
    pelvis_orientation_task->set_position_functions(0, rx_desired_pelvis);
    pelvis_orientation_task->set_position_functions(1, ry_desired_pelvis);
    pelvis_orientation_task->set_position_functions(2, rz_desired_pelvis);
    pelvis_orientation_task->set_wrt_body("pelvis");
    pelvis_orientation_task->set_express_body("ground");
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(pelvis_orientation_task);

    //========================
    // HANDS
    //========================

    // RIGHT HAND
    auto hand_position_r = new StationTask();
    hand_position_r->setName("hand_position_r");
    hand_position_r->set_priority(0);
    hand_position_r->set_kp(Array<double>(100,3));
    hand_position_r->set_kv(Array<double>(20,3));
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 0.0;
    hand_position_r->set_weight(weights);
    hand_position_r->set_point(hand_r.getMassCenter());
    auto tx_desired_hand_r = Constant(initialPosition_hand_r[0]);
    auto ty_desired_hand_r = Constant(initialPosition_hand_r[1]);
    auto tz_desired_hand_r = Constant(initialPosition_hand_r[2]);
    hand_position_r->set_position_functions(0, tx_desired_hand_r);
    hand_position_r->set_position_functions(1, ty_desired_hand_r);
    hand_position_r->set_position_functions(2, tz_desired_hand_r);
    hand_position_r->set_wrt_body("hand_r");
    hand_position_r->set_express_body("ground");
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(hand_position_r);

    auto hand_orientation_r = new OrientationTask();
    hand_orientation_r->setName("hand_orientation_r");
    hand_orientation_r->set_priority(0);
    hand_orientation_r->set_kp(Array<double>(100,3));
    hand_orientation_r->set_kv(Array<double>(20,3));
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 0.0;
    hand_orientation_r->set_weight(weights);
    auto rx_desired_hand_r = Constant(initialOrientation_hand_r[0]);
    auto ry_desired_hand_r = Constant(initialOrientation_hand_r[1]);
    auto rz_desired_hand_r = Constant(initialOrientation_hand_r[2]);
    hand_orientation_r->set_position_functions(0, rx_desired_hand_r);
    hand_orientation_r->set_position_functions(1, ry_desired_hand_r);
    hand_orientation_r->set_position_functions(2, rz_desired_hand_r);
    hand_orientation_r->set_wrt_body("hand_r");
    hand_orientation_r->set_express_body("ground");
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(hand_orientation_r);
    
    // LEFT HAND
    auto hand_position_l = new StationTask();
    hand_position_l->setName("hand_position_l");
    hand_position_l->set_priority(0);
    hand_position_l->set_kp(Array<double>(100,3));
    hand_position_l->set_kv(Array<double>(20,3));
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 0.0;
    hand_position_l->set_weight(weights);
    hand_position_l->set_point(hand_l.getMassCenter());
    auto tx_desired_hand_l = Constant(initialPosition_hand_l[0]);
    auto ty_desired_hand_l = Constant(initialPosition_hand_l[1]);
    auto tz_desired_hand_l = Constant(initialPosition_hand_l[2]);
    hand_position_l->set_position_functions(0, tx_desired_hand_l);
    hand_position_l->set_position_functions(1, ty_desired_hand_l);
    hand_position_l->set_position_functions(2, tz_desired_hand_l);
    hand_position_l->set_wrt_body("hand_l");
    hand_position_l->set_express_body("ground");
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(hand_position_l);

    auto hand_orientation_l = new OrientationTask();
    hand_orientation_l->setName("hand_orientation_l");
    hand_orientation_l->set_priority(0);
    hand_orientation_l->set_kp(Array<double>(100,3));
    hand_orientation_l->set_kv(Array<double>(20,3));
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 0.0;
    hand_orientation_l->set_weight(weights);
    auto rx_desired_hand_l = Constant(initialOrientation_hand_l[0]);
    auto ry_desired_hand_l = Constant(initialOrientation_hand_l[1]);
    auto rz_desired_hand_l = Constant(initialOrientation_hand_l[2]);
    hand_orientation_l->set_position_functions(0, rx_desired_hand_l);
    hand_orientation_l->set_position_functions(1, ry_desired_hand_l);
    hand_orientation_l->set_position_functions(2, rz_desired_hand_l);
    hand_orientation_l->set_wrt_body("hand_l");
    hand_orientation_l->set_express_body("ground");
    controller->upd_TaskSpaceTaskSet().adoptAndAppend(hand_orientation_l);

    model.setUseVisualizer(true);
    state = model.initSystem();

#if USE_VISUALIZER == 1
    model.updVisualizer().updSimbodyVisualizer().setBackgroundColor(Vec3(0));
    model.updVisualizer().updSimbodyVisualizer().setBackgroundType(Visualizer::BackgroundType::SolidColor);
    model.updVisualizer().updSimbodyVisualizer().setDesiredFrameRate(200);
    model.updVisualizer().updSimbodyVisualizer().setShowFrameRate(true);
    model.updVisualizer().updSimbodyVisualizer().setShowSimTime(true);
    model.updVisualizer().updSimbodyVisualizer().setRealTimeScale(0.5);
    model.updMatterSubsystem().setShowDefaultGeometry(false);
#endif

    state.setTime(0.0);
    Manager manager(model);
    manager.initialize(state);
    double end_time = 2;
    log_info("\nIntegrating from {} to {}", 0.0, end_time);
    state = manager.integrate(end_time);
    model.setUseVisualizer(false);

    // export results
    IO::makeDir("./results");
    controller->printResults(example, "./results");
    bodyKinematics->printResults(example, "./results");
    kinematics->printResults(example, "./results");
    auto tab = manager.getStatesTable();
    StatesTrajectory ST = StatesTrajectory::createFromStatesTable(model, tab);
    std::vector<std::string> contact_r;
    std::vector<std::string> contact_l;
    contact_r.push_back("/forceset/contactHeel_r");
    contact_r.push_back("/forceset/contactToes_r");
    contact_l.push_back("/forceset/contactHeel_l");
    contact_l.push_back("/forceset/contactToes_l");
    auto externalForcesTableFlat = createExternalLoadsTableForGait(model, ST, contact_r, contact_l);
    STOFileAdapter::write(externalForcesTableFlat,
                          "./results/"+example+"_grf.sto");
    log_info("Done!");
}

int main(int argc, char* argv[]) {
    Logger::setLevel(Logger::Level::Info);
    try {
        balanceSimulation();
    } catch (exception& e) {
        cout << typeid(e).name() << ": " << e.what() << endl;
        getchar();
        return -1;
    }
    return 0;
}
