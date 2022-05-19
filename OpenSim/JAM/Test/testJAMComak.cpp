/* -------------------------------------------------------------------------- *
 *                        OpenSim JAM: testCOMAK.cpp                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Colin Smith                                                     *
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

#include <OpenSim/JAM/COMAKTool.h>
#include <OpenSim/JAM/COMAKInverseKinematicsTool.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>

using namespace OpenSim;

void testCOMAK();

int main() {
    try {
        testCOMAK();

    } catch (const Exception& e) {
        e.print(std::cerr);
        return 1;
    }
    std::cout << "Done" << std::endl;
    return 0;
}

void testCOMAK() {
    bool useVisualizer = true;
    std::string model_dir = "C:/github/jam-resources/models/knee_healthy";
    std::string model_file = model_dir + "/lenhart2015/lenhart2015.osim";
    std::string external_loads_xml_file = model_dir + "/experimental_data/motion_analysis/overground_17_ext_loads.xml";

    std::string results_basename = "walk_test";
    std::string result_dir = "testJAMCOMAK";
    std::string ik_result_dir = result_dir + "/comak-inverse-kinematics";
    std::string comak_result_dir = result_dir + "/comak";
    std::string id_result_dir = result_dir + "/inverse-dynamics";
    std::string jnt_mech_result_dir = result_dir + "/joint-mechanics";

    IO::makeDir(result_dir);

    double start_time = 1.16;
    double start_pad = 0.0;
    double stop_time = 2.36;

    COMAKInverseKinematicsTool comak_ik = COMAKInverseKinematicsTool();
    comak_ik.set_model_file(model_file);
    comak_ik.set_results_directory(ik_result_dir);
    comak_ik.set_results_prefix(results_basename);
    comak_ik.set_perform_secondary_constraint_sim(true);
    comak_ik.set_secondary_coordinates(0, "/jointset/knee_r/knee_add_r");
    comak_ik.set_secondary_coordinates(1, "/jointset/knee_r/knee_rot_r");
    comak_ik.set_secondary_coordinates(2, "/jointset/knee_r/knee_tx_r");
    comak_ik.set_secondary_coordinates(3, "/jointset/knee_r/knee_ty_r");
    comak_ik.set_secondary_coordinates(4, "/jointset/knee_r/knee_tz_r");
    comak_ik.set_secondary_coordinates(5, "/jointset/pf_r/pf_flex_r");
    comak_ik.set_secondary_coordinates(6, "/jointset/pf_r/pf_rot_r");
    comak_ik.set_secondary_coordinates(7, "/jointset/pf_r/pf_tilt_r");
    comak_ik.set_secondary_coordinates(8, "/jointset/pf_r/pf_tx_r");
    comak_ik.set_secondary_coordinates(9, "/jointset/pf_r/pf_ty_r");
    comak_ik.set_secondary_coordinates(10, "/jointset/pf_r/pf_tz_r");
    comak_ik.set_secondary_coupled_coordinate("/jointset/knee_r/knee_flex_r");
    comak_ik.set_secondary_constraint_sim_settle_threshold(1e-4);
    comak_ik.set_secondary_constraint_sim_sweep_time(1.0);
    comak_ik.set_secondary_coupled_coordinate_start_value(0);
    comak_ik.set_secondary_coupled_coordinate_stop_value(90);
    comak_ik.set_secondary_constraint_sim_integrator_accuracy(1e-2);
    comak_ik.set_secondary_constraint_sim_internal_step_limit(10000);
    comak_ik.set_secondary_constraint_function_file(
        "./results/comak-inverse-kinematics/secondary_coordinate_constraint_functions.xml");
    comak_ik.set_constraint_function_num_interpolation_points(20);
    comak_ik.set_print_secondary_constraint_sim_results(true);
    comak_ik.set_constrained_model_file(
        "./results/comak-inverse-kinematics/ik_constrained_model.osim");
    comak_ik.set_perform_inverse_kinematics(true);
    comak_ik.set_marker_file(model_dir + 
        "/experimental_data/motion_analysis/overground_17.trc");

    comak_ik.set_output_motion_file("overground_17_ik.mot");
    comak_ik.set_time_range(0, start_time - start_pad);
    comak_ik.set_time_range(1, stop_time);
    comak_ik.set_report_errors(true);
    comak_ik.set_report_marker_locations(false);
    comak_ik.set_ik_constraint_weight(100);
    comak_ik.set_ik_accuracy(1e-5);
    comak_ik.set_use_visualizer(useVisualizer);


    IKTaskSet ik_task_set = IKTaskSet();

    IKMarkerTask ik_task = IKMarkerTask();

    ik_task.setName("S2");
    ik_task.setWeight(10);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.ASIS");
    ik_task.setWeight(10);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.PSIS");
    ik_task.setWeight(10);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.ASIS");
    ik_task.setWeight(10);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.PSIS");
    ik_task.setWeight(10);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.Clavicle");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.Clavicle");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.Shoulder");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.Shoulder");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.Bicep");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.Elbow");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.Forearm");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.Wrist");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.Bicep");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.Elbow");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.Forearm");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.Wrist");
    ik_task.setWeight(1);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.Knee");
    ik_task.setWeight(10);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.TH1");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.TH2");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.TH3");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.Ankle");
    ik_task.setWeight(10);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.SH1");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.SH2");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.SH3");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.MT5");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("R.Heel");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.Knee");
    ik_task.setWeight(10);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.TH1");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.TH2");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.TH3");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.TH4");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.Ankle");
    ik_task.setWeight(10);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.SH1");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.SH2");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.SH3");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.MT5");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    ik_task.setName("L.Heel");
    ik_task.setWeight(5);
    ik_task_set.cloneAndAppend(ik_task);

    comak_ik.set_IKTaskSet(ik_task_set);
    //comak_ik.print("./inputs/comak_inverse_kinematics_settings.xml");
    //comak_ik.run();

    InverseDynamicsTool id = InverseDynamicsTool();
    id.setResultsDir(id_result_dir);
    id.setModelFileName(model_file);
    id.setStartTime(start_time);
    id.setEndTime(stop_time);
    Array<std::string> exclude_frc = Array<std::string>();

    /*model = Model(model_file);

    for f = 1:model.getForceSet().getSize()
        frc = model.getForceSet().get(f - 1);

    exclude_frc.append(frc.getName());
    end*/

    exclude_frc.append("ALL");

    std::string comak_states_sto = "C:\\github\\jam-resources\\matlab\\example\\Walking\\Walking\\results\\comak\\walking_states.sto";

    id.setExcludedForces(exclude_frc);
    id.setExternalLoadsFileName(external_loads_xml_file);
    id.setCoordinatesFileName(comak_states_sto);
    id.setLowpassCutoffFrequency(6);
    id.setOutputGenForceFileName("test_inverse - dynamics.sto");
    id.run();
}
