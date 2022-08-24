/* -------------------------------------------------------------------------- *
 *                         OpenSim JAM: testJAMForsim.cpp                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include <OpenSim/JAM/ForsimTool.h>
#include <OpenSim/JAM/JointMechanicsTool.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>

using namespace OpenSim;

void testPassiveFlexion();
void testLigamentBalance();

int main() {
    try {
        //testPassiveFlexion();

    } catch (const Exception& e) {
        e.print(std::cerr);
        return 1;
    }
    std::cout << "Done" << std::endl;
    return 0;
}

void testPassiveFlexion() { 
    bool useVisualizer = true;
    std::string model_file = "C:/github/jam-resources/models/knee_healthy/smith2019/smith2019.osim";
    std::string forsim_result_dir = "./results/forsim";
    std::string jnt_mech_result_dir = "./results/joint-mechanics";
    std::string basename = "healthy";
    std::string prescribed_coordinates_file = "./inputs/prescribed_coordinates.sto";
    std::string muscle_input_sto_file = "./inputs/muscle_inputs.sto";
    //model = Model(model_file);

    ForsimTool forsim = ForsimTool();
    forsim.set_model_file(model_file);
    forsim.set_results_directory(forsim_result_dir);
    forsim.set_results_file_basename(basename);
    forsim.set_stop_time(-1);
    forsim.set_integrator_accuracy(1e-2);
    forsim.set_constant_muscle_control(0.02);
    forsim.set_use_activation_dynamics(true);
    forsim.set_use_tendon_compliance(true);
    forsim.set_use_muscle_physiology(true);
    forsim.set_equilibrate_muscles(true);
    forsim.set_unconstrained_coordinates(0, "/jointset/knee_r/knee_add_r");
    forsim.set_unconstrained_coordinates(1, "/jointset/knee_r/knee_rot_r");
    forsim.set_unconstrained_coordinates(2, "/jointset/knee_r/knee_tx_r");
    forsim.set_unconstrained_coordinates(3, "/jointset/knee_r/knee_ty_r");
    forsim.set_unconstrained_coordinates(4, "/jointset/knee_r/knee_tz_r");
    forsim.set_unconstrained_coordinates(5, "/jointset/pf_r/pf_flex_r");
    forsim.set_unconstrained_coordinates(6, "/jointset/pf_r/pf_rot_r");
    forsim.set_unconstrained_coordinates(7, "/jointset/pf_r/pf_tilt_r");
    forsim.set_unconstrained_coordinates(8, "/jointset/pf_r/pf_tx_r");
    forsim.set_unconstrained_coordinates(9, "/jointset/pf_r/pf_ty_r");
    forsim.set_unconstrained_coordinates(10, "/jointset/pf_r/pf_tz_r");
    forsim.set_unconstrained_coordinates(11, "/jointset/meniscus_medial_r/meniscus_medial_flex_r");
    forsim.set_unconstrained_coordinates(12, "/jointset/meniscus_medial_r/meniscus_medial_add_r");
    forsim.set_unconstrained_coordinates(13, "/jointset/meniscus_medial_r/meniscus_medial_rot_r");
    forsim.set_unconstrained_coordinates(14, "/jointset/meniscus_medial_r/meniscus_medial_tx_r");
    forsim.set_unconstrained_coordinates(15, "/jointset/meniscus_medial_r/meniscus_medial_ty_r");
    forsim.set_unconstrained_coordinates(16, "/jointset/meniscus_medial_r/meniscus_medial_tz_r");
    forsim.set_unconstrained_coordinates(17, "/jointset/meniscus_lateral_r/meniscus_lateral_flex_r");
    forsim.set_unconstrained_coordinates(18, "/jointset/meniscus_lateral_r/meniscus_lateral_add_r");
    forsim.set_unconstrained_coordinates(19, "/jointset/meniscus_lateral_r/meniscus_lateral_rot_r");
    forsim.set_unconstrained_coordinates(20, "/jointset/meniscus_lateral_r/meniscus_lateral_tx_r");
    forsim.set_unconstrained_coordinates(21, "/jointset/meniscus_lateral_r/meniscus_lateral_ty_r");
    forsim.set_unconstrained_coordinates(22, "/jointset/meniscus_lateral_r/meniscus_lateral_tz_r");
    forsim.set_prescribed_coordinates_file(prescribed_coordinates_file);

    forsim.set_actuator_input_file(muscle_input_sto_file);
    forsim.set_use_visualizer(useVisualizer);

    AnalysisSet analysis_set = AnalysisSet();

    ForceReporter frc_reporter = ForceReporter();
    frc_reporter.setName("ForceReporter");
    analysis_set.cloneAndAppend(frc_reporter);


    MuscleAnalysis msl_analysis = MuscleAnalysis();
    msl_analysis.setName("MuscleAnalysis");

    analysis_set.cloneAndAppend(msl_analysis);
    forsim.set_AnalysisSet(analysis_set);

    //forsim.print('./inputs/healthy_forsim_settings.xml');
    forsim.run();

    JointMechanicsTool jnt_mech = JointMechanicsTool();
    jnt_mech.set_model_file(model_file);
    jnt_mech.set_use_activation_dynamics(true);
    jnt_mech.set_use_tendon_compliance(true);
    jnt_mech.set_use_muscle_physiology(true);
    jnt_mech.set_input_states_file(
        forsim_result_dir + "/" + basename + "_states.sto");
    jnt_mech.set_input_activations_file(
        forsim_result_dir + "/" + basename + "_activations.sto");
    jnt_mech.set_input_forces_file(
        forsim_result_dir + "/" + basename + "_forces.sto");
    jnt_mech.set_results_file_basename(basename);
    jnt_mech.set_results_directory(jnt_mech_result_dir);
    jnt_mech.set_stop_time(-1);
    jnt_mech.set_normalize_to_cycle(false);
    jnt_mech.set_contacts(0, "all");
    jnt_mech.set_ligaments(0, "all");
    jnt_mech.set_muscles(0, "all");
    jnt_mech.set_muscles(0, "/forceset/vasmed_r");
    jnt_mech.set_muscles(1, "/forceset/vaslat_r");
    jnt_mech.set_muscles(2, "/forceset/vasint_r");
    jnt_mech.set_muscle_outputs(0, "all");
    jnt_mech.set_attached_geometry_bodies(0, "/bodyset/femur_distal_r");
    jnt_mech.set_attached_geometry_bodies(1, "/bodyset/tibia_proximal_r");
    jnt_mech.set_attached_geometry_bodies(2, "/bodyset/patella_r");
    jnt_mech.set_output_orientation_frame("ground");
    jnt_mech.set_output_position_frame("ground");
    jnt_mech.set_write_vtp_files(true);
    jnt_mech.set_write_h5_file(true);
    jnt_mech.set_h5_kinematics_data(true);
    jnt_mech.set_h5_states_data(true);

    jnt_mech.set_use_visualizer(useVisualizer);
    jnt_mech.print("./inputs/healthy_joint_mechanics_settings.xml");

    jnt_mech.run();
}

void testLigamentBalance() {
    /* for (int j = 0; j < 2; ++j) { 
        std::string basename = "ligament_balance_" + std::to_string(j);


    }


        ForsimTool lig_forsim = ForsimTool();
        lig_forsim.setModel(model);
        lig_forsim.set_results_directory(lig_balance_result_dir{i});
        lig_forsim.set_results_file_basename(basename);

        lig_forsim.set_start_time(0);
        lig_forsim.set_stop_time(2);
        lig_forsim.set_integrator_accuracy(1e-3);
        lig_forsim.set_constant_muscle_control(0.02);
        lig_forsim.set_use_activation_dynamics(true);
        lig_forsim.set_use_tendon_compliance(false);
        lig_forsim.set_use_muscle_physiology(true);
        lig_forsim.set_equilibrate_muscles(true);
        lig_forsim.set_unconstrained_coordinates(0, "/jointset/knee_r/knee_add_r");
        lig_forsim.set_unconstrained_coordinates(1, "/jointset/knee_r/knee_rot_r");
        lig_forsim.set_unconstrained_coordinates(2, "/jointset/knee_r/knee_tx_r");
        lig_forsim.set_unconstrained_coordinates(3, "/jointset/knee_r/knee_ty_r");
        lig_forsim.set_unconstrained_coordinates(4, "/jointset/knee_r/knee_tz_r");
        lig_forsim.set_unconstrained_coordinates(5, "/jointset/pf_r/pf_flex_r");
        lig_forsim.set_unconstrained_coordinates(6, "/jointset/pf_r/pf_rot_r");
        lig_forsim.set_unconstrained_coordinates(7, "/jointset/pf_r/pf_tilt_r");
        lig_forsim.set_unconstrained_coordinates(8, "/jointset/pf_r/pf_tx_r");
        lig_forsim.set_unconstrained_coordinates(9, "/jointset/pf_r/pf_ty_r");
        lig_forsim.set_unconstrained_coordinates(10, "/jointset/pf_r/pf_tz_r");

        lig_forsim.set_use_visualizer(true);
        lig_forsim.print("./inputs/ligament_balance_settings.xml");

        log_info("Running ForsimTool to settle knee. iteration: {}", i);
        lig_forsim.run();

        lig_balance_states_sto =
                [lig_balance_result_dir { i } '/' basename '_states.sto'];
        lig_balance_results =
                osimTableToStruct(TimeSeriesTable(lig_balance_states_sto));

        for
            s = 1 : numSecondaryCoordinates label =
                            ['a_jointset_' secondary_joints {
                                s
                            } '_' secondary_coordinates { s } '_value'];
        value = lig_balance_results.(label);
        value = value(end);
        coord = model.getCoordinateSet().get(secondary_coordinates{s});
        coord.setValue(state, value);
        coord.setDefaultValue(value);
        end

                nLig = 0;

        for
            f = 0 : frc_set.getSize() - 1 frc = frc_set.get(f);
        if (strcmp(frc.getConcreteClassName(), 'Blankevoort1991Ligament'))
            lig = Blankevoort1991Ligament.safeDownCast(frc);
        nLig = nLig + 1;
        lig.setSlackLengthFromReferenceStrain(default_ref_strain(nLig), state);
        end end end

                model.finalizeConnections();
        new_model_file = ['./inputs/DM_' sim_names { i } '.osim'];
        model.print(new_model_file); 
        */

}