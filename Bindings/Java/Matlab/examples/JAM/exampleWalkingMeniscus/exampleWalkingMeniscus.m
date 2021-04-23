%% Setup Environment and Folders
clear;
import org.opensim.modeling.*
Logger.setLevelString('Info');

model_file = '../models/healthy/smith2019/smith2019.osim';
results_basename = 'walking_meniscus';
ik_result_dir = './results/comak-inverse-kinematics';
comak_result_dir = './results/comak';
jnt_mech_result_dir = './results/joint-mechanics';

if(exist('./inputs','dir')~=7)
    mkdir('./inputs')
end
if(exist('./results','dir')~=7)
    mkdir('./results')
end
if(exist('./results/graphics','dir')~=7)
    mkdir('./results/graphics')
end

%% Perform Inverse Kinematics
comak_ik = COMAKInverseKinematicsTool();
comak_ik.set_model_file(model_file);
comak_ik.set_results_directory(ik_result_dir);
comak_ik.set_results_prefix(results_basename);
comak_ik.set_perform_secondary_constraint_sim(true);
comak_ik.set_secondary_coordinates(0,'/jointset/knee_r/knee_add_r');
comak_ik.set_secondary_coordinates(1,'/jointset/knee_r/knee_rot_r');
comak_ik.set_secondary_coordinates(2,'/jointset/knee_r/knee_tx_r');
comak_ik.set_secondary_coordinates(3,'/jointset/knee_r/knee_ty_r');
comak_ik.set_secondary_coordinates(4,'/jointset/knee_r/knee_tz_r');
comak_ik.set_secondary_coordinates(5,'/jointset/pf_r/pf_flex_r');
comak_ik.set_secondary_coordinates(6,'/jointset/pf_r/pf_rot_r');
comak_ik.set_secondary_coordinates(7,'/jointset/pf_r/pf_tilt_r');
comak_ik.set_secondary_coordinates(8,'/jointset/pf_r/pf_tx_r');
comak_ik.set_secondary_coordinates(9,'/jointset/pf_r/pf_ty_r');
comak_ik.set_secondary_coordinates(10,'/jointset/pf_r/pf_tz_r');
comak_ik.set_secondary_coordinates(11,'/jointset/meniscus_medial_r/meniscus_medial_flex_r');
comak_ik.set_secondary_coordinates(12,'/jointset/meniscus_medial_r/meniscus_medial_add_r');
comak_ik.set_secondary_coordinates(13,'/jointset/meniscus_medial_r/meniscus_medial_rot_r');
comak_ik.set_secondary_coordinates(14,'/jointset/meniscus_medial_r/meniscus_medial_tx_r');
comak_ik.set_secondary_coordinates(15,'/jointset/meniscus_medial_r/meniscus_medial_ty_r');
comak_ik.set_secondary_coordinates(16,'/jointset/meniscus_medial_r/meniscus_medial_tz_r');
comak_ik.set_secondary_coordinates(17,'/jointset/meniscus_lateral_r/meniscus_lateral_flex_r');
comak_ik.set_secondary_coordinates(18,'/jointset/meniscus_lateral_r/meniscus_lateral_add_r');
comak_ik.set_secondary_coordinates(19,'/jointset/meniscus_lateral_r/meniscus_lateral_rot_r');
comak_ik.set_secondary_coordinates(20,'/jointset/meniscus_lateral_r/meniscus_lateral_tx_r');
comak_ik.set_secondary_coordinates(21,'/jointset/meniscus_lateral_r/meniscus_lateral_ty_r');
comak_ik.set_secondary_coordinates(22,'/jointset/meniscus_lateral_r/meniscus_lateral_tz_r');
comak_ik.set_secondary_coupled_coordinate('/jointset/knee_r/knee_flex_r');
comak_ik.set_secondary_constraint_sim_settle_threshold(1e-4);
comak_ik.set_secondary_constraint_sim_sweep_time(3.0);
comak_ik.set_secondary_coupled_coordinate_start_value(0);
comak_ik.set_secondary_coupled_coordinate_stop_value(100);
comak_ik.set_secondary_constraint_sim_integrator_accuracy(1e-3);
comak_ik.set_secondary_constraint_sim_internal_step_limit(10000);
comak_ik.set_secondary_constraint_function_file(...
    './results/comak-inverse-kinematics/secondary_coordinate_constraint_functions.xml');
comak_ik.set_constraint_function_num_interpolation_points(20);
comak_ik.set_print_secondary_constraint_sim_results(true);
comak_ik.set_constrained_model_file('./results/comak-inverse-kinematics/ik_constrained_model.osim');
comak_ik.set_perform_inverse_kinematics(true);
comak_ik.set_marker_file('../models/healthy/experimental_data/motion_analysis/overground_17.trc');
comak_ik.set_output_motion_file('overground_17_ik.mot');
comak_ik.set_time_range(0, 0);
comak_ik.set_time_range(1, 2.36);
comak_ik.set_report_errors(true);
comak_ik.set_report_marker_locations(false);
comak_ik.set_ik_constraint_weight(100);
comak_ik.set_ik_accuracy(1e-5);
comak_ik.set_use_visualizer(true);
comak_ik.set_verbose(10);


ik_task_set = IKTaskSet();

ik_task=IKMarkerTask();

ik_task.setName('R_HJC');
ik_task.setWeight(0);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L_HJC');
ik_task.setWeight(0);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('S2');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.ASIS');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.PSIS');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.ASIS');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.PSIS');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Clavicle');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Clavicle');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Scapula');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Scapula');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Shoulder');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Shoulder');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Bicep');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Elbow');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Forearm');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Wrist');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Bicep');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Elbow');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Forearm');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Wrist');
ik_task.setWeight(1);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Knee');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.TH1');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.TH2');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.TH3');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.TH4');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Ankle');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.SH1');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.SH2');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.SH3');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.SH4');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.MT5');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('R.Heel');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Knee');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.TH1');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.TH2');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.TH3');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.TH4');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Ankle');
ik_task.setWeight(10);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.SH1');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.SH2');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.SH3');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.SH4');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.MT5');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

ik_task.setName('L.Heel');
ik_task.setWeight(5);
ik_task_set.cloneAndAppend(ik_task);

comak_ik.set_IKTaskSet(ik_task_set);
comak_ik.print('./inputs/comak_inverse_kinematics_settings.xml');

disp('Running COMAKInverseKinematicsTool...')
% comak_ik.run();
%% Perform COMAK Simulation

comak = COMAKTool();
comak.set_model_file(model_file);
comak.set_coordinates_file('./results/comak-inverse-kinematics/overground_17_ik.mot');
comak.set_external_loads_file('../models/healthy/experimental_data/motion_analysis/overground_17_ext_loads.xml'),
comak.set_results_directory(comak_result_dir);
comak.set_results_prefix(results_basename);
comak.set_replace_force_set(false);
comak.set_force_set_file('../models/healthy/smith2019/smith2019_reserve_actuators.xml');
comak.set_start_time(1.16);
comak.set_stop_time(2.32);
% comak.set_stop_time(1.26);
comak.set_time_step(0.01);
comak.set_lowpass_filter_frequency(6);
comak.set_print_processed_input_kinematics(false);
comak.set_prescribed_coordinates(0,'/jointset/gnd_pelvis/pelvis_tx');
comak.set_prescribed_coordinates(1,'/jointset/gnd_pelvis/pelvis_ty');
comak.set_prescribed_coordinates(2,'/jointset/gnd_pelvis/pelvis_tz');
comak.set_prescribed_coordinates(3,'/jointset/gnd_pelvis/pelvis_tilt');
comak.set_prescribed_coordinates(4,'/jointset/gnd_pelvis/pelvis_list');
comak.set_prescribed_coordinates(5,'/jointset/gnd_pelvis/pelvis_rot');
comak.set_prescribed_coordinates(6,'/jointset/subtalar_r/subt_angle_r');
comak.set_prescribed_coordinates(7,'/jointset/mtp_r/mtp_angle_r');
comak.set_prescribed_coordinates(8,'/jointset/hip_l/hip_flex_l');
comak.set_prescribed_coordinates(9,'/jointset/hip_l/hip_add_l');
comak.set_prescribed_coordinates(10,'/jointset/hip_l/hip_rot_l');
comak.set_prescribed_coordinates(11,'/jointset/pf_l/pf_l_r3');
comak.set_prescribed_coordinates(12,'/jointset/pf_l/pf_l_tx');
comak.set_prescribed_coordinates(13,'/jointset/pf_l/pf_l_ty');
comak.set_prescribed_coordinates(14,'/jointset/knee_l/knee_flex_l');
comak.set_prescribed_coordinates(15,'/jointset/ankle_l/ankle_flex_l');
comak.set_prescribed_coordinates(16,'/jointset/subtalar_l/subt_angle_l');
comak.set_prescribed_coordinates(17,'/jointset/mtp_l/mtp_angle_l');
comak.set_prescribed_coordinates(18,'/jointset/pelvis_torso/lumbar_ext');
comak.set_prescribed_coordinates(19,'/jointset/pelvis_torso/lumbar_latbend');
comak.set_prescribed_coordinates(20,'/jointset/pelvis_torso/lumbar_rot');
comak.set_prescribed_coordinates(21,'/jointset/torso_neckhead/neck_ext');
comak.set_prescribed_coordinates(22,'/jointset/torso_neckhead/neck_latbend');
comak.set_prescribed_coordinates(23,'/jointset/torso_neckhead/neck_rot');
comak.set_prescribed_coordinates(24,'/jointset/acromial_r/arm_add_r');
comak.set_prescribed_coordinates(25,'/jointset/acromial_r/arm_flex_r');
comak.set_prescribed_coordinates(26,'/jointset/acromial_r/arm_rot_r');
comak.set_prescribed_coordinates(27,'/jointset/elbow_r/elbow_flex_r');
comak.set_prescribed_coordinates(28,'/jointset/radioulnar_r/pro_sup_r');
comak.set_prescribed_coordinates(29,'/jointset/radius_hand_r/wrist_flex_r');
comak.set_prescribed_coordinates(30,'/jointset/acromial_l/arm_add_l');
comak.set_prescribed_coordinates(31,'/jointset/acromial_l/arm_flex_l');
comak.set_prescribed_coordinates(32,'/jointset/acromial_l/arm_rot_l');
comak.set_prescribed_coordinates(33,'/jointset/elbow_l/elbow_flex_l');
comak.set_prescribed_coordinates(34,'/jointset/radioulnar_l/pro_sup_l');
comak.set_prescribed_coordinates(35,'/jointset/radius_hand_l/wrist_flex_l');
 
comak.set_primary_coordinates(0,'/jointset/hip_r/hip_flex_r');
comak.set_primary_coordinates(1,'/jointset/hip_r/hip_add_r');
comak.set_primary_coordinates(2,'/jointset/hip_r/hip_rot_r');
comak.set_primary_coordinates(3,'/jointset/knee_r/knee_flex_r');
comak.set_primary_coordinates(4,'/jointset/ankle_r/ankle_flex_r');

secondary_coord_set = COMAKSecondaryCoordinateSet(); 
secondary_coord = COMAKSecondaryCoordinate();

secondary_coord.setName('knee_add_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/knee_r/knee_add_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_rot_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/knee_r/knee_rot_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_tx_r');
secondary_coord.set_max_change(0.05);
secondary_coord.set_coordinate('/jointset/knee_r/knee_tx_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_ty_r');
secondary_coord.set_max_change(0.05);
secondary_coord.set_coordinate('/jointset/knee_r/knee_ty_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('knee_tz_r');
secondary_coord.set_max_change(0.05);
secondary_coord.set_coordinate('/jointset/knee_r/knee_tz_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_flex_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/pf_r/pf_flex_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_rot_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/pf_r/pf_rot_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_tilt_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/pf_r/pf_tilt_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_tx_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/pf_r/pf_tx_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_ty_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/pf_r/pf_ty_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('pf_tz_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/pf_r/pf_tz_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_medial_flex_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/meniscus_medial_r/meniscus_medial_flex_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_medial_add_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/meniscus_medial_r/meniscus_medial_add_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_medial_rot_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/meniscus_medial_r/meniscus_medial_rot_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_medial_tx_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/meniscus_medial_r/meniscus_medial_tx_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_medial_ty_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/meniscus_medial_r/meniscus_medial_ty_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_medial_tz_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/meniscus_medial_r/meniscus_medial_tz_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_lateral_flex_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/meniscus_lateral_r/meniscus_lateral_flex_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_lateral_add_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/meniscus_lateral_r/meniscus_lateral_add_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_lateral_rot_r');
secondary_coord.set_max_change(0.01);
secondary_coord.set_coordinate('/jointset/meniscus_lateral_r/meniscus_lateral_rot_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_lateral_tx_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/meniscus_lateral_r/meniscus_lateral_tx_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_lateral_ty_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/meniscus_lateral_r/meniscus_lateral_ty_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

secondary_coord.setName('meniscus_lateral_tz_r');
secondary_coord.set_max_change(0.005);
secondary_coord.set_coordinate('/jointset/meniscus_lateral_r/meniscus_lateral_tz_r');
secondary_coord_set.cloneAndAppend(secondary_coord);

comak.set_COMAKSecondaryCoordinateSet(secondary_coord_set);

comak.set_settle_secondary_coordinates_at_start(true);
comak.set_settle_threshold(1e-3);
comak.set_settle_accuracy(1e-2);
comak.set_settle_internal_step_limit(10000);
comak.set_print_settle_sim_results(true);
comak.set_settle_sim_results_directory(comak_result_dir);
comak.set_settle_sim_results_prefix('walking_meniscus_settle_sim');
comak.set_max_iterations(25);
comak.set_udot_tolerance(1);
comak.set_udot_worse_case_tolerance(50);
comak.set_unit_udot_epsilon(1e-6);
comak.set_optimization_scale_delta_coord(1);
comak.set_ipopt_diagnostics_level(3);
comak.set_ipopt_max_iterations(500);
comak.set_ipopt_convergence_tolerance(1e-4);
comak.set_ipopt_constraint_tolerance(1e-4);
comak.set_ipopt_limited_memory_history(200);
comak.set_ipopt_nlp_scaling_max_gradient(10000);
comak.set_ipopt_nlp_scaling_min_value(1e-8);
comak.set_ipopt_obj_scaling_factor(1);
comak.set_activation_exponent(2);
comak.set_contact_energy_weight(0);
comak.set_non_muscle_actuator_weight(1000);
comak.set_model_assembly_accuracy(1e-12);
comak.set_use_visualizer(true);
comak.set_verbose(2);

comak.print('./inputs/comak_settings.xml');

disp('Running COMAK Tool...')
comak.run();

%% Perform Joint Mechanics Analysis
jnt_mech = JointMechanicsTool();
jnt_mech.set_model_file(model_file);
jnt_mech.set_input_states_file([comak_result_dir '/' results_basename '_states.sto']);
jnt_mech.set_use_muscle_physiology(false);
jnt_mech.set_results_file_basename(results_basename);
jnt_mech.set_results_directory(jnt_mech_result_dir);
jnt_mech.set_start_time(1.16);
jnt_mech.set_stop_time(-1);
jnt_mech.set_resample_step_size(-1);
jnt_mech.set_normalize_to_cycle(true);
jnt_mech.set_lowpass_filter_frequency(-1);
jnt_mech.set_print_processed_kinematics(false);
jnt_mech.set_contacts(0,'all');
jnt_mech.set_contact_outputs(0,'all');
jnt_mech.set_contact_mesh_properties(0,'none');
jnt_mech.set_ligaments(0,'all');
jnt_mech.set_ligament_outputs(0,'all');
jnt_mech.set_muscles(0,'all');
jnt_mech.set_muscle_outputs(0,'all');

jnt_mech.set_attached_geometry_bodies(0,'all');

jnt_mech.set_output_orientation_frame('ground');
jnt_mech.set_output_position_frame('ground');
jnt_mech.set_write_vtp_files(true);
jnt_mech.set_vtp_file_format('binary');
jnt_mech.set_write_h5_file(false);
jnt_mech.set_h5_kinematics_data(true);
jnt_mech.set_h5_states_data(true);
jnt_mech.set_write_transforms_file(false);
jnt_mech.set_output_transforms_file_type('sto');
jnt_mech.set_use_visualizer(true);
jnt_mech.set_verbose(0);

analysis_set = AnalysisSet();

frc_reporter = ForceReporter();
frc_reporter.setName('ForceReporter');

analysis_set.cloneAndAppend(frc_reporter);
jnt_mech.set_AnalysisSet(analysis_set);
jnt_mech.print('./inputs/joint_mechanics_settings.xml');

disp('Running JointMechanicsTool...');
jnt_mech.run();
