close all; clear

import org.opensim.modeling.*
Logger.setLevelString('Info');

if(exist('./inputs','dir')~=7)
    mkdir('./inputs')
end
if(exist('./results','dir')~=7)
    mkdir('./results')
end
if(exist('./results/graphics','dir')~=7)
    mkdir('./results/graphics')
end

model_file = '../models/healthy/current/full_body_healthy_knee.osim';

force_magnitude = 100; % 100 N anterior force, similar to KT-1000 arthrometer
force_point_height = -0.1; %Apply at the tibial tuberosity height

%% Simulation Time
% Simulation consists of four phases:
% settle : allow knee to settle into equilbrium
% flex   : hip and knee flexion
% settle : allow knee to settle into equilbrium 
% force  : ramp up the anterior force

time_step = 0.01;

settle1_duration = 0.5;
flex_duration = 1.0;
settle2_duration = 0.5;
force_duration = 1.0;


settle1_time = 0 : time_step : settle1_duration;

flex_time = ...
    settle1_duration + time_step : ...
    time_step : ...
    settle1_duration + flex_duration;

settle2_time = ...
    settle1_duration + flex_duration + time_step : ...
    time_step : ...
    settle1_duration + flex_duration + settle2_duration;

force_time = ...
    settle1_duration + flex_duration + settle2_duration + time_step : ...
    time_step : ...
    settle1_duration + flex_duration + settle2_duration + force_duration;

time = [settle1_time, flex_time, settle2_time, force_time];

time_points = [0,settle1_duration,...
    settle1_duration + flex_duration,...
    settle1_duration + flex_duration + settle2_duration,...
    settle1_duration + flex_duration + settle2_duration + force_duration];

num_settle1_steps = length(settle1_time);
num_flex_steps = length(flex_time);
num_settle2_steps = length(settle2_time);
num_force_steps = length(force_time);
num_steps = length(time);

%% Create Input Files
% Prescribed Coordinates File
%----------------------------
prescribed_coordinates_file = './inputs/prescribed_coordinates.sto';

max_hip_flex = 25;
max_knee_flex = 25;

coord_data.time = time;

hip_flex = [0,0,max_hip_flex,max_hip_flex,max_hip_flex];
knee_flex = [0,0,max_knee_flex,max_knee_flex,max_knee_flex];

smooth_hip_flex = interp1(time_points, hip_flex, time,'pchip');
smooth_knee_flex = interp1(time_points, knee_flex, time,'pchip');

coord_data.hip_flex_r = smooth_hip_flex';
coord_data.knee_flex_r = smooth_knee_flex';

% Function distributed in OpenSim Resources\Code\Matlab\Utilities
coord_table = osimTableFromStruct(coord_data); 

STOFileAdapter.write(coord_table,prescribed_coordinates_file);

% Plot Prescribed coordinates 
%----------------------------
coord_fig = figure('name','INPUT: Prescribed Coordinates',...
    'Position',  [100, 100, 667, 300]);

subplot(1,2,1);
plot(time,coord_data.hip_flex_r,'LineWidth',2)
ylim([0.0 35])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Hip Flexion (hip\_flex\_r)')
box off

subplot(1,2,2);
plot(time,coord_data.knee_flex_r,'LineWidth',2)
ylim([0.0 35])
xlabel('Time [s]')
ylabel('Angle [^o]')
title('Knee Flexion (knee\_flex\_r)')
box off

saveas(coord_fig,'./results/graphics/prescribed_coordinates.png')

% External Loads Files
%---------------------

% write .sto file
external_loads_sto_file = 'external_loads.sto';

force_vx = [0,0,0,0,force_magnitude];
smooth_force_vx = interp1(time_points, force_vx, time,'pchip');

force_data.time = time;
force_data.tibia_proximal_r_force_vx = smooth_force_vx';
force_data.tibia_proximal_r_force_vy = zeros(num_steps,1);
force_data.tibia_proximal_r_force_vz = zeros(num_steps,1);
force_data.tibia_proximal_r_force_px = zeros(num_steps,1);
force_data.tibia_proximal_r_force_py = ...
                                    ones(num_steps,1) * force_point_height;
force_data.tibia_proximal_r_force_pz = zeros(num_steps,1);
force_data.tibia_proximal_r_torque_x = zeros(num_steps,1);
force_data.tibia_proximal_r_torque_y = zeros(num_steps,1);
force_data.tibia_proximal_r_torque_z = zeros(num_steps,1);

% Function distributed in OpenSim Resources\Code\Matlab\Utilities
force_table = osimTableFromStruct(force_data); 

force_table.addTableMetaDataString(...
    'header','Anterior Tibial External Force')

STOFileAdapter.write(force_table,['./inputs/' external_loads_sto_file]);

% write .xml file
external_loads_xml_file = './inputs/external_loads.xml';

ext_force = ExternalForce();
ext_force.setName('AnteriorForce');
ext_force.set_applied_to_body('tibia_proximal_r');
ext_force.set_force_expressed_in_body('tibia_proximal_r');
ext_force.set_point_expressed_in_body('tibia_proximal_r');
ext_force.set_force_identifier('tibia_proximal_r_force_v');
ext_force.set_point_identifier('tibia_proximal_r_force_p');
ext_force.set_torque_identifier('tibia_proximal_r_torque_');

ext_loads = ExternalLoads();
ext_loads.setDataFileName(external_loads_sto_file);
ext_loads.adoptAndAppend(ext_force);
ext_loads.print(external_loads_xml_file );

% Plot External Loads
ext_loads_fig = figure('name','INPUT: External Loads', ...
                'Position',  [100, 100, 333, 300]);
plot(time,force_data.tibia_proximal_r_force_vx,'LineWidth',2)
ylim([0.0 100])
xlabel('Time [s]')
ylabel('Anterior Force [N]')
title('External Loads on the Tibia')
box off

saveas(ext_loads_fig,'./results/graphics/external_loads.png')

%% Create ACL deficient model

model = Model(model_file);
acld_model = model;

%Remove ACL Ligaments
n=1;
for i = 0:acld_model.getForceSet.getSize()-1
    force = acld_model.getForceSet.get(i);
    if(contains(char(force.getName()),'ACL'))
        ACL_names{n} = force.getName();
        n=n+1;
    end
end

for i = 1:length(ACL_names)
    force = acld_model.getForceSet.get(ACL_names{i});
    acld_model.getForceSet.remove(force);
end

acld_model.initSystem();
acld_model_file = './inputs/acl_deficient_knee.osim';
acld_model.print(acld_model_file);

%% Perform Simulation with ForsimTool
% Healthy
healthy_forsim_result_dir = './results/healthy_forsim';
healthy_basename = 'healthy';

forsim = ForsimTool();
forsim.setModel(model);
forsim.set_results_directory(healthy_forsim_result_dir);
forsim.set_results_file_basename(healthy_basename);
forsim.set_start_time(-1);
forsim.set_stop_time(-1);
forsim.set_integrator_accuracy(1e-2);%Note this should be 1e-6 for research
%Set all muscles to 2% activation to represent passive state
forsim.set_constant_muscle_control(0.02); 
forsim.set_ignore_activation_dynamics(true);
forsim.set_ignore_tendon_compliance(true);
forsim.set_unconstrained_coordinates(0,'/jointset/knee_r/knee_add_r');
forsim.set_unconstrained_coordinates(1,'/jointset/knee_r/knee_rot_r');
forsim.set_unconstrained_coordinates(2,'/jointset/knee_r/knee_tx_r');
forsim.set_unconstrained_coordinates(3,'/jointset/knee_r/knee_ty_r');
forsim.set_unconstrained_coordinates(4,'/jointset/knee_r/knee_tz_r');
forsim.set_unconstrained_coordinates(5,'/jointset/pf_r/pf_flex_r');
forsim.set_unconstrained_coordinates(6,'/jointset/pf_r/pf_rot_r');
forsim.set_unconstrained_coordinates(7,'/jointset/pf_r/pf_tilt_r');
forsim.set_unconstrained_coordinates(8,'/jointset/pf_r/pf_tx_r');
forsim.set_unconstrained_coordinates(9,'/jointset/pf_r/pf_ty_r');
forsim.set_unconstrained_coordinates(10,'/jointset/pf_r/pf_tz_r');
forsim.set_prescribed_coordinates_file(prescribed_coordinates_file);
forsim.set_external_loads_file('external_loads.xml');%external_loads_xml_file);
forsim.set_use_visualizer(true);
forsim.print('./inputs/healthy_forsim_settings.xml');


disp('Running Forsim Tool...')
forsim.run();

% ACL Deficient 
acld_forsim_result_dir = './results/acld_forsim';
acld_basename = 'acld';

forsim.setModel(acld_model);
forsim.set_results_directory(acld_forsim_result_dir);
forsim.set_results_file_basename(acld_basename);
forsim.print('./inputs/acld_forsim_settings.xml');

disp('Running Forsim Tool...')
forsim.run();
%% Perform Analysis with JointMechanicsTool
%Healthy
healthy_jnt_mech_result_dir = './results/healthy_joint_mechanics';

jnt_mech = JointMechanicsTool();
jnt_mech.setModel(model);
jnt_mech.set_input_states_file(...
    [healthy_forsim_result_dir '/' healthy_basename '_states.sto']);
jnt_mech.set_results_file_basename(healthy_basename);
jnt_mech.set_results_directory(healthy_jnt_mech_result_dir);
jnt_mech.set_start_time(-1);
jnt_mech.set_stop_time(-1);
jnt_mech.set_normalize_to_cycle(false);
jnt_mech.set_contacts(0,'all');
jnt_mech.set_ligaments(0,'all');
jnt_mech.set_muscles(0,'none');
jnt_mech.set_attached_geometry_bodies(0,'/bodyset/femur_distal_r');
jnt_mech.set_attached_geometry_bodies(1,'/bodyset/tibia_proximal_r');
jnt_mech.set_attached_geometry_bodies(2,'/bodyset/patella_r');
jnt_mech.set_output_orientation_frame('ground');
jnt_mech.set_output_position_frame('ground');
jnt_mech.set_write_vtp_files(true);
jnt_mech.set_write_h5_file(true);
jnt_mech.set_h5_kinematics_data(true);
jnt_mech.set_h5_states_data(true);
jnt_mech.print('./inputs/healthy_joint_mechanics_settings.xml');

% ACL Deficient 
jnt_mech.setModel(acld_model);
jnt_mech.set_input_states_file(...
    [acld_forsim_result_dir '/' acld_basename '_states.sto']);
jnt_mech.set_results_file_basename(acld_basename);
jnt_mech.set_results_directory(acld_jnt_mech_result_dir);
jnt_mech.print('./inputs/acld_joint_mechanics_settings.xml');


disp('Running JointMechanicsTool...');
jnt_mech.run();

