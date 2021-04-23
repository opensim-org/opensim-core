%% Setup Environment and Folders
import org.opensim.modeling.*
Logger.setLevelString('Info');

if(exist('./inputs','dir')~=7)
    mkdir('./inputs')
end
if(exist('./inputs/Geometry','dir')~=7)
    mkdir('./inputs/Geometry')
end
if(exist('./results','dir')~=7)
    mkdir('./results')
end
if(exist('./results/passive','dir')~=7)
    mkdir('./results/passive')
end
if(exist('./results/graphics','dir')~=7)
    mkdir('./results/graphics')
end



%% Build Model

% Copy Geometries to example dir
geometry_dir = '../models/healthy/lenhart2015/Geometry/';
local_geometry_dir = './inputs/Geometry/';

femur_bone_mesh_file = 'lenhart2015-R-femur-bone.stl';
tibia_bone_mesh_file = 'lenhart2015-R-tibia-bone.stl';
patella_bone_mesh_file = 'lenhart2015-R-patella-bone.stl';
femur_cartilage_mesh_file = 'lenhart2015-R-femur-cartilage.stl';
tibia_cartilage_mesh_file = 'lenhart2015-R-tibia-cartilage.stl';
patella_cartilage_mesh_file = 'lenhart2015-R-patella-cartilage.stl';

copyfile([geometry_dir femur_bone_mesh_file],[local_geometry_dir femur_bone_mesh_file])
copyfile([geometry_dir tibia_bone_mesh_file],[local_geometry_dir tibia_bone_mesh_file])
copyfile([geometry_dir patella_bone_mesh_file],[local_geometry_dir patella_bone_mesh_file])

copyfile([geometry_dir femur_cartilage_mesh_file],[local_geometry_dir femur_cartilage_mesh_file])
copyfile([geometry_dir tibia_cartilage_mesh_file],[local_geometry_dir tibia_cartilage_mesh_file])
copyfile([geometry_dir patella_cartilage_mesh_file],[local_geometry_dir patella_cartilage_mesh_file])

ModelVisualizer.addDirToGeometrySearchPaths(local_geometry_dir);

model = Model();


% Add Bodies
% Mass properties are arbitrary as the model will be kinematically driven,
% so no dynamic simulation is performed
massCenter = Vec3(0.0);                       
inertia = Inertia(0.1, 0.1, 0.1, 0, 0, 0);

femur  = Body();
femur.setName('femur');
femur.setMass(1.0);                                      
femur.setMassCenter(massCenter);                        
femur.setInertia(inertia); 
femur.attachGeometry(Mesh(femur_bone_mesh_file));
model.addBody(femur);

tibia  = Body();
tibia.setName('tibia');
tibia.setMass(1.0);                                      
tibia.setMassCenter(massCenter);                        
tibia.setInertia(inertia);
tibia.attachGeometry(Mesh(tibia_bone_mesh_file));
model.addBody(tibia);

patella  = Body();
patella.setName('patella');
patella.setMass(1.0);                                      
patella.setMassCenter(massCenter);                        
patella.setInertia(inertia); 
patella.attachGeometry(Mesh(patella_bone_mesh_file));
model.addBody(patella);

%Add Joints

ground_transform = SpatialTransform();

ground_transform.get_rotation1().set_coordinates(0,'ground_femur_rz_r');
ground_transform.get_rotation1().set_axis(Vec3(0,0,1));
ground_transform.get_rotation1().set_function(LinearFunction());

ground_transform.get_rotation2().set_coordinates(0,'ground_femur_rx_r');
ground_transform.get_rotation2().set_axis(Vec3(1,0,0));
ground_transform.get_rotation2().set_function(LinearFunction());

ground_transform.get_rotation3().set_coordinates(0,'ground_femur_ry_r');
ground_transform.get_rotation3().set_axis(Vec3(0,1,0));
ground_transform.get_rotation3().set_function(LinearFunction());

ground_transform.get_translation1().set_coordinates(0,'ground_femur_tx_r');
ground_transform.get_translation1().set_axis(Vec3(1,0,0));
ground_transform.get_translation1().set_function(LinearFunction());

ground_transform.get_translation2().set_coordinates(0,'ground_femur_ty_r');
ground_transform.get_translation2().set_axis(Vec3(0,1,0));
ground_transform.get_translation2().set_function(LinearFunction());

ground_transform.get_translation3().set_coordinates(0,'ground_femur_tz_r');
ground_transform.get_translation3().set_axis(Vec3(0,0,1));
ground_transform.get_translation3().set_function(LinearFunction());

ground_femur_r = CustomJoint('ground_femur_r',...
    model.getGround(),Vec3(0.0),Vec3(0.0),femur,Vec3(0.0),Vec3(0.0),...
    ground_transform);

model.addJoint(ground_femur_r);

knee_transform = SpatialTransform();

knee_transform.get_rotation1().set_coordinates(0,'knee_flex_r');
knee_transform.get_rotation1().set_axis(Vec3(0,0,1));
knee_transform.get_rotation1().set_function(LinearFunction());

knee_transform.get_rotation2().set_coordinates(0,'knee_add_r');
knee_transform.get_rotation2().set_axis(Vec3(1,0,0));
knee_transform.get_rotation2().set_function(LinearFunction());

knee_transform.get_rotation3().set_coordinates(0,'knee_rot_r');
knee_transform.get_rotation3().set_axis(Vec3(0,1,0));
knee_transform.get_rotation3().set_function(LinearFunction());

knee_transform.get_translation1().set_coordinates(0,'knee_ant_r');
knee_transform.get_translation1().set_axis(Vec3(1,0,0));
knee_transform.get_translation1().set_function(LinearFunction());

knee_transform.get_translation2().set_coordinates(0,'knee_sup_r');
knee_transform.get_translation2().set_axis(Vec3(0,1,0));
knee_transform.get_translation2().set_function(LinearFunction());

knee_transform.get_translation3().set_coordinates(0,'knee_lat_r');
knee_transform.get_translation3().set_axis(Vec3(0,0,1));
knee_transform.get_translation3().set_function(LinearFunction());

knee_r = CustomJoint('knee_r',...
    femur,Vec3(0.0),Vec3(0.0),tibia,Vec3(0.0),Vec3(0.0),...
    knee_transform);

model.addJoint(knee_r);

pf_transform = SpatialTransform();

pf_transform.get_rotation1().set_coordinates(0,'pf_flex_r');
pf_transform.get_rotation1().set_axis(Vec3(0,0,1));
pf_transform.get_rotation1().set_function(LinearFunction());

pf_transform.get_rotation2().set_coordinates(0,'pf_rot_r');
pf_transform.get_rotation2().set_axis(Vec3(1,0,0));
pf_transform.get_rotation2().set_function(LinearFunction());

pf_transform.get_rotation3().set_coordinates(0,'pf_tilt_r');
pf_transform.get_rotation3().set_axis(Vec3(0,1,0));
pf_transform.get_rotation3().set_function(LinearFunction());

pf_transform.get_translation1().set_coordinates(0,'pf_ant_r');
pf_transform.get_translation1().set_axis(Vec3(1,0,0));
pf_transform.get_translation1().set_function(LinearFunction());

pf_transform.get_translation2().set_coordinates(0,'pf_sup_r');
pf_transform.get_translation2().set_axis(Vec3(0,1,0));
pf_transform.get_translation2().set_function(LinearFunction());

pf_transform.get_translation3().set_coordinates(0,'pf_lat_r');
pf_transform.get_translation3().set_axis(Vec3(0,0,1));
pf_transform.get_translation3().set_function(LinearFunction());

pf_r = CustomJoint('pf_r',...
    femur,Vec3(0.0),Vec3(0.0),patella,Vec3(0.0),Vec3(0.0),...
    pf_transform);

model.addJoint(pf_r);

femur_cnt_mesh = Smith2018ContactMesh('femur_cartilage',...
                    femur_cartilage_mesh_file,femur);
model.addContactGeometry(femur_cnt_mesh);
                

tibia_cnt_mesh = Smith2018ContactMesh('tibia_cartilage',...
                    tibia_cartilage_mesh_file,tibia);
model.addContactGeometry(tibia_cnt_mesh);


patella_cnt_mesh = Smith2018ContactMesh('patella_cartilage',...
                    patella_cartilage_mesh_file,patella);
model.addContactGeometry(patella_cnt_mesh);

tf_contact = Smith2018ArticularContactForce('knee_contact',...
                femur_cnt_mesh,tibia_cnt_mesh);
model.addForce(tf_contact);

pf_contact = Smith2018ArticularContactForce('pf_contact',...
                femur_cnt_mesh,patella_cnt_mesh);
model.addForce(pf_contact);

%Add MCL Ligament
mcl_wrap = WrapEllipsoid();
mcl_wrap.set_active(true);
mcl_wrap.set_dimensions(Vec3(0.03, 0.018, 0.012));
mcl_wrap.set_translation(Vec3(0.001, -0.0346, -0.0187));
mcl_wrap.set_xyz_body_rotation(Vec3(-0.105, -0.035, 0.182));
mcl_wrap.set_quadrant('all');
tibia.addWrapObject(mcl_wrap);

MCL1_pt1 = Vec3(0.0115, 0.0075, -0.0405);
MCL1_pt2 = Vec3(0.0149, -0.0618, -0.0153);
MCL1 = Blankevoort1991Ligament('MCL1',femur,MCL1_pt1,tibia,MCL1_pt2);
model.addForce(MCL1);
MCL1.get_GeometryPath().addPathWrap(mcl_wrap);

MCL2_pt1 = Vec3(0.0052, 0.0083, -0.0406);
MCL2_pt2 = Vec3(0.0076, -0.0611, -0.0191);
MCL2 = Blankevoort1991Ligament('MCL2',femur,MCL2_pt1,tibia,MCL2_pt2);
model.addForce(MCL2);
MCL2.get_GeometryPath().addPathWrap(mcl_wrap);

MCL3_pt1 = Vec3(-0.0004, 0.0072, -0.0388);
MCL3_pt2 = Vec3(0.0012, -0.0619, -0.0188);
MCL3 = Blankevoort1991Ligament('MCL3',femur,MCL3_pt1,tibia,MCL3_pt2);
model.addForce(MCL3);
MCL3.get_GeometryPath().addPathWrap(mcl_wrap);

state = model.initSystem();

MCL1.setSlackLengthFromReferenceStrain(-0.03,state);
MCL1.set_linear_stiffness(1000);
MCL2.setSlackLengthFromReferenceStrain(0.0,state);
MCL2.set_linear_stiffness(1000);
MCL3.setSlackLengthFromReferenceStrain(0.03,state);
MCL3.set_linear_stiffness(1000);

%Print model file
model_file = './inputs/knee.osim';
model.print(model_file);

%% Perform Analysis with JointMechanicsTool
basename = 'passive';
jnt_mech = JointMechanicsTool();
jnt_mech.set_model_file(model_file);
jnt_mech.set_input_states_file(...
    '../models/healthy/experimental_data/dynamic_mri/test_passive_coordinates.sto');
jnt_mech.set_results_file_basename('passive');
jnt_mech.set_results_directory('./results/passive/joint-mechanics');
jnt_mech.set_start_time(-1);
jnt_mech.set_stop_time(-1);
jnt_mech.set_normalize_to_cycle(false);
jnt_mech.set_contacts(0,'all');
jnt_mech.set_ligaments(0,'all');
jnt_mech.set_muscles(0,'none');
jnt_mech.set_attached_geometry_bodies(0,'/bodyset/femur');
jnt_mech.set_attached_geometry_bodies(1,'/bodyset/tibia');
jnt_mech.set_attached_geometry_bodies(2,'/bodyset/patella');
jnt_mech.set_output_orientation_frame('ground');
jnt_mech.set_output_position_frame('ground');
jnt_mech.set_write_vtp_files(true);
jnt_mech.set_write_h5_file(true);
jnt_mech.set_h5_kinematics_data(true);
jnt_mech.set_h5_states_data(true);
jnt_mech.set_use_visualizer(true);
jnt_mech.print(['./inputs/' basename '_joint_mechanics_settings.xml']);

disp('Running JointMechanicsTool...');
jnt_mech.run();


