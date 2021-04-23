import org.opensim.modeling.*
Logger.setLevelString('info');

num_simulations = 3;

% Load model
model_file = '../models/healthy/lenhart2015/lenhart2015.osim';

model = Model(model_file);

state = model.initSystem();


if(exist('./inputs','dir')~=7)
    mkdir('./inputs')
end

if(exist('./inputs/shared','dir')~=7)
    mkdir('./inputs/shared')
end
if(exist('./inputs/shared/Geometry','dir')~=7)
    copyfile('../models/healthy/current/Geometry','./inputs/shared/Geometry')
end

for i = 0:num_simulations-1
    dir_name = ['./inputs/' int2str(i)];
    if(exist(dir_name,'dir')~=7)
        mkdir(dir_name)
    end
end




% Get ACLpl1 ligament
ACLpl1 = Blankevoort1991Ligament.safeDownCast(model.getComponent('/forceset/ACLpl1'));

% Set Stiffness
% ACLpl1.get_linear_stiffness();
 ACLpl1.set_linear_stiffness(333);
 

% Set Slack Length 
% ACLpl1.get_slack_length();
% ACLpl1.set_slack_length();
ACLpl1.setSlackLengthFromReferenceStrain(0.02,state);
%ACLpl1.setSlackLengthFromReferenceForce(20,state)

% Move attachment point
ACLp1_pt0 = PathPoint.safeDownCast(ACLpl1.get_GeometryPath().getPathPointSet().get(0));
ACLp1_pt0.set_location(Vec3(1.0,2.0,3.0));

% Add wrap object
mcl_wrap = WrapEllipsoid();
mcl_wrap.setName('mcl_wrap')
mcl_wrap.set_active(true);
mcl_wrap.set_dimensions(Vec3(0.03, 0.018, 0.012));
mcl_wrap.set_translation(Vec3(0.001, -0.0346, -0.0187));
mcl_wrap.set_xyz_body_rotation(Vec3(-0.105, -0.035, 0.182));
mcl_wrap.set_quadrant('all');
model.getBodySet().get('tibia_proximal_r').addWrapObject(mcl_wrap);

% Add new ligament
MCL1_pt1 = Vec3(0.0115, 0.0075, -0.0405);
MCL1_pt2 = Vec3(0.0149, -0.0618, -0.0153);
MCL1 = Blankevoort1991Ligament('MCL1',...
    model.getBodySet().get('femur_distal_r'),MCL1_pt1,...
    model.getBodySet().get('tibia_proximal_r'),MCL1_pt2);
model.addForce(MCL1);
MCL1.get_GeometryPath().addPathWrap(mcl_wrap);
MCL1.set_linear_stiffness(300);
MCL1.set_slack_length(0.05);

model.finalizeConnections();
model.print('./inputs/0/model.osim'),
