function jntangles = transform_staticAnglesToModelCS(cStatic, backStatic)
% Purpose:  Performs the following computations:
%           (1) transforms joint angles in cStatic, read from the C3D file 
%               of a static trial of a Gillette subject, to a 
%               coordinate system consistent with Darryl's model.  
%           (2) appends back angles (already in the model CS)
%               to output structure.
%           (3) appends zero arrays for the subtalar and mtp joints.
%
% Input:    cStatic is a structure that contains the following data, 
%             in addition to other information:
%                  *.video        .rate,  .nframes, .units
%                   .jntangles     .L(),   .label, .data
%                                  .R(),   .label, .data
%           backStatic returns a structure containing the angles between 
%             the model's pelvis and trunk segments, as determined from 
%             the subject's marker data, in the following format:
%                  *.extension(nVideoFrames)
%                   .bending(")
%                   .rotation(")
%
% Output:   jntangles returns a structure containing the transformed 
%             jntangle trajectories, with the following format:
%               *.dataName(nVideoFrames), where dataName corresponds to
%                   each DOF in the model.
%    
% Called Functions:
%       extract_jntangleData(c, abbr, limb)
%
% ASA, 10-05


% Store back angles.
jntangles.lumbar_extension = backStatic.extension;
jntangles.lumbar_bending = backStatic.bending;
jntangles.lumbar_rotation = backStatic.rotation;

% Transform and store pelvis angles.
pelvisAngles = extract_jntangleData(cStatic, 'RPelvisAngles', 'R');
jntangles.pelvis_tilt = -1*pelvisAngles(:, 1) + 12;
jntangles.pelvis_list = -1*pelvisAngles(:, 2);
jntangles.pelvis_rotation = pelvisAngles(:, 3);

% Transform and store hip angles.
rHipAngles = extract_jntangleData(cStatic, 'RHipAngles', 'R');
lHipAngles = extract_jntangleData(cStatic, 'LHipAngles', 'L');
jntangles.hip_flexion_r = rHipAngles(:, 1) - 12;
jntangles.hip_adduction_r = rHipAngles(:, 2);
jntangles.hip_rotation_r = rHipAngles(:, 3);
jntangles.hip_flexion_l = lHipAngles(:, 1) - 12;
jntangles.hip_adduction_l = lHipAngles(:, 2);
jntangles.hip_rotation_l = lHipAngles(:, 3);

% Transform and store knee angles.
rKneeAngles = extract_jntangleData(cStatic, 'RKneeAngles', 'R');
lKneeAngles = extract_jntangleData(cStatic, 'LKneeAngles', 'L');
jntangles.knee_angle_r = -1*rKneeAngles(:, 1);
jntangles.knee_angle_l = -1*lKneeAngles(:, 1);

% Transform and store ankle angles; add subtalar and mpt angles.
rAnkleAngles = extract_jntangleData(cStatic, 'RAnkleAngles', 'R');
lAnkleAngles = extract_jntangleData(cStatic, 'LAnkleAngles', 'L');
jntangles.ankle_angle_r = rAnkleAngles(:, 1);
jntangles.ankle_angle_l = lAnkleAngles(:, 1);
jntangles.subtalar_angle_r = zeros(cStatic.video.nframes, 1);
jntangles.subtalar_angle_l = zeros(cStatic.video.nframes, 1);
jntangles.mtp_angle_r = zeros(cStatic.video.nframes, 1);
jntangles.mtp_angle_l = zeros(cStatic.video.nframes, 1);
return;
