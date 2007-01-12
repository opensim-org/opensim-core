function jntangles = transform_jntanglesToModelCS(cSim, ...
                                        corrected_kAxisAngles, backAngles)
% Purpose:  Performs the following computations:
%           (1) transforms joint angles in cSim, read from the C3D file of
%               a Gillette subject, to a coordinate system consistent with 
%               Darryl's model.  
%           (2) transforms joint angles (i.e., hip rotation, knee flexion)
%               corrected for kAxis malalignment to a coordinate system 
%               consistent with Darryl's model.
%           (3) appends back angles (already in the model CS)
%               to output structure.
%           (4) appends zero arrays for the subtalar and mtp joints.
%
% Input:    cSim is a structure that contains the following data, 
%             in addition to other information:
%                   *.video        .rate,  .nframes, .units
%                   .jntangles     .L(),   .label, .data
%                                  .R(),   .label, .data
%           corrected_kAxisAngles is a structure containing the
%             'corrected' knee flex/ext, knee var/val, knee rotation,
%              and hip rotation angles, in this format:
%               *.R     .kf(nVideoFrames of 'simulateable' segment)
%               *.L     .kv(")
%                       .kr(")
%                       .hr(")
%                       .kAxisOffset
%           backAngles returns a structure containing the angles between 
%             the model's pelvis and trunk segments, as determined from 
%             the subject's marker data, in the following format:
%                  *.extension(nVideoFrames of 'simulateable' segment)
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
jntangles.lumbar_extension = backAngles.extension;
jntangles.lumbar_bending = backAngles.bending;
jntangles.lumbar_rotation = backAngles.rotation;

% Transform and store pelvis angles.
pelvisAngles = extract_jntangleData(cSim, 'RPelvisAngles', 'R');
jntangles.pelvis_tilt = -1*pelvisAngles(:, 1) + 12;
jntangles.pelvis_list = -1*pelvisAngles(:, 2);
jntangles.pelvis_rotation = pelvisAngles(:, 3);

% Transform and store hip angles.
rHipAngles = extract_jntangleData(cSim, 'RHipAngles', 'R');
lHipAngles = extract_jntangleData(cSim, 'LHipAngles', 'L');
jntangles.hip_flexion_r = rHipAngles(:, 1) - 12;
jntangles.hip_adduction_r = rHipAngles(:, 2);
jntangles.hip_rotation_r = corrected_kAxisAngles.R.hr';
jntangles.hip_flexion_l = lHipAngles(:, 1) - 12;
jntangles.hip_adduction_l = lHipAngles(:, 2);
jntangles.hip_rotation_l = corrected_kAxisAngles.L.hr';

% Transform and store knee angles.
jntangles.knee_angle_r = -1*corrected_kAxisAngles.R.kf';
jntangles.knee_angle_l = -1*corrected_kAxisAngles.L.kf';

% Transform and store ankle angles; add subtalar and mpt angles.
rAnkleAngles = extract_jntangleData(cSim, 'RAnkleAngles', 'R');
lAnkleAngles = extract_jntangleData(cSim, 'LAnkleAngles', 'L');
jntangles.ankle_angle_r = rAnkleAngles(:, 1);
jntangles.ankle_angle_l = lAnkleAngles(:, 1);
jntangles.subtalar_angle_r = zeros(cSim.video.nframes, 1);
jntangles.subtalar_angle_l = zeros(cSim.video.nframes, 1);
jntangles.mtp_angle_r = zeros(cSim.video.nframes, 1);
jntangles.mtp_angle_l = zeros(cSim.video.nframes, 1);
return;
