function measured_kAxisAngles = get_kAxisAngles(cSim)
% Purpose:  Extracts knee flex/ext, knee var/val, knee rotation, and
%           hip rotation angles from cSim, a 'simulateable' segment
%           read from the C3D file of a Gillette control subject; 
%           these data are needed to estimate and correct for 
%           malalignment of the knee flex/ext axis.
%
% Input:    cSim is a structure that contains the following data, in 
%             addition to other information:
%               cSim.video                 .rate,  .nframes
%               cSim.jntangles             .R(),   .label, .data
%                                          .L(),   .label, .data 
%
% Output:   measured_kAxisAngles returns a structure containing the
%           'measured' knee flex/ext, knee var/val, knee rotation,
%           and hip rotation angles, in degrees:
%               *.R     .kf(nVideoFrames of 'simulateable' segment)
%               *.L     .kv(")
%                       .kr(")
%                       .hr(")
%
% ASA, 10-05


% Get lists of labels corresponding to the R and L joint angles in cSim.
nAngles = length(cSim.jntangles.R);
for jntangleIndex = 1:nAngles
    RjaLabels{jntangleIndex} = cSim.jntangles.R(jntangleIndex).label;
    LjaLabels{jntangleIndex} = cSim.jntangles.L(jntangleIndex).label;
end

% Extract relevant joint angles.
% NOTE: use '*Angles' for angles determined by Vicon;
%       use '*AnglesDJC' for angles determined by dynamic joint centers'
rKneeIndex = strmatch('RKneeAngles', RjaLabels, 'exact');
rHipIndex = strmatch('RHipAngles', RjaLabels, 'exact');
lKneeIndex = strmatch('LKneeAngles', LjaLabels, 'exact');
lHipIndex = strmatch('LHipAngles', LjaLabels, 'exact');
kfColIndex = 1;         % column index corresponding to knee flex/ext
kvColIndex = 2;         % column index corresponding to knee var/val
krColIndex = 3;         % column index corresponding to knee rotation
hrColIndex = 3;         % column index corresponding to hip rotation

% NOTE:  The C3D Server returns cell arrays;
%        these arrays are converted to numeric arrays.
for vframeNum = 1:cSim.video.nframes
    measured_kAxisAngles.R.kf(vframeNum) = ...
        cSim.jntangles.R(rKneeIndex).data{vframeNum, kfColIndex}; 
    measured_kAxisAngles.R.kv(vframeNum) = ...
        cSim.jntangles.R(rKneeIndex).data{vframeNum, kvColIndex}; 
    measured_kAxisAngles.R.kr(vframeNum) = ...
        cSim.jntangles.R(rKneeIndex).data{vframeNum, krColIndex}; 
    measured_kAxisAngles.R.hr(vframeNum) = ...
        cSim.jntangles.R(rHipIndex).data{vframeNum, hrColIndex}; 
    
    measured_kAxisAngles.L.kf(vframeNum) = ...
        cSim.jntangles.L(lKneeIndex).data{vframeNum, kfColIndex}; 
    measured_kAxisAngles.L.kv(vframeNum) = ...
        cSim.jntangles.L(lKneeIndex).data{vframeNum, kvColIndex}; 
    measured_kAxisAngles.L.kr(vframeNum) = ...
        cSim.jntangles.L(lKneeIndex).data{vframeNum, krColIndex}; 
    measured_kAxisAngles.L.hr(vframeNum) = ...
        cSim.jntangles.L(lHipIndex).data{vframeNum, hrColIndex}; 
end
return;
