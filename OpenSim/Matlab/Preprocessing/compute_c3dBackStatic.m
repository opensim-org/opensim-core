function [backStatic, trunk_rzStatic] = ...
                            compute_c3dBackStatic(cStatic, figHandle)
% Purpose:  Performs the following functions:
%           (1) determines the orientation of the trunk relative to the   
%               pelvis from the marker data in cStatic, extracted from the 
%               C3D file of a static trial of a Gillette control subject, 
%           (2) performs the transformations necessary to describe these 
%               angles as rotations between the pelvis and trunk segment
%               coordinate systems of Darryl's model.
%
%           To do this, need to determine the RotZ, RotX, RotY such that
%           R_trunkmodelNpelvismodel = [RotZ]*[RotX]*[RotY], where
%           R_trunkmodelNpelvismodel = R_pelvismarkersNpelvismodel
%                                       * R_labNpelvismarkers
%                                       * R_trunkmarkersNlab
%                                       * R_trunkmodelNtrunkmarkers
%           Notation:  R_trunkmodelNpelvismodel describes the orientation
%                      of the trunk model reference frame with respect to
%                      the pelvis model reference frame.
%                      ("trunk IN pelvis frame")
%
% Notes:    1.  This function is interactive; the user is allowed to
%               tweak the assumed orientation of the axis defined by
%               C7 and the sternum in the sagittal plane, so that the
%               back extension in the upright standing position is
%               ~ -1*mean(pelvic tilt).   This value is plotted for
%               reference.
%
%           2.  Since the sternal notch and xyphoid markers are often not 
%               labelled in static trials from Gillette control subjects,
%               this function was revised (11-18-05) to avoid using
%               these markers when the static trial is processed.
%
% Input:    cStatic is a structure that contains the following data, in 
%             addition to other information:
%                   *.trial       .num,   .type 
%                   *.video       .rate,  .nframes
%                   *.markers()   .label, .data
%           figHandle is the number of the figure window to be generated
%
% Output:   backStatic returns a structure containing the angles between 
%             the model's pelvis and trunk segments, as determined from 
%             the subject's marker data, in the following format:
%                  *.extension(nVideoFrames) = -1 * pelvic tilt,
%                                               assuming upright standing
%                   .bending(")
%                   .rotation(")
%           trunk_rzStatic returns the user-defined R(z) angle that puts 
%            the model in the upright standing position in the static trial.  
%            To apply the correction, set 
%               R_trunkmarkersNtrunkmodel = ...
%                           get_RfromKtheta([0 0 1], trunk_rzStatic)
%
% Called Functions:
%       extract_markerCoords(c, abbr)
%       extract_jntangleData(c, abbr, limb)
%       get_sternumAngle(trunkMkrs, source)
%       get_pelvisCSfromAsisPsis(landmarks)
%       get_trunkCSfromSternumC7(landmarks)
%       get_RzxyAngles(R)
%       plot_backAngles(backAngles, pelvAnglesNmodel, sternumAngles, ...
%                                       trunk_rz, vTime, figHandle);
%       get_RfromKtheta(K, theta_deg)
%       Suptitle(titleString)
%
% ASA, 10-05


%%% PROCESS EXPERIMENTAL DATA
% Extract measured trajectories of relevant markers.
% NOTES:  Marker abbreviations must correspond to the C3D file;
%         these abbreviations are valid for Gillette control subjects, 
%         but may need to be changed for CP subjects. 
expt_pelvmkrs.rASIS = extract_markerCoords(cStatic, 'RASI');
expt_pelvmkrs.lASIS = extract_markerCoords(cStatic, 'LASI');
expt_pelvmkrs.rPSIS = extract_markerCoords(cStatic, 'RPSI');
expt_pelvmkrs.lPSIS = extract_markerCoords(cStatic, 'LPSI');
expt_trunkmkrs.rSubclav = extract_markerCoords(cStatic, 'RAC');
expt_trunkmkrs.lSubclav = extract_markerCoords(cStatic, 'LAC');
expt_trunkmkrs.neck = extract_markerCoords(cStatic, 'N');
expt_trunkmkrs.sternalnotch = ...
            (expt_trunkmkrs.rSubclav + expt_trunkmkrs.lSubclav)/2.0;

% Extract experimentally-determined pelvis joint angles, for reference,
% and convert to model CS.
% NOTES: (1) Pelvis angles corresponding to the R limb are extracted,
%            since the sign convention for the R pelvis is identical to 
%            the sign convention of the back in the model. 
%        (2) Angles are are returned in an array in the order given in the
%            C3D file; for Gillette control subjects, the order is:
%            tilt, obliquity, and rotation.
expt_pelvAngles = extract_jntangleData(cStatic, 'RPelvisAngles', 'R');
pelvAnglesNmodel.tilt = -1*expt_pelvAngles(:, 1) + 12;
pelvAnglesNmodel.obliquity = -1*expt_pelvAngles(:, 2);
pelvAnglesNmodel.rotation = expt_pelvAngles(:, 3);

% Define array of time values corresponding to the static trial.
vFrames = 1:1:cStatic.video.nframes;
vTime = vFrames/cStatic.video.rate;

% NOTE: 
% The function plot_backAngles() requires a structure called sternumAngles
% as an input argument.  This is the angle in the sagittal plane between
% the vertical axis and the vector(sternal_notch - xyphoid).  Since I 
% discovered after writing this code that the xyphoid marker is often 
% unavailable for static trials from Gillette, I'm setting
% sternumAngles.expt == 0 for now ...
sternumAngles.expt = zeros(1, length(vTime));

%%% PROCESS MODEL
% Specify coordinates of relevant markers as defined in the model.  
% NOTE:  Coordinates from gait23_gilletteControls.jnt
model_pelvmkrs.rASIS = [0.0140 0.0000 0.1280];  
model_pelvmkrs.lASIS = [0.0140 0.0000 -0.1280]; 
model_pelvmkrs.rPSIS = [-0.158 0.037853 0.055989];
model_pelvmkrs.lPSIS = [-0.158 0.037853 -0.055989];
model_trunkmkrs.rSubclav = [0.030 0.380 0.0675]; 
model_trunkmkrs.lSubclav = [0.030 0.380 -0.0675]; 
model_trunkmkrs.sternalnotch = [0.030 0.380 0.000];
model_trunkmkrs.xyphoid = [0.0975 0.230 0.000];
model_trunkmkrs.neck = [-0.075 0.4225 0.000];

% Get constant rotation matrix R_pelvismarkersNpelvismodel, which
% describes the pelvis marker CS in the model's pelvis CS.
R_pelvismarkersNpelvismodel = get_pelvisCSfromAsisPsis(model_pelvmkrs);
[pelvis_rz, pelvis_rx, pelvis_ry] = get_RzxyAngles(R_pelvismarkersNpelvismodel);

% Get constant rotation matrix R_trunkmarkersNtrunkmodel, which
% describes the trunk marker CS in the model's trunk CS.
R_trunkmarkersNtrunkmodel = get_trunkCSfromSternumC7(model_trunkmkrs);
[trunk_rz, trunk_rx, trunk_ry] = get_RzxyAngles(R_trunkmarkersNtrunkmodel);

% For reference, get angle in sagittal plane, between vertical axis and the
% vector (sternal_notch - xyphoid) for the model.
model_sternumVector = model_trunkmkrs.sternalnotch - model_trunkmkrs.xyphoid;
sternumAngles.model = get_sternumAngle(model_sternumVector, 'model');

% Compute back angles at each video frame and plot the back angles and
% measured pelvis angles in the model CS.
done = 0;
while ~done
    
    % Get back angles at each video frame.
    for vframeNum = 1:cStatic.video.nframes
        pelvmkrs.rASIS = expt_pelvmkrs.rASIS(vframeNum, :);
        pelvmkrs.lASIS = expt_pelvmkrs.lASIS(vframeNum, :);
        pelvmkrs.rPSIS = expt_pelvmkrs.rPSIS(vframeNum, :);
        pelvmkrs.lPSIS = expt_pelvmkrs.lPSIS(vframeNum, :);
        R_pelvismarkersNlab = ...
            get_pelvisCSfromAsisPsis(pelvmkrs);
        
        trunkmkrs.rSubclav = expt_trunkmkrs.rSubclav(vframeNum, :);
        trunkmkrs.lSubclav = expt_trunkmkrs.lSubclav(vframeNum, :);
         trunkmkrs.sternalnotch = expt_trunkmkrs.sternalnotch(vframeNum, :);
        trunkmkrs.neck = expt_trunkmkrs.neck(vframeNum, :);
        R_trunkmarkersNlab = ...
            get_trunkCSfromSternumC7(trunkmkrs);
        
        R_trunkmodelNpelvismodel = R_pelvismarkersNpelvismodel ...
                                 * R_pelvismarkersNlab' ...
                                 * R_trunkmarkersNlab ...
                                 * R_trunkmarkersNtrunkmodel';            
        [rzNmodel(vframeNum), rxNmodel(vframeNum), ...
           ryNmodel(vframeNum)] = get_RzxyAngles(R_trunkmodelNpelvismodel);          
    end
    
    % Store back angles.
    backStatic.extension = rzNmodel';
    backStatic.bending = rxNmodel';
    backStatic.rotation = ryNmodel';
            
    % Generate plot and add title.
    plot_backAngles(backStatic, pelvAnglesNmodel, sternumAngles, ...
                           trunk_rz, vTime, figHandle);
    titleString = sprintf('%s%s%s%s', ...
        'Back, R Pelvis, and Sternum Angles in Model CS:  Subject ', ...
        char(cStatic.subject), '-', char(cStatic.trial.num));
    Suptitle(titleString);    % MATLAB m-file for adding "super title"

    % Query user.
    query = 'Adjust back extension?';
    opt1 = 'tilt anteriorly';
    opt2 = 'tilt posteriorly';
    opt3 = 'print';
    opt4 = 'done';
    userInput = menu(query, opt1, opt2, opt3, opt4);
    switch userInput
        case 1
            sternumAngles.model = sternumAngles.model - 1;
            trunk_rz = trunk_rz + 1;
            R_trunkmarkersNtrunkmodel = ...
                get_RfromKtheta([0 0 1], trunk_rz);
        case 2
            sternumAngles.model = sternumAngles.model + 1;
            trunk_rz = trunk_rz - 1;
            R_trunkmarkersNtrunkmodel = ...            
                get_RfromKtheta([0 0 1], trunk_rz);
        case 3
            orient(figHandle, 'landscape');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
        case 4
            done = 1;
    end
end

trunk_rzStatic = trunk_rz;
return;
