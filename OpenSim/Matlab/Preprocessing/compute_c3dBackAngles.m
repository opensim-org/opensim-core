function backAngles = compute_c3dBackAngles(cSim, tInfo, figHandle, varargin)
% Purpose:  Determines the orientation of the trunk relative to the pelvis  
%           from the marker data in cSim, extracted from the C3D file of a
%           Gillette control subject, and performs the transformations
%           necessary to describe these angles as rotations between the
%           pelvis and trunk segment coordinate systems of Darryl's model.
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
%           NOTE:  This function is interactive; the user is allowed to
%                  tweak the assumed orientation of the axis defined by
%                  C7 and the sternum in the sagittal plane.
%
% Input:    cSim is a structure that contains the following data, in 
%             addition to other information:
%                   *.video       .rate,  .nframes
%                   *.markers()   .label, .data
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.trial - trial number to analyze ('character array')
%               *.speed - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%           figHandle is the number of the figure window to be generated
%           varargin specifies an optional argument, trunk_rzStatic,
%             which is returned from compute_c3dBackStatic if a static
%             trial is available. 
%
% Output:   backAngles returns a structure containing the angles between 
%             the model's pelvis and trunk segments, as determined from 
%             the subject's marker data, in the following format:
%                  *.extension(nVideoFrames of 'simulateable' segment)
%                   .bending(")
%                   .rotation(")
%
% Called Functions:
%       extract_markerCoords(c, abbr)
%       extract_jntangleData(c, abbr, limb)
%       get_sternumAngle(trunkMkrs, source)
%       get_pelvisCSfromAsisPsis(landmarks)
%       get_trunkCSfromSternumC7(landmarks)
%       get_RzxyAngles(R)
%       plot_backAngles(backAngles, pelvAnglesNmodel, sternumAngles, ...
%                                             trunk_rz, vTime, figHandle);
%       get_RfromKtheta(K, theta_deg)
%       Suptitle(titleString)
%
% ASA, 10-05


%%% PROCESS EXPERIMENTAL DATA
% Extract measured trajectories of relevant markers.
% NOTES:  Marker abbreviations must correspond to the C3D file;
%         these abbreviations are valid for Gillette control subjects, 
%         but may need to be changed for CP subjects. 
expt_pelvmkrs.rASIS = extract_markerCoords(cSim, 'RASI');
expt_pelvmkrs.lASIS = extract_markerCoords(cSim, 'LASI');
expt_pelvmkrs.rPSIS = extract_markerCoords(cSim, 'RPSI');
expt_pelvmkrs.lPSIS = extract_markerCoords(cSim, 'LPSI');
expt_trunkmkrs.rSubclav = extract_markerCoords(cSim, 'RAC');
expt_trunkmkrs.lSubclav = extract_markerCoords(cSim, 'LAC');
expt_trunkmkrs.sternalnotch = extract_markerCoords(cSim, 'STRN');
expt_trunkmkrs.xyphoid = extract_markerCoords(cSim, 'XYPH');
expt_trunkmkrs.neck = extract_markerCoords(cSim, 'N');

% Extract experimentally-determined pelvis joint angles, for reference,
% and convert to model CS.
% NOTES: (1) Pelvis angles corresponding to the R limb are extracted,
%            since the sign convention for the R pelvis is identical to 
%            the sign convention of the back in the model. 
%        (2) Angles are are returned in an array in the order given in the
%            C3D file; for Gillette control subjects, the order is:
%            tilt, obliquity, and rotation.
expt_pelvAngles = extract_jntangleData(cSim, 'RPelvisAngles', 'R');
pelvAnglesNmodel.tilt = -1*expt_pelvAngles(:, 1) + 12;
pelvAnglesNmodel.obliquity = -1*expt_pelvAngles(:, 2);
pelvAnglesNmodel.rotation = expt_pelvAngles(:, 3);

% For reference, get angle in sagittal plane, between vertical axis and the
% vector (sternal_notch - xyphoid) for the experimental data.
% NOTE: need to determine whether subject faced +X or -X direction.
expt_sternumVectors = expt_trunkmkrs.sternalnotch - expt_trunkmkrs.xyphoid;
if expt_trunkmkrs.sternalnotch(1, 1) > expt_trunkmkrs.neck(1, 1)
    sternumAngles.expt = ...
        get_sternumAngle(expt_sternumVectors, 'gillettePosX');
else
    sternumAngles.expt = ...
        get_sternumAngle(expt_sternumVectors, 'gilletteNegX');
end

% Define array of time values corresponding to the 'simulateable' data.
% Set t = 0 at IC of the 1st FP strike; note that the data starts at OTO, 
% which occurs at cSim.tDS seconds later.
vFrames = 1:1:cSim.video.nframes;
vTime = vFrames/cSim.video.rate + cSim.tDS;


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
if isempty(varargin)
    R_trunkmarkersNtrunkmodel = get_trunkCSfromSternumC7(model_trunkmkrs);
    [trunk_rz, trunk_rx, trunk_ry] = get_RzxyAngles(R_trunkmarkersNtrunkmodel);
else
    trunk_rz = varargin{1};
    R_trunkmarkersNtrunkmodel = get_RfromKtheta([0 0 1], trunk_rz);
end

% For reference, get angle in sagittal plane, between vertical axis and the
% vector (sternal_notch - xyphoid) for the model.
model_sternumVector = model_trunkmkrs.sternalnotch - model_trunkmkrs.xyphoid;
sternumAngles.model = get_sternumAngle(model_sternumVector, 'model');

% Compute back angles at each video frame and plot the back angles and
% measured pelvis angles in the model CS; also plot the computed and
% measured "sternum" angles, for reference.
% NOTE:  Since the relative positions of the markers on the model may be
%        different than the positions of the markers on the subject, the
%        user is allowed to tweak the assumed orientation of the model's
%        trunk axis defined by markers on the sternal notch and zyphoid,
%        in the sagittal plane.
done = 0;
while ~done
    
    % Get back angles at each video frame.
    for vframeNum = 1:cSim.video.nframes
        pelvmkrs.rASIS = expt_pelvmkrs.rASIS(vframeNum, :);
        pelvmkrs.lASIS = expt_pelvmkrs.lASIS(vframeNum, :);
        pelvmkrs.rPSIS = expt_pelvmkrs.rPSIS(vframeNum, :);
        pelvmkrs.lPSIS = expt_pelvmkrs.lPSIS(vframeNum, :);
        R_pelvismarkersNlab = ...
            get_pelvisCSfromAsisPsis(pelvmkrs);
        
        trunkmkrs.rSubclav = expt_trunkmkrs.rSubclav(vframeNum, :);
        trunkmkrs.lSubclav = expt_trunkmkrs.lSubclav(vframeNum, :);
        trunkmkrs.sternalnotch = expt_trunkmkrs.sternalnotch(vframeNum, :);
        trunkmkrs.xyphoid = expt_trunkmkrs.xyphoid(vframeNum, :);
        trunkmkrs.neck = expt_trunkmkrs.neck(vframeNum, :);
        R_trunkmarkersNlab = ...
            get_trunkCSfromSternumC7(trunkmkrs);
        
        R_trunkmodelNpelvismodel = R_pelvismarkersNpelvismodel ...
                                 * R_pelvismarkersNlab' ...
                                 * R_trunkmarkersNlab ...
                                 * R_trunkmarkersNtrunkmodel';            
      [rzNmodel(vframeNum), rxNmodel(vframeNum), ryNmodel(vframeNum)] = ...
            get_RzxyAngles(R_trunkmodelNpelvismodel);
    end
    
    % Store back angles.
    backAngles.extension = rzNmodel';
    backAngles.bending = rxNmodel';
    backAngles.rotation = ryNmodel';
        
    % Generate plot and add title.
    plot_backAngles(backAngles, pelvAnglesNmodel, sternumAngles, ...
                                          trunk_rz, vTime, figHandle);
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Back, R Pelvis, and Sternum Angles in Model CS:  Subject ', ...
        char(cSim.subject), '-', tInfo.trial, ', Speed ', ...
        tInfo.speed, ' m/s');
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
return;