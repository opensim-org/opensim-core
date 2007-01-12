function [] = prepro_GilletteControls(subject, tInfo, ref_dataFormat, fileName)
% Purpose:  Pre-processes experimental data, read from the C3D file of a 
%           Gillette control subject, and writes the .mot and .trc files
%           needed for input into the UWGait simulation workflow.
%
% Input:    subject is a 6-digit subject ID ('character array')
%           tInfo is a structure containing the following 'trial info':
%               *.trial - trial number to analyze ('character array')
%               *.mass  - mass of subject in kg
%               *.speed - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%               *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%               *.static - trial number of corresponding static trial
%                      (cell array: 1st cell = 'static trial #'
%                                   2nd cell = 'all' or 'min',
%                                   corresponding to available marker set
%
% Output:   If tInfo.static ~= {}, then
%           (1) the specified static trial is also processed
%               (i.e., in addition to the specified dynamic trial).
%           (2) both anatomical and tracking markers are processed, 
%               in addition to the joint centers.
%           (3) the following files are written to the current directory:
%                   subject_trial#.trc, subject_trial#.mot
%                   subject_staticTrial#.trc, subject_staticTrial#.mot
%
%           If tInfo.static == {}, then
%           (1) only the dynamic trial is processed
%               (i.e., no static trial is processed).
%           (2) only anatomical markers and joint centers 
%               (no tracking markers) are processed.
%           (3) the following files are written to the current directory:
%                   subject_trial#.trc, subject_trial#.mot            
%
% Called Functions:
%    read_c3dFile(fname)
%    extract_cStatic(c, startFrame)
%    compute_c3dBackStatic(cStatic, figHandle)
%    transform_staticAnglesToModelCS(cStatic, backStatic)
%    write_c3dTrcStaticAll(mStatic, cStatic, fname)
%    write_c3dTrcStaticMin(mStatic, cStatic, fname)
%    write_c3dMotStatic(mStatic, mkrMatrix, cStatic, fname)
%    get_ictoEvents(c, tInfo, figHandle)
%    extract_c3dSimData(c, ictoEvents)
%    process_c3dGRFs(cSim, tInfo, figHandleArray)
%    evaluate_c3dkAxisMalalignment(cSim, tInfo, ictoEvents, figHandleArray)
%    compute_c3dBackAngles(cSim, tInfo, figHandle, varargin)
%    transform_markersToModelCS(c, tInfo)
%    transform_jntcentersToModelCS(c, tInfo)
%    transform_jntanglesToModelCS(cSim, corrected_kAxisAngles, backAngles)
%    transform_forcesToModelCS(GRFTz_byLimb, COP_smoothed, cSim, tInfo)
%    write_c3dTrcFile(mCS, cSim, fname, contents)
%    write_c3dMotFile(mCS, mkrMatrix, fname)
%    write_c3dMotMoments(jntmoments, time, fname)
%
% ASA, 10-05

if nargin < 4
	fileName = '';
end

% Display status on screen.
status = sprintf('\n\n\n%s%s%s%s%s%3.2f%s', ...
            'Pre-Processing Gait Data for Subject ', subject, ...
            ', Trial ', tInfo.trial, ...
            ', Speed ', tInfo.speed, ' m/s');
disp(status);


%%% PROCESS STATIC TRIAL, IF INDICATED.
if ~isempty(tInfo.static)    
    
    % Read C3D file corresponding to static trial, 
    % and make sure file is type 'static'.
    fname = [subject, ' ', tInfo.static{1}, '.c3d'];
    c = read_c3dFile(fname, ref_dataFormat); 
    if ~strcmpi(c.trial.type, 'static')
        error('Incorrect static trial number; static trial not processed.')
    end
    
    % Delete first 10 video frames from static trial, 
    % to avoid bad marker data that is often present at beginning of trial.
    startFrame = 10;
    cStatic = extract_c3dStatic(c, startFrame);
    clear c;
    
    % Compute back angles in reference frames consistent with model;
    % determine rotation matrix to correct for discrepancies between
    % geometry/markers of subject and model.
    figHandle = 1;
    [backStatic, trunk_rzStatic] = ...
        compute_c3dBackStatic(cStatic, figHandle);

    % Transform data from the lab CS to the model CS.
    mStatic.markers = transform_markersToModelCS(cStatic, tInfo);
    mStatic.jntcenters = transform_jntcentersToModelCS(cStatic, tInfo);
    mStatic.jntangles = transform_staticAnglesToModelCS(cStatic, backStatic);
    
    % Define array of time values corresponding to the 'static' trial.
    vFrames = 1:1:cStatic.video.nframes;
    mStatic.time = vFrames/cStatic.video.rate';
    
    % Write static *.trc file.
    if strcmpi(tInfo.static{2}, 'all')
        fname = [subject, '_', tInfo.static{1}, '.trc'];  
        mkrMatrix = write_c3dTrcStaticAll(mStatic, cStatic, fname);  
    elseif strcmpi(tInfo.static{2}, 'min')
        % Add sternal notch marker, defined as midpoint between the
        % subclavicular markers.
        mStatic.markers.STRN = ...
            (mStatic.markers.RAC + mStatic.markers.LAC)/2.0;
        
        fname = [subject, '_', tInfo.static{1}, '.trc'];  
        mkrMatrix = write_c3dTrcStaticMin(mStatic, cStatic, fname);  
    end
    
    % Write static *.mot file.
    fname = [subject, '_', tInfo.static{1}, '.mot'];  
    write_c3dMotStatic(mStatic, mkrMatrix, cStatic, fname);  
end


%%% READ AND EXTRACT 'SIMULATEABLE' SEGMENT FROM C3D FILE (GAIT TRIAL)
%   NOTE:  cSim returns a structure with the following format:
%               cSim.subject
%               cSim.trial                 .num,     .type 
%               cSim.events()              .label,   .time
%               cSim.fpInfo()              .corners, .origin
%               cSim.tDS  - gives the time of double support (in seconds) 
%                           between IC and OTO of the 1st FP hit; 
%                 
%               POINT/VIDEO Data:
%               cSim.video         .rate,  .nframes, .units
%               cSim.markers()             .label, .data
%               cSim.jntcenters    .L(),   .label, .data
%                                  .R(),   .label, .data
%               cSim.jntangles     .L(),   .label, .data 
%                                  .R(),   .label, .data
%               cSim.jntmoments    .L(),   .label, .data
%                                  .R(),   .label, .data
%               cSim.jntpowers     .L(),   .label, .data
%                                  .R(),   .label, .data
%
%               ANALOG Data:
%               cSim.analog                .rate,  .ratio
%               cSim.grf()                 .label, .data
%               cSim.emg()                 .label, .data

if fileName
	fname = fileName;
else
	fname = [subject, ' ', tInfo.trial, '.c3d'];
end
fname
c = read_c3dFile(fname, ref_dataFormat); 

% Get analog frames corresponding to IC and TO events; 
% if icto data have not been previously stored, call get_ictoEvents();
% if icto data have been stored, convert matrix to structure.
if isempty(tInfo.ictoMatrix)            
    figHandle = 2;
    ictoEvents = get_ictoEvents(c, tInfo, figHandle, ref_dataFormat);
else                                    
    for fpHitNum = 1:length(tInfo.FP)     
        ictoEvents(fpHitNum).ic  = tInfo.ictoMatrix(fpHitNum, 1);
        ictoEvents(fpHitNum).oto = tInfo.ictoMatrix(fpHitNum, 2);
        ictoEvents(fpHitNum).oic = tInfo.ictoMatrix(fpHitNum, 3);
        ictoEvents(fpHitNum).to  = tInfo.ictoMatrix(fpHitNum, 4);
        ictoEvents(fpHitNum).icNext = tInfo.ictoMatrix(fpHitNum, 5);
    end
end

% Extract 'simulateable' segment of data from c, 
% from 1st OTO to last OIC of available FP hits.
cSim = extract_c3dSimData(c, ictoEvents);           


%%% COMPUTE BACK ANGLES IN REF FRAMES CONSISTENT WITH MODEL (GAIT TRIAL)
%   NOTE:  backAngles returns a structure containing the angles between 
%          the model's pelvis and trunk segments, as determined from 
%          the subject's marker data, in the following format:
%           *.extension(nVideoFrames of 'simulateable' segment)
%            .bending(")
%            .rotation(")
figHandle = 2;
if ~isempty(tInfo.static) 
    backAngles = compute_c3dBackAngles(cSim, tInfo, figHandle, trunk_rzStatic);
else
    backAngles = compute_c3dBackAngles(cSim, tInfo, figHandle);
end

%%% CORRECT JOINT ANGLES FOR MALALIGNED KNEE AXIS, AS NEEDED (GAIT TRIAL)
%   NOTE:  corrected_kAxisAngles returns a structure containing the
%          'corrected' knee flex/ext, knee var/val, knee rotation,
%          and hip rotation angles, in the following format:
%           *.R     .kf(nVideoFrames of 'simulateable' segment)
%           *.L     .kv(")
%                   .kr(")
%                   .hr(")
%                   .kAxisOffset
figHandleArray = 3:4;
corrected_kAxisAngles = ...
    evaluate_c3dkAxisMalalignment(cSim, tInfo, ictoEvents, figHandleArray);


%%% PROCESS ANALOG GRF DATA (GAIT TRIAL)
%   NOTE:  GRFTz_byLimb returns a structure with the following format:
%               *.R         .Fx(nAnalogFrames of 'simulateable' segment)
%               *.L         .Fy(")
%                           .Fz(")
%                           .Tz(")
%                           .COPx(")
%                           .COPy(")
%                           .startIndex - analog frame numbers indicating
%                                         start of COP, Tz data
%                           .stopIndex  - analog frame numbers indicating
%                                         end of COP, Tz data
%           COP_smoothed returns a structure with the following format:
%               *.R         .COPx("), discontinuities removed
%               *.L         .COPy("), discontinuities removed
figHandleArray = 5:6;
[GRFTz_byLimb, COP_smoothed] = process_c3dGRFs(cSim, tInfo, figHandleArray);


%%% TRANSFORM DATA FROM THE LAB CS TO THE MODEL CS (GAIT TRIAL)
%   NOTES: mCS ('model CS') returns a structure with the following format:
%          *.markers     .dataName(nVideoFrames, XYZ coordinates)
%          *.markers     .pelvisTranslations(nVideoFrames, XYZ coordinates)
%          *.jntcenters  .dataName(nVideoFrames, XYZ coordinates)
%          *.jntangles   .dataName(nVideoFrames)
%          *.forces      .dataName(nVideoFrames)
%          *.time(nVideoFrames)
mCS.markers = transform_markersToModelCS(cSim, tInfo);
mCS.jntcenters = transform_jntcentersToModelCS(cSim, tInfo);
mCS.jntangles = transform_jntanglesToModelCS(cSim, ...
                                        corrected_kAxisAngles, backAngles);
mCS.forces = transform_forcesToModelCS(GRFTz_byLimb, COP_smoothed, ...
                                        cSim, tInfo);

% Define array of time values corresponding to the 'simulateable' data.
% Set t = 0 at IC of the 1st FP strike; note that the data starts at OTO, 
% which occurs at cSim.tDS seconds later.
vFrames = 1:1:cSim.video.nframes;
mCS.time = (vFrames/cSim.video.rate + cSim.tDS)';

                                        
%%% WRITE DATA TO FILES FOR USE IN DARRYL'S EXECUTABLES (GAIT TRIAL)
%   NOTE:  If tInfo.static == [], write anatomical markers only.
%          If tInfo.static ~= [], write anatomical and tracking markers.

% Write *.trc file, and return matrix of marker data in units of m.
fname = [subject, '_', tInfo.trial, '.trc'];  
if isempty(tInfo.static)    
    mkrMatrix = write_c3dTrcFile(mCS, cSim, fname, 'anatOnly');   
else
    mkrMatrix = write_c3dTrcFile(mCS, cSim, fname, 'anatTracking');   
end

% Write *.mot file.
fname = [subject, '_', tInfo.trial, '.mot'];
write_c3dMotFile(mCS, mkrMatrix, fname);

% Write motion file containing joint moments;
% useful for overlaying measured moments on plots 
% generated from UW-Gait invdyn.exe.
fname = [subject, '_', tInfo.trial, 'Moments.mot'];
write_c3dMotMoments(cSim.jntmoments, mCS.time, fname);
return;
