function [] = get_c3dEmgEnv(subject, tInfo, ref_dataFormat)
% Purpose:  Processes the analog EMG data read from the C3D file of a 
%           Gillette control subject, extracts a 'simulateable' segment,
%           and writes the normalized rectified EMG data and EMG envelopes, 
%           sampled at the analog frame rate, to a motion file.
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
%
% Output:   EMG data are written to the file subject_trial#EmgEnv.mot, 
%           in the current directory.  
%
% Notes:    1. Times are referenced to IC of the 1st FP strike, which
%              is defined as t = 0.
%           2. This code calls subfunctions that assume subjects were 
%              tested using Gillette's 'new' 10-channel protocol for EMG
%              data collection (i.e., tested after 11-19-01).
%           3. EMG processing is done by the function process_emgChannel.
%           4. Data written to the motion file (proEMG.nrect, nlow from
%              process_emgChannel() are normalized by the max/min values 
%              from the original trial before extracting a simulateable 
%              segment.
%           5. I tried writing data at the video frame rate, but the
%              rectified EMG data lost too much signal ...
%
% Called Functions:
%       read_c3dFile(fname)
%       get_ictoEvents(c, tInfo, figHandle)
%       extract_c3dSimData(c, ictoEvents)
%       process_emgChannel(c, emgChannel)
%       extract_emgSimData(proEMG, c, ictoEvents)
%       plot_proEmgChannel(emgData, aTime, emgChannel)
%       get_emgLabels(emgChannel)
%       write_c3dMotEmgEnv(emgEnv, fname)
%
% ASA, 11-05


% Display status on screen.
status = sprintf('\n\n\n%s%s%s%s%s%3.2f%s', ...
            'Processing EMG Envelopes for Subject ', subject, ...
            ', Trial ', tInfo.trial, ...
            ', Speed ', tInfo.speed, ' m/s');
disp(status);

% Read data from C3D file.
fname = [subject, ' ', tInfo.trial, '.c3d'];
c = read_c3dFile(fname); 

% Get analog frames corresponding to IC and TO events; 
% if icto data have not been previously stored, call get_ictoEvents();
% if icto data have been stored, convert matrix to structure.
if isempty(tInfo.ictoMatrix)            
    figHandle = 1;
    ictoEvents = get_ictoEvents(c, tInfo, figHandle);
else                                    
    for fpHitNum = 1:length(tInfo.FP)     
        ictoEvents(fpHitNum).ic  = tInfo.ictoMatrix(fpHitNum, 1);
        ictoEvents(fpHitNum).oto = tInfo.ictoMatrix(fpHitNum, 2);
        ictoEvents(fpHitNum).oic = tInfo.ictoMatrix(fpHitNum, 3);
        ictoEvents(fpHitNum).to  = tInfo.ictoMatrix(fpHitNum, 4);
        ictoEvents(fpHitNum).icNext = tInfo.ictoMatrix(fpHitNum, 5);
    end
end

% Extract 'simulateable' segment of C3D data from c, 
%   from 1st OTO to last OIC of available FP hits.
cSim = extract_c3dSimData(c, ictoEvents);           

% Extract 'simulateable' segments of EMG data, after signal processing. 
nChannels = length(c.emg);
for emgChannel = 1:nChannels        
    proEMG(emgChannel) = process_emgChannel(c, emgChannel);    
end
emgSim = extract_emgSimData(proEMG, c, ictoEvents);

% Visually assess algorithms for EMG processing based on entire trial by 
% plotting raw, band-passed, rectified, and low-pass filtered data vs time.
% NOTE:  set algEval_flag == 0 to skip these commands.
algEval_flag = 0;                       % 'algorithm evaluation' flag
if algEval_flag == 1       
    nAnalogFrames = c.video.nframes * c.analog.ratio;
    aFrames = 1:1:nAnalogFrames;
    aTime = aFrames/c.analog.rate;
    for emgChannel = 1:nChannels  
        plot_proEmgChannel(proEMG(emgChannel), aTime, emgChannel);
        pause;
    end
end

% Define arrays of time values corresponding to the 'simulateable' data.
% Set t = 0 at IC of the 1st FP strike; note that the data starts at OTO, 
% which occurs at cSim.tDS seconds later.
nAnalogFrames = cSim.video.nframes * cSim.analog.ratio;
aFrames = 1:1:nAnalogFrames;
aTime = aFrames/cSim.analog.rate + cSim.tDS;

% For each analog EMG channel ...
for emgChannel = 1:nChannels        

    % Get limb and muscle corresponding to the current EMG channel.
    [limb, muscle] = get_emgLabels(emgChannel, ref_dataFormat);
    
    % Store the normalized rectified EMG data and EMG envelopes,
    % sampled at analog frame rate.
    emgEnv(emgChannel).limb = limb;
    emgEnv(emgChannel).muscle = muscle;    
    emgEnv(emgChannel).emgRectified = [aTime' emgSim(emgChannel).nrect'];
    emgEnv(emgChannel).emgEnvelope = [aTime' emgSim(emgChannel).nlow'];
end

fname = [subject, '_', tInfo.trial, 'EmgEnv.mot'];
write_c3dMotEmgEnv(emgEnv, fname);
return;
