function [] = plot_c3dTrialvsCycle(subject, tInfo)
% Purpose:  Generates plots of joint angles, joint moments, and 
%           processed EMG envelopes vs gait cycle for all R and L cycles
%           corresponding to FP hits (as specified in tInfo) of a single
%           trial read from a C3D file of a Gillette control subject.
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
% Called Functions:
%       read_c3dFile(fname)
%       get_ictoEvents(c, tInfo, figHandle)
%       extract_c3dCycleData(c, ictoEvents, tInfo)
%       process_emgChannel(c, emgChannel)
%       extract_emgCycleData(proEMG, c, ictoEvents, tInfo)
%       read_gcdMean(fname)
%       create_jntAngMomFigTemplate(figHandle, g)
%       overlay_jntAngMomTrialData(gcR, gcL, tInfo, figHandle)
%       create_emgTrialFigs(emgCyc, muscTrialList, muscRefList, tInfo, ...
%                                                    figHandleArray)
% ASA, 9-05


% Display status on screen.
status = sprintf('\n\n\n%s%s%s%s%s%3.2f%s', ...
            'Plotting Trial Data vs Gait Cycle for Subject ', subject, ...
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

% Extract individual gait cycles from C3D data.
[gcR, gcL] = extract_c3dCycleData(c, ictoEvents, tInfo);

% Process and extract individual gait cycles from EMG data.
nChannels = length(c.emg);
for emgChannel = 1:nChannels        
    proEMG(emgChannel) = process_emgChannel(c, emgChannel);    
end
emgCyc = extract_emgCycleData(proEMG, c, ictoEvents, tInfo);

% Read averaged data from control subjects.
g = read_gcdMean('gilletteMeanSD.GCD');

% Create template figure for plotting joint angles and moments.
figHandle = 1;
create_jntAngMomFigTemplate(figHandle, g);

% Plot joint angles and joint moments.
overlay_jntAngMomTrialData(gcR, gcL, tInfo, figHandle);

% Specify muscles corresponding to EMG channels of interest.
% NOTE:  the muscle names must correspond to those in get_emgChannel().
muscTrialList{1} = 'Rectus Femoris';
muscTrialList{2} = 'Medial Hamstrings';
muscTrialList{3} = 'Lateral Hamstrings';
muscTrialList{4} = 'Tibialis Anterior';
muscTrialList{5} = 'Gastroc-Soleus';
    
% Specify muscles corresponding to EMG reference data of interest
% for overlaying EMG on/off times reported by Perry (1992).
% NOTE:  the muscle abbreviations must correspond to those in 
%        get_emgTimingFromPerry().
muscRefList{1} = {'RF', 'VASmed', 'VASint', 'VASlat'};
muscRefList{2} = {'SM', 'ST', 'GR', 'ADM'};
muscRefList{3} = {'BFLH', 'BFSH', 'VASlat'};
muscRefList{4} = {'TA', 'EDL', 'PERlong'};
muscRefList{5} = {'GAS', 'SOL'};
    
% Plot EMG and overlay Perry's data.
figHandleArray = 2:length(muscTrialList)+1;
create_emgTrialFigs(emgCyc, muscTrialList, muscRefList, tInfo, ...
                                                    figHandleArray);
return;