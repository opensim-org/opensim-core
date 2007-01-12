function [] = create_emgTrialFigs(emgCyc, muscTrialList, muscRefList, ...
                                        tInfo, figHandleArray, ref_dataFormat)
% Purpose:  Plots EMG envelopes vs gait cycle for muscles specified in
%           muscTrialList (one figure per muscle, using figure numbers
%           specified by figHandle()), for all R and L cycles of a single
%           trial read from the C3D file of a Gillette control subject.
%           For reference, the EMG on/off times reported by Perry, for
%           the muscles listed in muscRefList, are also displayed.
%
% Input:    emgCyc is a structure with the following format,
%               in analog frames:
%               *.R{cycleNum}{emgChannel} 
%               *.L{cycleNum}{emgChannel}
%                       .raw, .band, .rect, .low, .nrect, .nlow
%               *.subject
%               *.trial
%           muscTrialList is a cell array specifying the muscle names
%               corresponding to EMG channels of interest; these
%               must correspond to the names in get_emgLabels().
%           muscRefList is a cell array specifying the muscle abbreviations
%               corresponding to EMG reference data of interest; these
%               must correspond to the abbreviations in get_perryEMG().
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.trial - trial number to analyze ('character array')
%               *.speed - average walking speed during trial in m/s
%           figHandleArray is an array of handle numbers for figure windows,
%               one per muscle.
%                      
% NOTES:    Before returning to the calling function, the user is prompted
%           to select 'print' or 'done' from a menu.
%
% Called Functions:
%           get_emgChannel(limb, muscle)
%           get_emgTimingFromPerry(muscAbbr)
%           plot_emgTrialData(emgR, emgL, gaitCycle, perryData, figHandle)
%           Suptitle(titleString)
%
% ASA, 9-05


% Get number of analog frames per cycle, and create % gait cycle arrays.
% Store results in a structure with the following format:
%   gaitCycle.R(nRcycles).nAnalogFrames, .aCycle
%            .L(nLcycles).nAnalogFrames, .aCycle
nRcycles = length(emgCyc.R);
nLcycles = length(emgCyc.L);
for cycleNum = 1:nRcycles
    gaitCycle.R(cycleNum).nAnalogFrames = ...
            length(emgCyc.R{cycleNum}{1}.nlow);
    delta = 100/(gaitCycle.R(cycleNum).nAnalogFrames - 1);      
    gaitCycle.R(cycleNum).aCycle =  0:delta:100;
end
for cycleNum = 1:nLcycles
    gaitCycle.L(cycleNum).nAnalogFrames = ...
            length(emgCyc.L{cycleNum}{1}.nlow);
    delta = 100/(gaitCycle.L(cycleNum).nAnalogFrames - 1);        
    gaitCycle.L(cycleNum).aCycle = 0:delta:100;
end

% For each muscle of interest ...
nMuscles = length(muscTrialList);
for muscNum = 1:nMuscles
   
    % Get EMG channel numbers corresponding to muscle of interest.
    channelR = get_emgChannel('R', muscTrialList{muscNum}, ref_dataFormat);
    channelL = get_emgChannel('L', muscTrialList{muscNum}, ref_dataFormat);
        
    % Get EMG envelopes corresponding to muscle of interest.
    for cycleNum = 1:nRcycles
        emgR{cycleNum} = emgCyc.R{cycleNum}{channelR}.nlow;
    end
    for cycleNum = 1:nLcycles
        emgL{cycleNum} = emgCyc.L{cycleNum}{channelL}.nlow;
    end
    
    % Get EMG on/off reference data of interest.
    clear perryData;
    nRefMuscles = length(muscRefList{muscNum});
    for refNum = 1:nRefMuscles
        perryData(refNum) = ...
            get_emgTimingFromPerry(muscRefList{muscNum}{refNum});
    end
        
    % Generate figure. 
    figHandle = figHandleArray(muscNum);
    plot_emgTrialData(emgR, emgL, gaitCycle, perryData, figHandle);
        
    % Add title.
    titleString = sprintf('%s%s%s%s%s%s%3.2f%s', ...
       muscTrialList{muscNum}, ' EMG Envelopes:  Subject ', ...
       char(emgCyc.subject), '-', tInfo.trial, ', Speed ', ...
       tInfo.speed, ' m/s');
    Suptitle(titleString);      % MATLAB m-file for adding "super title"

    % Query user.
    done = 0;
    while ~done
        query = 'Send figure to printer?';
        opt1 = 'print';
        opt2 = 'done';
        userInput = menu(query, opt1, opt2);
        switch userInput
            case 1
                orient(figHandle, 'tall');  
                printCommand = ['print -f', num2str(figHandle), ' -r600'];
                eval(printCommand);
                done = 1;
            case 2
                done = 1;
        end
    end
    
end
return;
