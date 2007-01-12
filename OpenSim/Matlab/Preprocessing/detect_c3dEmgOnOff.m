function emgEnv = detect_c3dEmgOnOff(subject, tInfo)
% Purpose:  Processes the analog EMG data read from the C3D file of a 
%           Gillette control subject, extracts a 'simulateable' segment,
%           estimates the EMG on/off times, and returns the EMG envelopes,
%           sampled at the video frame rate.  
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
% Output:   EMG envelopes are returned in the structure emgEnv(emgChannel)
%           with the following format:
%               *().limb     - limb corresponding to EMG channel #
%               *().muscle   - muscle corresponding to EMG channel #
%               *().envelope - matrix [time value] corresponding to 
%                              rectified, filtered, normalized envelope,
%                              sampled at video frame rate
%           EMG on/off times are written to the screen.
%           Reference EMG on/off data from Perry (1992) are also written 
%               to the screen after being scaled to the subject's cycles.
%
% Notes:    1. This code is interactive; the user is queried to adjust the
%              threshold & window parameters used for EMG on/off detection.
%           2. Times are referenced to IC of the 1st FP strike, which
%              is defined as t = 0.
%           3. This code calls subfunctions that assume subjects were 
%              tested using Gillette's 'new' 10-channel protocol for EMG
%              data collection (i.e., tested after 11-19-01).
%
% Called Functions:
%       read_c3dFile(fname)
%       get_ictoEvents(c, tInfo, figHandle)
%       extract_c3dSimData(c, ictoEvents)
%       process_emgChannel(c, emgChannel)
%       extract_emgSimData(proEMG, c, ictoEvents)
%       plot_proEmgChannel(emgData, aTime, emgChannel)
%       get_vertGRF(c, tInfo)
%       detect_emgTimes(emgData, threshold, onWindow, offWindow, cSim)
%       get_emgLabels(emgChannel)
%       create_emgOnOffFig(emgData, fzData, aTime, figHandle)
%       overlay_emgOnOffTimes(onTimes, offTimes, aTime, ...
%                               threshold, onWindow, offWindow, figHandle)
%       get_muscRefList(muscle)
%       get_emgTimingFromPerry(muscAbbr)
%       convert_perryDataToTime(perryData, ictoEvents, analogRate, ...
%                                   limb, tInfo)
%       overlay_emgPerryScaled(perryScaled, aTime, figHandle)
%       overlay_emgFigTitles(limb, muscle, cSim, tInfo, figHandle)
%
% ASA, 9-05


% Display status on screen.
status = sprintf('\n\n\n%s%s%s%s%s%3.2f%s', ...
            'Detecting EMG On/Off Times for Subject ', subject, ...
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

% Get vertical GRFs, for reference.
[fzR, fzL] = get_vertGRF(cSim, tInfo);

% For each analog EMG channel ...
for emgChannel = 1:nChannels        

    % Get limb and muscle corresponding to the current EMG channel.
    [limb, muscle] = get_emgLabels(emgChannel, ref_dataFormat);

    % Set default parameters for detecting EMG on/off times.
    threshold = 0.10;    % threshold for on/off = 10% of normalized range                   
    onWindow = 50;       % # analog frames > threshold to be assumed 'on'      
    offWindow = 50;      % # analog frames < threshold to be assumed 'off'
        
    % Iteratively detect EMG on/off times and plot data until user is 
    % satisfied w/ parameters.
    done = 0;
    while ~done
        
        % Get EMG on/off times.
        [onTimes, offTimes] = detect_emgTimes(emgSim(emgChannel), ...
                                    threshold, onWindow, offWindow, cSim);
        % Generate figure.
        % Define number of figure window to correspond to emgChannel.
        figHandle = emgChannel;    
        if strcmpi(limb, 'R')
            create_emgOnOffFig(emgSim(emgChannel), fzR, aTime, figHandle);
        elseif strcmpi(limb, 'L')
            create_emgOnOffFig(emgSim(emgChannel), fzL, aTime, figHandle);
        end
        
        % Overlay EMG on/off times on figure.
        overlay_emgOnOffTimes(onTimes, offTimes, aTime, ...
                        threshold, onWindow, offWindow, figHandle);                            
                
        % Get EMG on/off reference data of interest, from Perry (1992),
        % and convert from % gait cycle to time.
        clear muscRefList;
        clear perryData;
        muscRefList = get_muscRefList(muscle);
        nRefMuscles = length(muscRefList);
        for refNum = 1:nRefMuscles
            perryData(refNum) = ...
                            get_emgTimingFromPerry(muscRefList{refNum});
        end
        perryScaled = convert_perryDataToTime(perryData, ictoEvents, ...
                            c.analog.rate, limb, tInfo);
        
        % Overlay reference data on figure.
        overlay_emgPerryScaled(perryScaled, figHandle);
        
        % Add title
        overlay_emgFigTitles(limb, muscle, cSim, tInfo, figHandle);

        % Get input from user.
        query = 'Adjust On/Off Parameters?';
        opt1 = 'threshold ++';
        opt2 = 'threshold --';
        opt3 = 'on window ++';
        opt4 = 'on window --';
        opt5 = 'off window ++';
        opt6 = 'off window --';
        opt7 = 'print';
        opt8 = 'done';
        userInput = ...
            menu(query, opt1, opt2, opt3, opt4, opt5, opt6, opt7, opt8);
        switch userInput
            case 1
                threshold = threshold + 0.01;
            case 2
                threshold = threshold - 0.01;
            case 3
                onWindow = onWindow + 10;
            case 4
                onWindow = onWindow - 10;
            case 5
                offWindow = offWindow + 10;
            case 6
                offWindow = offWindow - 10;
            case 7
                orient(figHandle, 'tall');  
                printCommand = ['print -f', num2str(figHandle), ' -r600'];
                eval(printCommand);
                done = 1;
            case 8    
                done = 1;
        end
    end

    % Write EMG on/off times to formatted strings; each string represents
    %   one row of a table to be written to the screen.
    onString = num2str(onTimes, '%6.4f\t');
    offString = num2str(offTimes, '%6.4f\t');
    table(emgChannel).muscle = sprintf('\n\t%s%s%s', limb, ' ', muscle);
    table(emgChannel).on   = sprintf('\t%6s%s%s', 'On:', '  ', onString);
    table(emgChannel).off  = sprintf('\t%6s%s%s', 'Off:', '  ', offString);
        
    % Store EMG envelopes, sampled at video frame rate.
    emgEnv(emgChannel).limb = limb;
    emgEnv(emgChannel).muscle = muscle;    
    vRateIndices = 1:cSim.analog.ratio:nAnalogFrames;
    emgEnv(emgChannel).envelope = [aTime(vRateIndices)' ...
                                   emgSim(emgChannel).nlow(vRateIndices)'];
end

% Write EMG on/off times to screen.
tableTitle = sprintf('\n\t%s\n\t%s', ...
                     'EMG On/Off Times from Analog Data: ', ...
                     '(t = 0 at IC of 1st FP hit)');
disp(tableTitle);
for emgChannel = 1:nChannels
    disp(table(emgChannel).muscle);
    disp(table(emgChannel).on);
    disp(table(emgChannel).off);
end

% Write EMG on/off times for 'all' muscles reported by Perry, 
% scaled to 'simulateable' segment, to screen.
tableTitle = sprintf('\n\n\t%s\n\t%s\n', ...
            'EMG On/Off Times Reported by Perry, Scaled to Subject:  ', ...
            '(t = 0 at IC of 1st FP hit)');
disp(tableTitle);
perryAll = get_emgTimingFromPerry('all');
perryAllScaledR = convert_perryDataToTime(perryAll, ictoEvents, ...
                            c.analog.rate, 'R', tInfo);
perryAllScaledL = convert_perryDataToTime(perryAll, ictoEvents, ...
                            c.analog.rate, 'L', tInfo);                            
for muscleNum = 1:length(perryAllScaledL)
    perryOnString = ...
        num2str(perryAllScaledL(muscleNum).onoff(:, 1)', '%6.4f\t');
    perryOffString = ...
        num2str(perryAllScaledL(muscleNum).onoff(:, 2)', '%6.4f\t');
    perryAbbrTable = ...
        sprintf('\t%s%s', 'L ', perryAllScaledL(muscleNum).muscle);
    perryOnTable = sprintf('\t%6s%s%s', 'On:', '  ', perryOnString);
    perryOffTable = sprintf('\t%6s%s%s', 'Off:', '  ', perryOffString);
    disp(perryAbbrTable);
    disp(perryOnTable);
    disp(perryOffTable);
end 
disp(' ');
for muscleNum = 1:length(perryAllScaledR)
    perryOnString = ...
        num2str(perryAllScaledR(muscleNum).onoff(:, 1)', '%6.4f\t');
    perryOffString = ...
        num2str(perryAllScaledR(muscleNum).onoff(:, 2)', '%6.4f\t');
    perryAbbrTable = ...
        sprintf('\t%s%s', 'R ', perryAllScaledR(muscleNum).muscle);
    perryOnTable = sprintf('\t%6s%s%s', 'On:', '  ', perryOnString);
    perryOffTable = sprintf('\t%6s%s%s', 'Off:', '  ', perryOffString);
    disp(perryAbbrTable);
    disp(perryOnTable);
    disp(perryOffTable);
end
return;
