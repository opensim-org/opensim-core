function [] = get_c3dEmgAveOnOff(subject, tInfo, refTrials, ref_dataFormat, fileNames)
% Purpose:  This function:
%           1. processes analog EMG data read from the C3D file of a 
%              Gillette control subject.
%           2. processes analog EMG data read from 'reference' C3D files
%              from the same subject, at similar speeds.
%           3. computes the ensemble-averaged EMG envelopes +/-1SD from all
%              cycles of all trials, and plots these data vs. gait cycle
%           4. writes trial-specific normalized rectified EMG data and 
%              EMG envelopes for the 'simulateable segment', sampled at 
%              the analog frame rate, to a motion file along with the
%              ensemble-averaged EMG envelopes and SDs
%           5. writes reference EMG on/off times from Perry (1992),
%              scaled to the subject's stance and swing phases,
%              to the screen.
%           6. gives the user the option to identify EMG on/off times
%              from the EMG envelopes, and write the results to the screen.
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
%           refTrials is a structure containing a list of reference trial#s
%             and the following information:
%             *(n Ref Trials).trial
%                            .FP
%                            .limb
%                            .ictoMatrix
%
% Output:   EMG data are written to the file subject_trial#EmgEnv.mot, 
%               in the current directory.  
%           Reference EMG on/off times from Perry (1992) are written 
%               to the screen.
%           EMG on/off times identified by the user (optional) are written 
%               to the screen.
%
% Notes:    1. Times are referenced to IC of the 1st FP strike, which
%              is defined as t = 0.
%           2. This code calls subfunctions that assume subjects were 
%              tested using Gillette's 'new' 10-channel protocol for
%              EMG data collection (i.e., tested after 11-19-01).
%           3. EMG processing is done by the function process_emgChannel.
%           4. Data written to the motion file are normalized by the 
%              max/min values from the original trial before extracting 
%              a simulateable segment.
%           5. I tried writing data at the video frame rate, but the
%              rectified EMG data lost too much signal ...
%
% Called Functions:
%   read_c3dFile(fname)
%   process_emgChannel(c, emgChannel)
%   extract_emgCycleData(proEMG, c, ictoEvents, tInfo)
%   resample_emgEnvCycle(emgCyc, cycle)
%   get_emgEnsembleAve(emgEnvResamp, nChannels)
%   create_emgEnsembleAveFigs(emgEnsembleAve, cycle, ...
%                                            TOI_emgCyc, tInfo, nChannels)
%   extract_c3dSimData(c, ictoEvents)
%   extract_emgSimData(proEMG, c, ictoEvents)
%   plot_proEmgChannel(emgData, aTime, emgChannel)
%   convert_emgEnsembleAveToTime(emgEnsembleAve, cycle, tInfo, ...
%                                                ictoEvents, analogRate)
%   resample_emgEnvTime(emgEnsembleAveSim, emgEnsembleAveTime, aTime)
%   write_c3dMotEmgEnvAve(emgEnv, fname)
%   get_vertGRF(c, tInfo)
%   get_emgLabels(emgChannel)
%   detect_emgAveTimes(emgData, threshold, onWindow, offWindow, cSim)
%   create_emgAveOnOffFig(emgData, fzData, aTime, figHandle)
%   overlay_emgOnOffTimes(onTimes, offTimes, aTime, ...
%                               threshold, onWindow, offWindow, figHandle)
%   get_muscRefList(muscle)
%   get_emgTimingFromPerry(muscAbbr)
%   convert_perryDataToTime(perryData, ictoEvents, analogRate, ...
%                                   limb, tInfo)
%   overlay_emgPerryScaled(perryScaled, aTime, figHandle)
%   overlay_emgFigTitles(limb, muscle, cSim, tInfo, figHandle)
%
% ASA, 12-05

if nargin < 5
	fileNames = {};
end

cycle = 0:0.2:100;

%%% CALCULATE ENSEMBLE-AVERAGED EMG ENVELOPES
% For each file to be included in the ensemble average ...
nRefTrials = length(refTrials);
nFiles = nRefTrials + 1;
for fileNum = 1:nFiles
    
    % Generate C3D filename, store trial information, 
    % and display status on screen.
    % NOTE:  fname{1} is the trial of interest; 
    %        fname{>1} are reference trials.
    if fileNum == 1
		if length(fileNames)
			fname{fileNum} = fileNames{fileNum};
		else
			fname{fileNum} = [subject, ' ', tInfo.trial, '.c3d'];
		end
        refInfo = tInfo;
        status = sprintf('\n\n\nProcessing EMG Envelopes for Subject %s, Trial %s (file="%s"), Speed %3.2f m/s', ...
                    subject, tInfo.trial, fname{fileNum}, tInfo.speed);
        disp(status);        
    else
        refNum = fileNum - 1;
		if length(fileNames)
			fname{fileNum} = fileNames{fileNum};
		else
			fname{fileNum} = [subject, ' ', refTrials(refNum).trial, '.c3d'];
		end
        refInfo = refTrials(refNum);
        status = sprintf('\n\tReading Reference Trial %s (file="%s")', ...
					refInfo.trial, fname{fileNum});
        disp(status);        
    end
    
    % Read data from C3D file.
    c = read_c3dFile(fname{fileNum}, ref_dataFormat); 
        
    % Get analog frames corresponding to IC and TO events.
    for fpHitNum = 1:length(refInfo.FP)     
        ictoEvents(fpHitNum).ic  = refInfo.ictoMatrix(fpHitNum, 1);
        ictoEvents(fpHitNum).oto = refInfo.ictoMatrix(fpHitNum, 2);
        ictoEvents(fpHitNum).oic = refInfo.ictoMatrix(fpHitNum, 3);
        ictoEvents(fpHitNum).to  = refInfo.ictoMatrix(fpHitNum, 4);
        ictoEvents(fpHitNum).icNext = refInfo.ictoMatrix(fpHitNum, 5);
    end
    
    % Extract EMG records for each gait cycle in each trial, 
    % after signal processing.
    nChannels = length(c.emg);
    for emgChannel = 1:nChannels
        proEMG(emgChannel) = process_emgChannel(c, emgChannel);    
    end
    emgCyc = extract_emgCycleData(proEMG, c, ictoEvents, refInfo);

    % Re-sample each cycle of EMG envelope data to have the same # pts,
    % as specified by cycle.
    emgEnvResamp(fileNum) = resample_emgEnvCycle(emgCyc, cycle);
    
    % Store processed EMG data for the trial of interest ('TOI').
    if fileNum == 1
        TOI_c = c;
        TOI_ictoEvents = ictoEvents;
        TOI_proEMG = proEMG;
        TOI_emgCyc = emgCyc;
    end
    clear c refInfo ictoEvents proEmg emgCyc;
end
    
% Compute ensemble average of the EMG envelopes for each channel.
emgEnsembleAve = get_emgEnsembleAve(emgEnvResamp, nChannels, ref_dataFormat);
 
% Plot EMG envelopes vs gait cycle.
% NOTE:  set algEval_flag == 0 to skip these commands.
algEval_flag = 0;                       % 'algorithm evaluation' flag
if algEval_flag == 1       
    create_emgEnsembleAveFigs(emgEnsembleAve, cycle, ...
                                        TOI_emgCyc, tInfo, nChannels, ref_dataFormat);
end

    
%%% PROCESS SIMULATEABLE SEGMENT
c = TOI_c;
ictoEvents = TOI_ictoEvents;
proEMG = TOI_proEMG;

% Extract 'simulateable' segments of data from the trial of interest.
if ref_dataFormat.extractSimulateableSegment
    cSim = extract_c3dSimData(c, ictoEvents); 
    emgSim = extract_emgSimData(proEMG, c, ictoEvents);
else
    cSim = c;
    cSim.tDS = 0;
    emgSim = proEMG;
end

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

% Define array of time values corresponding to the 'simulateable' data.
% Set t = 0 at IC of the 1st FP strike; note that the data starts at OTO, 
% which occurs at cSim.tDS seconds later.
nAnalogFrames = cSim.video.nframes * cSim.analog.ratio;
aFrames = 1:1:nAnalogFrames;
aTime = aFrames/cSim.analog.rate + cSim.tDS;


%%% WRITE EMG ENVELOPES TO A MOTION FILE
% Convert ensemble-averaged data from gait cycle to time corresponding to
% the 'simulateable' segment.
[emgEnsembleAveSim, ...
   emgEnsembleAveTime] = convert_emgEnsembleAveToTime(emgEnsembleAve, ...
                               cycle, tInfo, ictoEvents, cSim.analog.rate, ref_dataFormat);

% Re-sample the ensemble-averaged EMG envelopes and SDs to have the
% same # pts as specified by aTime.
emgEnsembleAveResamp = ...
  resample_emgEnvTime(emgEnsembleAveSim, emgEnsembleAveTime, aTime);

% For each analog EMG channel ...
for emgChannel = 1:nChannels        

    % Store limb, muscle, time corresponding to current channel.
    emgEnv(emgChannel).limb = emgEnsembleAveResamp(emgChannel).limb;
    emgEnv(emgChannel).muscle = emgEnsembleAveResamp(emgChannel).muscle;   
    emgEnv(emgChannel).time = aTime;
    
    % Store trial-specific normalized rectified EMG data and envelope,
    % sampled at the analog frame rate.
    emgEnv(emgChannel).emgRectified = emgSim(emgChannel).nrect;
    emgEnv(emgChannel).emgEnvelope = emgSim(emgChannel).nlow;
    
    % Store the ensemble-averaged EMG envelope and SD,
    % sampled at the analog frame rate.
    emgEnv(emgChannel).emgAve = emgEnsembleAveResamp(emgChannel).envAve;
    emgEnv(emgChannel).emgSD = emgEnsembleAveResamp(emgChannel).envSD;
end

% Write data to file.
fname = [subject, '_', tInfo.trial, 'EmgEnv.mot'];
write_c3dMotEmgEnvAve(emgEnv, fname, ref_dataFormat);


%%% DETECT SUBJECT-SPECIFIC EMG ON/OFF TIMES (optional)
% Give user option of detecting EMG on/off times.
query = 'Detect EMG On/Off Times?';
opt1 = 'YES';
opt2 = 'NO';
userInput = menu(query, opt1, opt2);
switch userInput
    case 1
        detectOnOffFlag = 1;
    case 2
        detectOnOffFlag = 0;
end

% If detectOnOffFlag == 1 ...
if detectOnOffFlag == 1
    
    % Get vertical GRFs, for reference.
    [fzR, fzL] = get_vertGRF(cSim, tInfo);

    % For each analog EMG channel ...
    for emgChannel = 1:nChannels        

        % Get limb and muscle corresponding to the current EMG channel.
        [limb, muscle] = get_emgLabels(emgChannel, ref_dataFormat);

        % Set default parameters for detecting EMG on/off times.
        threshold = 0.15;    % threshold for on/off = 15% of normalized range                   
        onWindow = 50;       % # analog frames > threshold to be assumed 'on'      
        offWindow = 50;      % # analog frames < threshold to be assumed 'off'

        % Iteratively detect EMG on/off times and plot data until user is 
        % satisfied w/ parameters.
        done = 0;
        while ~done

            % Get EMG on/off times.
            [onTimes, offTimes] = detect_emgAveTimes(emgEnv(emgChannel), ...
                                        threshold, onWindow, offWindow, cSim);

            % Generate figure.
            % Define number of figure window to correspond to emgChannel.
            figHandle = emgChannel;    
            if strcmpi(limb, 'R')
              create_emgAveOnOffFig(emgEnv(emgChannel), fzR, aTime, figHandle);
            elseif strcmpi(limb, 'L')
              create_emgAveOnOffFig(emgEnv(emgChannel), fzL, aTime, figHandle);
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


    %%% WRITE EMG ON/OFF TIMES FROM PERRY, SCALED TO SUBJECT, TO SCREEN
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
end
return;
