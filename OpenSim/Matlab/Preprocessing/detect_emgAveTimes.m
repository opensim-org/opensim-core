function [onTimes, offTimes] = ...
          detect_emgAveTimes(emgData, threshold, onWindow, offWindow, cSim)
% Purpose:  Detects EMG on/off times corresponding to the EMG envelope 
%           stored in emgData based on three parameters:
%           threshold, onWindow, and offWindow.
%
% Input:  emgData is a structure corresponding to one EMG channel,  
%             with the following format, sampled at the analog frame rate:
%              *.emgRectified - array corresponding to
%                                 rectified, normalized EMG data,
%                                 sampled at analog frame rate
%              *.emgEnvelope  - array corresponding to 
%                                 rectified, filtered, normalized envelope,
%                                 sampled at analog frame rate
%              *.emgAve -       array corresponding to
%                                 ensemble-averaged EMG data,
%                                 sampled at same analog frame rate
%              *.emgSD -        array corresponding to 1SD of
%                                 ensemble-averaged EMG data,
%                                 sampled at same analog frame rate
%           threshold is the value of the normalized EMG envelope at which
%               an on/off event is detected
%           onWindow specifies the number of analog frames > threshold
%               needed for a muscle to be assumed 'on'
%           offWindow specifies the number of analog frames < threshold
%               needed for a muscle to be assumed 'off'
%           cSim is a structure from extract_c3dSimData()
%
% Output:   onTimes is an array of ON times, to nearest analog frame
%           offTimes is an array of OFF times, to nearest analog frame
%
% ASA, 9-05, revised 12-05


% Determine number of analog frames.
nAnalogFrames = cSim.video.nframes * cSim.analog.ratio;

% Initialize flags and counters.
onFlag = 0;         % assume muscle is inactive at start of trial
offFlag = 1;            
nOnEvents = 0;      % variables for counting # of on/off events
nOffEvents = 0;    
onTimes = [];       % arrays for storing on/off times
offTimes = [];

% Get EMG on/off times from normalized, low-pass filtered data.
for aFrameNum = 1:(nAnalogFrames-max(onWindow, offWindow));
    if offFlag
        if emgData.emgAve(aFrameNum) > threshold & ...
           min(emgData.emgAve(aFrameNum:aFrameNum+onWindow)) > threshold;
            onFlag = 1;
            offFlag = 0;
            nOnEvents = nOnEvents + 1;
            onTimes(nOnEvents) = ...
                aFrameNum/cSim.analog.rate + cSim.tDS;
        end
    elseif onFlag
        if emgData.emgAve(aFrameNum) < threshold & ...
           max(emgData.emgAve(aFrameNum:aFrameNum+offWindow)) < threshold;
            onFlag = 0;
            offFlag = 1;
            nOffEvents = nOffEvents + 1;
            offTimes(nOffEvents) = ...
                aFrameNum/cSim.analog.rate + cSim.tDS;
        end
    end
end  
return;                             
                                    