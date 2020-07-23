function [array, names, time] = loadFilterCropArray(filepath, lowpassFreq, ...
                                                    timeRange)
% LOADFILTERCROPARRAY
% ===================
%
% Inputs
% ------
%      filepath: Path to a STO or MOT file containing kinematic or force data.
%   lowpassFreq: Lowpass frequency (Hz) used to process data with a 4th-order 
%                Butterworth filter.
%     timeRange: 1x2 array containing start and end time points where the data
%                will be cropped.
%
% Outputs
% -------
%         array: Filtered and cropped data array. 
%         names: Data labels corresponding to the columns in 'array'.
%          time: Cropped time vector corresponding to the rows in 'array'.
                                              
import org.opensim.modeling.*;

% Load the data and convert it to a MATLAB array. You must have the OpenSim
% MATLAB utilities (found in <your-OpenSim-resources-path>\Code\Matlab\Utilities)  
% in your MATLAB path to use 'osimTableToStruct()'.
dataStruct = osimTableToStruct(TimeSeriesTable(filepath));
time = dataStruct.time;
dataStruct = rmfield(dataStruct, 'time');
names = fieldnames(dataStruct);
arrayRaw = cell2mat(struct2cell(dataStruct)');

% Construct a 4th-order lowpass Butterworth filter based on the 'lowpassFreq' 
% argument. Type "help butter" in the command window for more details.
timeStep = time(2) - time(1);
sampleRate = 1 / timeStep;
halfSampleRate = sampleRate / 2;
cutoffFreq = lowpassFreq / halfSampleRate;
[B,A] = butter(4, cutoffFreq, 'low');

% Filter all columns of the data array. 'filtfilt' filters in both the forward
% and reverse directions to eliminate "phase distortion", where data is shifted 
% in time due to the filtering process.
for i = 1:size(arrayRaw,2)
   arrayRaw(:,i) = filtfilt(B, A, arrayRaw(:,i));
end

% Extract subarrays based on 'timeRange' argument.
[~, istart] = min(abs(time-timeRange(1)));
[~, iend] = min(abs(time-timeRange(2)));
time = time(istart:iend,:);
array = arrayRaw(istart:iend,:);

end