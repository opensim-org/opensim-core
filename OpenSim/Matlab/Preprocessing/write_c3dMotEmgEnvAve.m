function [] = write_c3dMotEmgEnvAve(emgEnv, fname, ref_dataFormat)
% Purpose:  Writes EMG envelopes to a motion file (fname) to facilitate
%           comparison of measured EMG data with CMC-generated 
%           muscle activations.
%
% Input:   emgEnv is a structure containing the following data, 
%            formatted as follows:
%              *().limb    - limb corresponding to EMG channel #
%              *().muscle  - muscle corresponding to EMG channel #
%              *().emgRectified - array corresponding to
%                                 rectified, normalized EMG data,
%                                 sampled at analog frame rate
%              *().emgEnvelope  - array corresponding to 
%                                 rectified, filtered, normalized envelope,
%                                 sampled at analog frame rate
%              *().emgAve -       array corresponding to
%                                 ensemble-averaged EMG data,
%                                 sampled at same analog frame rate
%              *().emgSD -        array corresponding to 1SD of
%                                 ensemble-averaged EMG data,
%                                 sampled at same analog frame rate
%              *().time -         array of time values corresponding to
%                                 simulateable segment

%           fname is the name of the file to be written.
%
% Output:   The file 'fname' is written to the current directory.
% ASA, 11-05, revised 12-05

label{1}  = 'time';

% Get time array.
time = emgEnv(1).time;

% For each channel of EMG data stored in emgEnv, get data and map to 
% suitable names for motion file column labels.
nChannels = length(emgEnv);
for channelNum = 1:nChannels
    recData = emgEnv(channelNum).emgRectified;
    envData = emgEnv(channelNum).emgEnvelope;
    aveData = emgEnv(channelNum).emgAve;
    sdData = emgEnv(channelNum).emgSD;

	prefix = get_emgColumnPrefix(emgEnv(channelNum).limb, emgEnv(channelNum).muscle, ref_dataFormat);

	recDataName = sprintf('%s_emgRec', prefix);
	envDataName = sprintf('%s_emgEnv', prefix);
	aveDataName = sprintf('%s_emgAve', prefix);
	sdpDataName = sprintf('%s_emgSDp', prefix);
	sdmDataName = sprintf('%s_emgSDm', prefix);

    eval([recDataName, '= recData;']); 
    eval([envDataName, '= envData;']); 
    eval([aveDataName, '= aveData;']); 
    eval([sdpDataName, '= aveData + sdData;']); 
    eval([sdmDataName, '= aveData - sdData;']); 

	label = {label{:} recDataName envDataName aveDataName sdpDataName sdmDataName};
end

% Initialize 'motion file data matrix' for writing data of interest.
nRows = length(time);
nCols = length(label);
motData = zeros(nRows, nCols);

% Write time array to data matrix.
motData(:, 1) = time;              

% Write EMG data to data matrix.
for colNum = 2:nCols
    eval(['motData(:, colNum) = ', label{colNum}, ';']);
end

q.labels = label;
q.data = motData;
write_motionFile(q, fname);
return;
