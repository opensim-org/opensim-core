function [] = write_c3dMotEmgEnv(emgEnv, fname, ref_dataFormat)
% Purpose:  Writes EMG envelopes to a motion file (fname) to facilitate
%           comparison of measured EMG data with CMC-generated 
%           muscle activations.
%
% Input:   emgEnv is a structure containing the following data, 
%            formatted as follows:
%              *().limb    - limb corresponding to EMG channel #
%              *().muscle  - muscle corresponding to EMG channel #
%              *().emgRectified - matrix [time value] corresponding to
%                                 rectified, normalized EMG data,
%                                 sampled at analog frame rate
%              *().emgEnvelope  - matrix [time value] corresponding to 
%                                 rectified, filtered, normalized envelope,
%                                 sampled at analog frame rate
%           fname is the name of the file to be written.
%
% Output:   The file 'fname' is written to the current directory.
% ASA, 11-05

label{1}  = 'time';

% Get time array.
time = emgEnv(1).emgEnvelope(:, 1);

% For each channel of EMG data stored in emgEnv, get data and map to 
% suitable names for motion file column labels.
nChannels = length(emgEnv);
for channelNum = 1:nChannels
    recData = emgEnv(channelNum).emgRectified(:, 2);
    envData = emgEnv(channelNum).emgEnvelope(:, 2);

	prefix = get_emgColumnPrefix(emgEnv(channelNum).limb, emgEnv(channelNum).muscle, ref_dataFormat);

	recDataName = sprintf('%s_emgRec', prefix);
	envDataName = sprintf('%s_emgEnv', prefix);

    eval([recDataName, '= recData;']); 
    eval([envDataName, '= envData;']); 

	label = {label{:} recDataName envDataName};
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
