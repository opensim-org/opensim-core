function [] = write_c3dMotMoments(jntmoments, time, fname)
% Purpose:  Writes joint moments to a motion file (fname) to facilitate
%           comparison of measured data with simulations.
%
% Input:   jntmoments is a structure containing the following data, 
%            formatted as follows:
%               *    .L(),   .label, .data
%                    .R(),   .label, .data
%           time is an array of time values corresponding to jntmoments
%           fname is the name of the file to be written.
%
% Output:   The file 'fname' is written to the current directory.
% ASA, 11-05


% Get number of frames.
nFrames = length(time);

% For each joint moment stored in jntcenters.R, get data.
% NOTE:  The C3D Server returns cell arrays;
%        these arrays are converted to numeric arrays.
for jntmomentNum = 1:length(jntmoments.R)
    for frameNum = 1:nFrames
        data(frameNum) = jntmoments.R(jntmomentNum).data{frameNum};
    end
    dataName = jntmoments.R(jntmomentNum).label;
    eval([dataName, '= data;']);  
end

% For each joint moment stored in jntcenters.L, get data.
% NOTE:  The C3D Server returns cell arrays;
%        these arrays are converted to numeric arrays.
for jntmomentNum = 1:length(jntmoments.L)
    for frameNum = 1:nFrames
        data(frameNum) = jntmoments.L(jntmomentNum).data{frameNum};
    end
    dataName = jntmoments.L(jntmomentNum).label;
    eval([dataName, '= data;']);  
end    

% Generate column labels for time and joint moments.
label{1} = 'time';
label{2} = 'RHipMoment';
label{3} = 'LHipMoment';
label{4} = 'RKneeMoment';
label{5} = 'LKneeMoment';
label{6} = 'RAnkleMoment';
label{7} = 'LAnkleMoment';
    
% Initialize 'motion file data matrix' for writing data of interest.
nRows = nFrames;
nCols = length(label);
motData = zeros(nRows, nCols);

% Write time array to data matrix.
motData(:, 1) = time;              

% Write joint moment data to data matrix.
for colNum = 2:nCols
    eval(['motData(:, colNum) = ', label{colNum}, ';']);
end

% Open file for writing.
fid = fopen(fname, 'w');
if fid == -1
    error(['unable to open ', fname])
end

% Write header.
fprintf(fid, 'name %s\n', fname);
fprintf(fid, 'datacolumns %d\n', nCols);
fprintf(fid, 'datarows %d\n', nRows);
fprintf(fid, 'range %d %d\n', time(1), time(nRows));
fprintf(fid, 'endheader\n\n');

% Write column labels.
for i = 1:length(label)
	fprintf(fid, '%20s\t', label{i});
end

% Write data.
for i = 1:nRows
    fprintf(fid, '\n'); 
	for j = 1:nCols
        fprintf(fid, '%20.8f\t', motData(i, j));
    end
end

fclose(fid);
return;

