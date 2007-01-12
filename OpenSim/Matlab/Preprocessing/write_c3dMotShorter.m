function [] = write_c3dMotShorter(inFile, outFile, startTime, stopTime)
% Purpose:  Reads a motion file (inFile), extracts a subset of the data
%           from startTime to stopTime, and writes the extracted data
%           to a new, shorter motion file (outFile).
%
% Input:    inFile is the name of the file to be read.
%           outFile is the name of the file to be written.
%           startTime and stopTime are the times (i.e., 1st column values)
%               to start and stop extracting data (inclusive).
%
% Output:   The file 'outFile' is written to the current directory.
% ASA, 10-05


%%% PROCESS INPUT FILE
% Open input file for reading.
fid = fopen(inFile, 'r');	
if fid == -1								
	error(['unable to open ', inFile])		
end

% Read the file header; 
% store # data rows, # data columns in input file.
nextline = fgetl(fid);	
while ~strncmpi(nextline, 'endheader', length('endheader'))
    if strncmpi(nextline, 'datacolumns', length('datacolumns')) 
		ncIn = str2num(nextline(findstr(nextline, ' ')+1 : length(nextline)));
	elseif strncmpi(nextline, 'datarows', length('datarows'))
		nrIn = str2num(nextline(findstr(nextline, ' ')+1 : length(nextline)));
	end
	nextline = fgetl(fid);
end

% Read and store the column labels.
nextline = fgetl(fid);			% read blank line before column labels
nextline = fgetl(fid);			% read line containing column labels
labels = cell(1, ncIn);
for j = 1:ncIn
	[labels{j}, nextline] = strtok(nextline);
end

% Read and store the data.
% Note:  transpose is needed since fscanf fills columns before rows.
inData = fscanf(fid, '%f', [ncIn, nrIn])';

% Get row indices corresponding to startTime and endTime.
inTime = inData(:, 1);
startIndex = max(find(inTime <= startTime));
stopIndex = min(find(inTime >= stopTime));
if isempty(startIndex)
    error('invalid start time')
end
if isempty(stopIndex)
    error('invalid stop time')
end

% Note # data rows, # data columns in output file.
ncOut = ncIn;
nrOut = stopIndex - startIndex + 1;
fclose(fid);


%%% PROCESS OUTPUT FILE
% Open output file for writing.
fid = fopen(outFile, 'w');
if fid == -1
    error(['unable to open ', outFile])
end

% Write header.
fprintf(fid, 'name %s\n', outFile);
fprintf(fid, 'datacolumns %d\n', ncOut);
fprintf(fid, 'datarows %d\n', nrOut);
fprintf(fid, 'range %d %d\n', inTime(startIndex), inTime(stopIndex));
fprintf(fid, 'endheader\n\n');

% Write column labels.
for i = 1:length(labels)
	fprintf(fid, '%20s\t', labels{i});
end

% Write data.
for i = 1:nrOut
    fprintf(fid, '\n'); 
	for j = 1:ncOut
        fprintf(fid, '%20.8f\t', inData(startIndex + i - 1, j));
    end
end

fclose(fid);
return;

