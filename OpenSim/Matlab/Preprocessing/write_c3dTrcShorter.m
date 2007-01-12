function [] = write_c3dTrcShorter(inFile, outFile, startTime, stopTime)
% Purpose:  Reads a *.trc file (inFile), extracts a subset of the data
%           from startTime to stopTime, and writes the extracted data
%           to a new, shorter *.trc file (outFile).
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

% Store header lines 1 & 2.
headerLine1 = fgetl(fid);
headerLine2 = fgetl(fid);

% Parse header line 3.
nextline = fgetl(fid);	
nElements = 8;
headerLine3 = cell(1, nElements);       
for j = 1:nElements
	[headerLine3{j}, nextline] = strtok(nextline);
end
DataRate = str2num(headerLine3{1});
CameraRate = str2num(headerLine3{2});
NumFrames = str2num(headerLine3{3});
NumMarkers = str2num(headerLine3{4});
Units = headerLine3{5};
OrigDataRate = str2num(headerLine3{6});
OrigDataStartFrame = str2num(headerLine3{7});
OrigNumFrames = str2num(headerLine3{8});

% Store header lines 4 & 5.
headerLine4 = fgetl(fid);
headerLine5 = fgetl(fid);

% Skip blank line.
nextline = fgetl(fid);	

% Read and store the data.
% Note:  transpose is needed since fscanf fills columns before rows.
ncIn = NumMarkers*3 + 2;        % 3 columns per marker, + frame, + time
nrIn = NumFrames;
inData = fscanf(fid, '%f', [ncIn, nrIn])';

% Get row indices corresponding to startTime and endTime.
inTime = inData(:, 2);
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

% Write header lines 1 & 2
fprintf(fid, [headerLine1, '\n']);
fprintf(fid, [headerLine2, '\n']);

% Write header line 3.
fprintf(fid, ['%7.1f    %7.1f    %7d    %7d    %7s    %7.1f    %7d    %7d\n'], ...
              DataRate, CameraRate, nrOut, NumMarkers, Units, ...
              OrigDataRate, OrigDataStartFrame, nrOut);

% Write header lines 4 & 5, blank line.
fprintf(fid, [headerLine1, '\n']);
fprintf(fid, [headerLine2, '\n']);
fprintf(fid, '\n');
            
% Write data.
for i = 1:nrOut
    fprintf(fid, '\t%d', i);
    fprintf(fid, '\t%.5f', inTime(startIndex + i - 1));
    for j = 3:ncOut
        fprintf(fid, '\t%.3f', inData(startIndex + i - 1, j));
    end
    fprintf(fid, '\n');
end
fclose(fid);
return;

