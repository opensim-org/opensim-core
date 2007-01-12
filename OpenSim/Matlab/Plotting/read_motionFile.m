function q = read_motionFile(fname)
% Purpose:  This function reads a file in the format of a SIMM motion file
%           and returns a data structure
%
% Input:    fname is the name of the ascii datafile to be read 
%           ('character array') 
%
% Output:   q returns a structure with the following format:
%				q.labels 	= array of column labels
%				q.data 		= matrix of data
%				q.nr 		= number of matrix rows
%				q.nc 		= number of matrix columns
%
% ASA 12/03
% Modified by Eran Guendelman 09/06
% Open ascii data file for reading.
fid = fopen(fname, 'r');	
if fid == -1								
	error(['unable to open ', fname])		
end
% Process the file header;
% store # data rows, # data columns.
q.nr = 0; % Added to ensure that the q structures from reading a motion file
q.nc = 0; % are always the same, even if nr and nc are different orders in file.
nextline = fgetl(fid);	
while ~strncmpi(nextline, 'endheader', length('endheader'))
	if strncmpi(nextline, 'datacolumns', length('datacolumns'))
		q.nc = str2num(nextline(findstr(nextline, ' ')+1 : length(nextline)));
	elseif strncmpi(nextline, 'datarows', length('datarows'))
		q.nr = str2num(nextline(findstr(nextline, ' ')+1 : length(nextline)));
	elseif strncmpi(nextline, 'nColumns', length('nColumns'))
		q.nc = str2num(nextline(findstr(nextline, '=')+1 : length(nextline)));
	elseif strncmpi(nextline, 'nRows', length('nRows'))
		q.nr = str2num(nextline(findstr(nextline, '=')+1 : length(nextline)));
	end
	nextline = fgetl(fid);
end
% Process the column labels.
nextline = fgetl(fid);
if (all(isspace(nextline))) % Blank line, so the next one must be the one containing the column labels
	nextline = fgetl(fid);
end
q.labels = cell(1, q.nc);
for j = 1:q.nc
	[q.labels{j}, nextline] = strtok(nextline);
end
% Process the data.
% Note:  transpose is needed since fscanf fills columns before rows.
q.data = fscanf(fid, '%f', [q.nc, q.nr])';
fclose(fid);
return;
