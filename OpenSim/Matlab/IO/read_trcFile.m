function q = read_trcFile(fname)

fin = fopen(fname, 'r');	
if fin == -1								
	error(['unable to open ', fname])		
end

nextline = fgetl(fin);
trcversion = sscanf(nextline, 'PathFileType %d');
if trcversion ~= 4
	disp('trc PathFileType is not 4, aborting');
	return;
end

nextline = fgetl(fin);

nextline = fgetl(fin);
values = sscanf(nextline, '%f %f %f %f');
numframes = values(3);
q.nummarkers = values(4);
numcolumns=3*q.nummarkers+2;

nextline = fgetl(fin);
q.labels = cell(1, numcolumns);
[q.labels{1}, nextline] = strtok(nextline); % should be Frame#
[q.labels{2}, nextline] = strtok(nextline); % should be Time
for i=1:q.nummarkers
	[markername, nextline] = strtok(nextline);
	q.labels{2+3*(i-1)+1} = [markername '_tx'];
	q.labels{2+3*(i-1)+2} = [markername '_ty'];
	q.labels{2+3*(i-1)+3} = [markername '_tz'];
end

while true
	nextline = fgetl(fin);
	if (all(isspace(nextline)))
		break
	end
end

% READ
q.data = fscanf(fin, '%f', [numcolumns, numframes])';
