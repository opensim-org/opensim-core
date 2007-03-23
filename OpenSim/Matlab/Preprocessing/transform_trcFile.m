function data = transform_trcFile(fnamein,fnameout,transform)

if nargin < 3
	transform = eye(3);
end

fin = fopen(fnamein, 'r');	
if fin == -1								
	error(['unable to open ', fnamein])		
end

fout = fopen(fnameout, 'wt');
if fout == -1								
	error(['unable to open ', fnameout])		
end

nextline = fgetl(fin);
fprintf(fout,'%s\n', nextline);
trcversion = sscanf(nextline, 'PathFileType %d');
if trcversion ~= 4
	disp('trc PathFileType is not 4, aborting');
	return;
end

nextline = fgetl(fin);
fprintf(fout,'%s\n', nextline);

nextline = fgetl(fin);
fprintf(fout,'%s\n', nextline);
values = sscanf(nextline, '%f %f %f %f');
numframes = values(3);
nummarkers = values(4);
numcolumns=3*nummarkers+2;

while true
	nextline = fgetl(fin);
	fprintf(fout,'%s\n', nextline);
	if (all(isspace(nextline)))
		break
	end
end

% READ
data = fscanf(fin, '%f', [numcolumns, numframes])';

% TRANSFORM
for m=1:nummarkers
	data(:,2+(3*(m-1))+(1:3)) = data(:,2+(3*(m-1))+(1:3)) * transform';
end

% WRITE
for r=1:numframes
	fprintf(fout, '%d\t', data(r,1));
	fprintf(fout, '%f\t', data(r,2:numcolumns));
	fprintf(fout, '\n');
end
