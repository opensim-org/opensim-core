function write_motionFile(q, fname)

fid = fopen(fname, 'w');	
if fid == -1								
	error(['unable to open ', fname])		
end

if length(q.labels) ~= size(q.data,2)
	error('Number of labels doesn''t match number of columns')
end

if q.labels{1} ~= 'time'
	error('Expected ''time'' as first column')
end

fprintf(fid, 'name %s\n', fname);
fprintf(fid, 'datacolumns %d\n', size(q.data,2));
fprintf(fid, 'datarows %d\n', size(q.data,1));
fprintf(fid, 'range %f %f\n', min(q.data(:,1)), max(q.data(:,1)));
fprintf(fid, 'endheader\n');

for i=1:length(q.labels)
	fprintf(fid, '%20s\t', q.labels{i});
end
fprintf(fid, '\n');

for i=1:size(q.data,1)
	fprintf(fid, '%20.8f\t', q.data(i,:));
	fprintf(fid, '\n');
end

fclose(fid);
return;
