function q = read_mslFile(fname)

fin = fopen(fname, 'r');	
if fin == -1								
	error(['unable to open ', fname])		
end

q.names = {};
q.max_force = [];

ready = false;

while true
	nextline = fgetl(fin);
	if ~ ischar(nextline)
		break;
	end
	if strncmp(nextline, 'beginmuscle', length('beginmuscle'))
		name = sscanf(nextline, 'beginmuscle %s');
		if ~ strcmp(name, 'defaultmuscle')
			q.names = {q.names{:} name};
			ready = true;
		end
	elseif strncmp(nextline, 'max_force', length('max_force'))
		if ~ ready
			error('error');
		end
		q.max_force = [q.max_force sscanf(nextline, 'max_force %f')];
		ready = false;
	end
end
