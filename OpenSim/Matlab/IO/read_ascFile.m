function geom = read_ascFile(fname)

fid = fopen(fname, 'r');
if fid == -1								
	error(['unable to open ', fname])		
end

nextline = fgetl(fid);	
if ~strcmp(nextline, 'NORM_ASCII')
    error('Expected NORM_ASCII header');
end

geom.nv = fscanf(fid, '%f', 1);
geom.np = fscanf(fid, '%f', 1);

geom.unknown = fscanf(fid, '%f', [6 1]);

vertices_and_normals = fscanf(fid, '%f', [6 geom.nv])';
geom.vertices = vertices_and_normals(:,1:3);
geom.normals = vertices_and_normals(:,4:6);

geom.poly = {};
for i=1:geom.np
    polysize = fscanf(fid, '%d', 1);
    geom.poly = {geom.poly{:} fscanf(fid, '%d', polysize)+1};
end
