function constraints = read_excitationGuiOutput(fname)

fid = fopen(fname, 'r');
if fid == -1
    error(['unable to open ', fname])
end

c = textscan(fid, '%s %f %f %f %f');
n = length(c{1});

for i=1:n
    constraints.(c{1}{i}).min_value = [];
    constraints.(c{1}{i}).max_value = [];
    constraints.(c{1}{i}).t = [];
end

for i=1:n
    constraints.(c{1}{i}).min_value(end+1) = c{2}(i);
    constraints.(c{1}{i}).max_value(end+1) = c{3}(i);
    constraints.(c{1}{i}).t(end+1) = c{5}(i);
%    if c{5}(i) == 100
%        constraints.(c{1}{i}).min_value = [c{2}(i) constraints.(c{1}{i}).min_value];
%        constraints.(c{1}{i}).max_value = [c{3}(i) constraints.(c{1}{i}).max_value];
%        constraints.(c{1}{i}).t = [0 constraints.(c{1}{i}).t];
%    end
end
