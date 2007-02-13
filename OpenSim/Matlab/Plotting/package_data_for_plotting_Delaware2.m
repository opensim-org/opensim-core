function package_data_for_plotting_Delaware2(subject, trial, suffix)

if nargin < 3
    suffix = 'packaged';
end

timeFile = sprintf('%s_%s_ik.mot', subject, trial);
grfFile = sprintf('%s_%s.mot', subject, trial);
frcFile = sprintf('ResultsCMC/%s_%s_Actuation_force.sto', subject, trial);
statesFile = sprintf('ResultsCMC/%s_%s_states.sto', subject, trial);
controlsFile = sprintf('ResultsCMC/%s_%s_controls.sto', subject, trial);
outFile = sprintf('%s_%s_%s.mot', subject, trial, suffix);

output.labels = {};
output.data = [];

disp(sprintf('Processing %s', timeFile'));
q = read_motionFile(timeFile);
time = q.data(:,1);
time_min = time(1);
time_max = time(end);
time = (time_min:0.001:time_max)';
output.labels = {'time'};
output.data = time;

disp(sprintf('Processing %s', grfFile'));
q = read_motionFile(grfFile);
I = find_columns_by_label(q.labels, '^ground_force_|^ground_torque_');
output.labels = {output.labels{:} q.labels{I}};
output.data(:,(end+1):(end+length(I))) = interp1(q.data(:,1),q.data(:,I),time);
time_min = max(time_min, q.data(1,1));
time_max = min(time_max, q.data(end,1));

disp(sprintf('Processing %s', frcFile'));
q = read_motionFile(frcFile);
notI = find_columns_by_label(q.labels, 'time|FX|FY|FZ|MX|MY|MZ|_reserve$');
I = setdiff(1:length(q.labels), notI);
newlabels = strcat(q.labels(I),'_frc');
output.labels = {output.labels{:} newlabels{:}};
output.data(:,(end+1):(end+length(I))) = interp1(q.data(:,1),q.data(:,I),time);
time_min = max(time_min, q.data(1,1));
time_max = min(time_max, q.data(end,1));

disp(sprintf('Processing %s', statesFile'));
q = read_motionFile(statesFile);
I = find_columns_by_label(q.labels, '.activation$');
if length(I)>0
	newlabels = strrep(q.labels(I),'.activation','');
else
	% Pre-restructure merge
	I = find_columns_by_label(q.labels, '.state_0$');
	newlabels = strrep(q.labels(I),'.state_0','');
end
output.labels = {output.labels{:} newlabels{:}};
output.data(:,(end+1):(end+length(I))) = interp1(q.data(:,1),q.data(:,I),time);
time_min = max(time_min, q.data(1,1));
time_max = min(time_max, q.data(end,1));

disp(sprintf('Processing %s', controlsFile'));
q = read_motionFile(controlsFile);
notI = find_columns_by_label(q.labels, 'time|FX.excitation|FY.excitation|FZ.excitation|MX.excitation|MY.excitation|MZ.excitation|_reserve.excitation');
I = setdiff(1:length(q.labels), notI);
newlabels = strrep(q.labels(I),'.excitation','_exc');
output.labels = {output.labels{:} newlabels{:}};
output.data(:,(end+1):(end+length(I))) = interp1(q.data(:,1),q.data(:,I),time);
time_min = max(time_min, q.data(1,1));
time_max = min(time_max, q.data(end,1));

disp(sprintf('Valid time range: %f - %f', time_min, time_max));
I = find(time_min <= time & time <= time_max);
output.data = output.data(I,:);
disp(sprintf('Writing to %s', outFile));
write_motionFile(output, outFile);
