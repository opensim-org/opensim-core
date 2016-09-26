%%
import org.opensim.modeling.*


time_interval = 0.01;


model = Model('double_pendulum_markers.osim');


%% add a table reporter for the coordinates
coordinateReporter = TableReporter();
coordinateReporter.set_report_time_interval(time_interval)
coordinateReporter.updInput('inputs').connect(model.getCoordinateSet.get(0).getOutput('value'));
coordinateReporter.updInput('inputs').connect(model.getCoordinateSet.get(1).getOutput('value'));
model.addComponent(coordinateReporter);

%% add a table reporter for the markers
markerReporter = TableReporterVec3();
markerReporter.set_report_time_interval(time_interval);

nMarkers = model.getMarkerSet.getSize;
for iMarker = 0 : nMarkers - 1
    markerReporter.updInput('inputs').connect(model.getMarkerSet.get(iMarker).getOutput('location'), char(model.getMarkerSet.get(iMarker).getName) );

end
model.addComponent(markerReporter);


%% Test that the long labels are respected
expectedLongLabels = [{'/double_pendulum/pin1/q1/value'}...
                      {'/double_pendulum/pin2/q2/value'}];

coord_name = coordinateReporter.getInputNames;

for i = 1 : length(expectedLongLabels) 
    if strcmp(...
       coordinateReporter.getInput(coord_name.get(0)).getLongLabel(i-1),...
       expectedLongLabels(i));
    else
        error(['Long labels do not match']);
    end
end


%% Get and Set Alias 
% check that Alias is empty
n = coordinateReporter.getInputNames;

if isempty(char(coordinateReporter.getInput(n.get(0)).getAlias(0)))
    
else
    error('Alias value should be empty')
end

% set all Alias
coordinateReporter.updInput(n.get(0)).setAlias('q1')

if strcmp(char(coordinateReporter.getInput(n.get(0)).getAlias(0)), 'q1') &&...
    strcmp(char(coordinateReporter.getInput(n.get(0)).getAlias(1)), 'q1')
else
    error('Alias should all be the same (q1 and q1)')
end
    
% Set the second Alias    
coordinateReporter.updInput(n.get(0)).setAlias(1,'q2')

if strcmp(char(coordinateReporter.getInput(n.get(0)).getAlias(0)), 'q1') &&...
    strcmp(char(coordinateReporter.getInput(n.get(0)).getAlias(1)), 'q2')
else
    error('Alias should all be different (q1 and q2)')
end

%% Test that the short labels are respected
expectedShortLabels = [{'NewMarker'} {'NewMarker_0'} {'NewMarker_1'} {'NewMarker_2'}];

name = markerReporter.getInputNames;

for i = 1 : length(expectedShortLabels) 
    if strcmp(markerReporter.getInput(name.get(0)).getShortLabel(i-1),...
        expectedShortLabels(i));
    else
        error(['Short labels do not match']);
    end
end

%% Simulate.
state = model.initSystem();
manager = Manager(model);
manager.setInitialTime(0); manager.setFinalTime(10.0);
manager.integrate(state);

%% Get tables from the reporters
coordinatetable = coordinateReporter.getReport;
markertable = markerReporter.getReport;

%% Print the rotated markers to trc file.
stofileadapter = STOFileAdapterVec3();
stofileadapter.write(markertable,'pendulum_markers.sto');



