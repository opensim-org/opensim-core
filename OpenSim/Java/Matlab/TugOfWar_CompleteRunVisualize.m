% first import the classes from the jar file so that these can be called
% directly
import org.opensim.modeling.*

% Turn up debug level so that exceptions due to typos etc. are handled gracefully
OpenSimObject.setDebugLevel(3);

% generate a new model object
osimModel = Model('tug-of-war-all.osim');

osimModel.setUseVisualizer(true)

osimModel.initSystem()

%% DEFINE SOME PARAMETERS FOR THE SIMULATION

% define the new tool object which will be run
tool = ForwardTool(); %--> to see all properties which can be set type 'methodsview(tool)'

% define the model which the forward tool will operate on
tool.setModel(osimModel);
% define the start and finish times for simulation
tool.setStartTime(0);
tool.setFinalTime(4);
tool.setSolveForEquilibrium(true);

% define the name of the forward analysis
tool.setName('tugOfWar')

% run the simulation
tool.run();

