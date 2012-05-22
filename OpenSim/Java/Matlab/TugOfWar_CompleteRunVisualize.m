% This script runs a forward simulation with the tug of war model created
% in the script OpenSimCreateTugOfWarModel.

% Rotate the view window to see the simulation run.

% First, import the classes from the jar file so that these can be called
% directly
import org.opensim.modeling.*

% Turn up debug level so that exceptions due to typos etc. are handled gracefully
OpenSimObject.setDebugLevel(3);

% Generate a new model object by loading the tug of war model from file
osimModel = Model('tug_of_war_muscles_controller.osim');

% Set up the visualizer to show the model and simulation
osimModel.setUseVisualizer(true)

% COMMENT
osimModel.initSystem()

%% DEFINE SOME PARAMETERS FOR THE SIMULATION

% Define the new tool object which will be run
tool = ForwardTool(); %--> to see all properties which can be set type 'methodsview(tool)'

% Define the model which the forward tool will operate on
tool.setModel(osimModel);

% Define the start and finish times for simulation
tool.setStartTime(0);
tool.setFinalTime(3);
tool.setSolveForEquilibrium(true);

% Define the name of the forward analysis
tool.setName('tugOfWar');

% Run the simulation
tool.run();  %--> rotate the view to see the tug of war simulation

