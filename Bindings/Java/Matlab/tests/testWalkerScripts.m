%% Test for the walker model scripts
clear all; close all; clc;
%% Move to the User Functions directory
cd('Dynamic_Walker_Challenge/UserFunctions')

%% Get a Structure with each 'Add' script in the folder;
scriptNames = dir('Add*');

for i = 1 : length(scriptNames)
    try eval(['run ' scriptNames(i).name])

    catch Me
      disp(Me.message);
      disp(['Error at line ' num2str( Me.stack(1).line)]);
      error(['Error in file ' scriptNames(i).name ':']);
    end
end

% Test the Forward Simulation script
try run DesignMainStarter

catch Me
  disp(Me.message);
  disp(['Error at line ' num2str(Me.stack(1).line)]);
  error(['Error in file ' scriptNames(i).name ':']);
end
