function [func] = InteractiveHopperSettings(name)
% INTERACTIVEHOPPERSETTINGS
%   Built-in settings for the InteractiveHopper GUI example

% Throw error if invalid setting name is passed
try 
   func = getFunction(name);
catch
   ME = MException('InteractiveHopperSettings:invalidSettingName', ...
                   'Input ''%s'' is not a valid InteractiveHopper setting', name);
   throw(ME);
end

% GETFUNCTION
%   Return settings function handle for specified function name
function [func] = getFunction(name)

switch name
    case 'averageJoe'
        func = @averageJoe;
    case 'arnold'
        func = @arnold;
    case 'katieLedecky'
        func = @katieLedecky;
    case 'passive'
        func = @passive;
    case 'activeControl'
        func = @activeControl;
    case 'activePropMyo'
        func = @activePropMyo;
end

% PASSIVE
%   Function to determine mass and spring stiffness of the passive device. 
%   In the InteractiveHopper GUI example, 'param' reflects the prescribed 
%   slider value for the passive device.
function [mass,stiffness] = passive(param)

stiffness = param*100;
mass = param*0.1;

% ACTIVECONTROL
%   Function to determine mass and maximum tension of the active device 
%   (with a unique user specified control). In the InteractiveHopper GUI 
%   example, 'param' reflects the prescribed slider value for the passive device.
function [mass,maxTension] = activeControl(param)

maxTension = param*10;
mass = param*0.1;

% ACTIVEPROPMYO
%   Function to determine mass and maximum tension of the active device 
%   (with a proportional myoelectric controller). In the InteractiveHopper GUI 
%   example, 'param' reflects the prescribed slider value for the passive device.
function [mass,gain] = activePropMyo(param)

gain = (param/100)+0.5;
mass = param*0.1;

% AVERAGEJOE
%   Settings for "The Average Joe" muscle in the InteractiveHopper GUI
%   example.
function [maxIsometricForce,tendonStiffness,tendonSlackLength,mass] = averageJoe()

maxIsometricForce = 4000.0;
tendonStiffness = 28.1;
tendonSlackLength = 0.25;
mass = 5;

% ARNOLD
%   Settings for "The Arnold" muscle in the InteractiveHopper GUI
%   example.
function [maxIsometricForce,tendonStiffness,tendonSlackLength,mass] = arnold()

maxIsometricForce = 5000.0;
tendonStiffness = 30;
tendonSlackLength = 0.15;
mass = 7.5;

% KATIELEDECKY
%   Settings for "The Katie Ledecky" muscle in the InteractiveHopper GUI
%   example.
function [maxIsometricForce,tendonStiffness,tendonSlackLength,mass] = katieLedecky()

maxIsometricForce = 3500.0;
tendonStiffness = 25;
tendonSlackLength = 0.30;
mass = 2.5;