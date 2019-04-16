function J = walker_simulation_objective_function(vod)
%% walker_simulation_objective_function.m
% Function for running forward simulation of a walker model and computing 
% the objective function (Pelvis X translation). The input to this function 
% is a 12x1 vector of initial coordinate value and speed states for a 
% dynamic walker model. See Matlab script walkerOptimization.m for use.  

% Written by James Dunne, Carmchael Ong, Tom Uchida, Ajay Seth. 

% Define Global variables. 
global model translation_store vectorOfDefaults_store;

% Import OpenSim libraries 
import org.opensim.modeling.*

% Store the vector of defaults that the optimizer chooses
vectorOfDefaults_store = [vectorOfDefaults_store;vod']; 

% Set the time for the simulation to take place  
finalTime = 10;

% Update model default coordinate values and speeds with optimizer defined 
% values.
model.getCoordinateSet().get('Pelvis_tx').setDefaultValue(vod(1));
model.getCoordinateSet().get('Pelvis_tx').setDefaultSpeedValue(vod(2));
model.getCoordinateSet().get('Pelvis_ty').setDefaultValue(vod(3));
model.getCoordinateSet().get('Pelvis_ty').setDefaultSpeedValue(vod(4));
model.getCoordinateSet().get('LHip_rz').setDefaultValue(vod(5));
model.getCoordinateSet().get('LHip_rz').setDefaultSpeedValue(vod(6));
model.getCoordinateSet().get('RHip_rz').setDefaultValue(vod(7));
model.getCoordinateSet().get('RHip_rz').setDefaultSpeedValue(vod(8));
model.getCoordinateSet().get('LKnee_rz').setDefaultValue(vod(9));
model.getCoordinateSet().get('LKnee_rz').setDefaultSpeedValue(vod(10));
model.getCoordinateSet().get('RKnee_rz').setDefaultValue(vod(11));
model.getCoordinateSet().get('RKnee_rz').setDefaultSpeedValue(vod(12));

% Initialize the model and get the state
s = model.initSystem();

% Simulate with the new dafault initial values
manager = Manager(model);
manager.initialize( s );
manager.integrate( finalTime );

% Get the managers state table (an internal table that holds all the states
% of a simulation). 
st = manager.getStatesTable();

%% Calculate the objective function
% When successful, the walker model will take steps down the platform and
% have not fallen by the end of the simulation time. We will use the Pelvis 
% X value at the end of the simulation as the measure of success.  

% Get the Pelvis X value (the pelvis x translation) at the end of the
% trial. This value will be higher if the model does not fall during the 
% simulation. 

x_translation = st.getDependentColumn('/jointset/PelvisToPlatform/Pelvis_tx/value').get(st.getNumRows()-1);
% Store the value of the pelvis translation. 
translation_store = [translation_store;x_translation];
% Display the X translation on in the command window. 
disp(['Walker Model Pelvis X Translation: ' num2str( x_translation) ' meters'])


% Compute the objective function. Since we are using fminunc, which is trying
% to find the minimum, we negate the pelvis translation value.  
J = -x_translation;

end
