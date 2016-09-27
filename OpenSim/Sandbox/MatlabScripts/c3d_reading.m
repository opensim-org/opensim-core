
import org.opensim.modeling.*

%% Use a c3dAdapter to turn read a c3d file
adapter = C3DFileAdapter()

tables = adapter.read('test_walking.c3d');

%% get the Markers
markers = tables.get('markers');

% Print the (unrotated) markers to trc file
trcfileadapter = TRCFileAdapter();
trcfileadapter.write(markers,'test_walking.trc');

%% Get the force and convert the times series table to type double
forces = tables.get('forces');

% Flatten the data to a times series table
% THE BELOW LINE GENERATES AN EXCEPTION
forcesdouble = forces.flatten()  ;

% Write flattened forces table to .mot file format
stofileadapter = STOFileAdapter();
stofileadapter.write(forces,'test_walking_grf.mot')

%% Define a rotation matix
Rot = 90;          
rotationMatrix = [1,0,0;0,cos(Rot*pi/180),-(sin(Rot*pi/180));0,sin(Rot*pi/180),cos(Rot*pi/180)];
