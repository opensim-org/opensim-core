%% clear working space
clear all;close all;clc;
%% import opensim libraries
import org.opensim.modeling.*
% Adding a call to Model(). If Model() is not instantiated, the test fails.
m = Model();
% Set the number format to long. Numbers have 15 decimal places. 
format long
%% Define an abriatory an OpenSim Vec3 instance.
ov = Vec3(pi, -sqrt(2), inf);
% Convert to a Matlab Vector
mv = osimVec3ToArray(ov);
 % Convert back to an OpenSim Vec3
n_ov = osimVec3FromArray(mv);
% Convert back to an OpenSim Vec3 Type
n_mv = osimVec3ToArray(n_ov);

%% Test the changes for consistency
% Test OpenSim Vec3() instances
if n_ov().get(0) ~= ov().get(0) | n_ov().get(1) ~= ov().get(1) | n_ov().get(2) ~= ov().get(2)
    error('OpenSim Vec3s are different. Should be the same')
end
% Test Matlab Vector instances
if ~isequal(mv,n_mv)
    error('Matlab arrays are not equal between conversions')
end
disp('Test Passed!')



