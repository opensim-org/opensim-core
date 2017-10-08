%% clear working space
clear all;close all;clc;
%% import opensim libraries
import org.opensim.modeling.*

table = DataTableVec3();
labels = StdVectorString();
labels.add('marker1');labels.add('marker2');
labels.add('marker3');labels.add('marker4');
table.setColumnLabels(labels);
%% Define an abriatory an OpenSim Vec3 instance
ov = Vec3(2,3,4);
% Convert to a Matlab Vector
% mv = osimVec3(ov);
% 
% % Convert back to an OpenSim Vec3
% n_ov = osimVec3(mv);
% % Convert back to an OpenSim Vec3 Type
% n_mv = osimVec3(n_ov);

%% Test the changes for consistency
% Test OpenSim Vec3() instances
% if n_ov().get(0) ~= ov().get(0) | n_ov().get(1) ~= ov().get(1) | n_ov().get(2) ~= ov().get(2)
%     error('OpenSim Vec3s are different. Should be the same')
% end
% % Test Matlab Vector instances
% if ~isequal(mv,n_mv)
%     error('Matlab arrays are not equal between conversions')
% end
disp('hello')



