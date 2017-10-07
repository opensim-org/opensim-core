%% Example of how to convert Data Types between Matlab and OpenSim 

%% import Java Libraries
import org.opensim.modeling.*

%% Define an abriatory an OpenSim Vec3 instance
ov = Vec3(2,3,4);
% Convert to a Matlab Vector
mv = osimVec3(ov);
% Do an operation on the Matlab vector. 
n_v = mv - [0 2 0];
% Convert back to an OpenSim Vec3 Type
n_ov = osimVec3(n_v);