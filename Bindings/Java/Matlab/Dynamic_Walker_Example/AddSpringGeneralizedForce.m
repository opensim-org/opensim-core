% Import Java Library 
import org.opensim.modeling.*

% NOTE: In this sample code, we've used arbitrary parameters. Tweak them to get
% your desired result!

% Open the model
walkerModel = Model('../Model/DW2013_WalkerModelTerrain.osim');

% Change the name
walkerModel.setName('DW2013_WalkerModelTerrainAddSpringGeneralizedForce');

% Create the springs
rightSpring = SpringGeneralizedForce('RHip_rz');
leftSpring = SpringGeneralizedForce('LHip_rz');

% Set names of the forces.
rightSpring.setName('spring_right_hip');
leftSpring.setName('spring_left_hip');

% Set the params
stiffness = 10;
restLength = 0.01;
viscosity = 0.01;
rightSpring.setStiffness(stiffness);
rightSpring.setRestLength(restLength);
rightSpring.setViscosity(viscosity);
leftSpring.setStiffness(stiffness);
leftSpring.setRestLength(restLength);
leftSpring.setViscosity(viscosity);

% Add the forces to the model
walkerModel.addForce(rightSpring);
walkerModel.addForce(leftSpring);

% Print a new model file
walkerModel.print('../Model/DW2013_WalkerModelTerrainAddSpringGeneralizedForce.osim');
