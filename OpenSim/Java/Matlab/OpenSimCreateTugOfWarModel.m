% OpenSimCreateTugOfWarModel.m - script to perform the same model building and
% simulation tasks as the MainExample from the SDK examples

% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

% Turn up debug level so that exceptions due to typos etc. are handled gracefully
OpenSimObject.setDebugLevel(3);

% Create a blank model
model = Model();
model.setName('TugOfWar')

% Add DisplayGeometry to ground body. Model comes with a ground body already
ground = model.getGroundBody()
ground.addDisplayGeometry('ground.vtp');
ground.addDisplayGeometry('anchor1.vtp');
ground.addDisplayGeometry('anchor2.vtp');

% Create a block Body with associate dimensions, mass properties, and DisplayGeometry
blockMass = 20.0, blockSideLength = 0.1;
blockSideLength = 0.1
vec3Zero = ArrayDouble.createVec3(0);
block = Body()
block.setMass(20)
block.setMassCenter(vec3Zero)
block.setName('Block')
block.addDisplayGeometry('block.vtp');

% Create a FreeJoint to connect the block to ground
% Note that the constructor takes both bodies (ground, block)
%
locationInParentVec3 = ArrayDouble.createVec3([0, .05, 0])
blockToGround = FreeJoint('blockToGround', ground, locationInParentVec3, vec3Zero, block, vec3Zero, vec3Zero, false);

% set bounds on coordinates to make them reasonable, names could also be assigned here if needed
jointCoordinateSet=blockToGround.getCoordinateSet()
for c =0:2 
	jointCoordinateSet.get(c).setRange([-1.57, 1.57]);
	jointCoordinateSet.get(c+3).setRange([-1, 1]);
end

% add the block body to the model
model.addBody(block)

%Write the model to .osim file for loading later if needed
model.print('TugOfWar_FreeJoint.osim')
