% ==============================================================================
% =========================================================================
% =====
% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

% Turn up debug level so that exceptions due to typos etc. are handled gracefully
OpenSimObject.setDebugLevel(3);

% 		// get arguments from command line input
% 		std::string modelFileIn(argv[1]);
% 		std::string stoMotionFileIn(argv[2]);
% 		std::string modelFileOut(argv[3]);
%  get names of files using Matlab GUI into modelFileIn, stoMotionFileIn, modelFileOut

osimModel =Model(modelFileIn);
% Initialize the system
si = osimModel.initSystem();
			
% Create the coordinate storage object from the input .sto file
coordinateSto=Storage(stoMotionFileIn);

% First column is time
rTime=ArrayDouble();
coordinateSto.getTimeColumn(rTime);

rCoord=ArrayDouble();
% Rename the modified Model
osimModel.setName('modelWithPrescribedMotion');

% get coordinate set from model, and count the number of coordinates
modelCoordSet = osimModel.getCoordinateSet();
nCoords = modelCoordSet.getSize();

% for all coordinates in the model, create a function and prescribe
for( i=0:nCoords){
	tempCoord = modelCoordSet.get(i);

	% if a coordinate is later constrained (e.g. coordinate coupler constraint) exclude it to avoid conflict
	if (tempCoord.isConstrained(si)){
		continue;
	}

	% reset array because getDataColumn concatenates
	rCoord.setSize(0);

	coordinateSto.getDataColumn(tempCoord.getName(),rCoord);

	% create natural cubic spline function with time and data
	myfunction = NaturalCubicSpline(rTime.getSize(),rTime,rCoord,tempCoord.getName());
				
	tempCoord.setPrescribedFunction(myfunction);
}

%  Save the Modified Model to a file
osimModel.print(modelFileOut);
