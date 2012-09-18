%filename = optimizationExample.m
%optimizationExample calls the Matlab optimizer in order to find
%an optimal set of variables for prescribed controls used to control
%muscles in the Thelene2003 model.

% Import modeling classes
import org.opensim.modeling.*

% Turn up debug level so that exceptions due to typos etc. are handled gracefully
OpenSimObject.setDebugLevel(3);

% Read in osim model
osimModel = Model('arm26.osim');

% Ser optimization parameters
params.initialTime = 0;
params.finalTime   = 3;
params.lowerBound  = 0.01;
params.upperBound  = 0.01;
params.model   	   = osimModel;

% We must make variables that change on each iteration global since 
% fminsearch will pass the original copy of params to subsequent 
% iterations of the objective function. 
global stepCount bestSoFar;
stepCount = 0;
bestSoFar = Inf;

% Define initial LinearFunction coefficients, there must be two
% for each muscle.
initialCoefficients = 0.01*ones(1,2*osimModel.getMuscles.getSize());
disp(strcat('Initial Coefficients: '))
disp(initialCoefficients);



% Set optimization parameters
options = optimset('MaxIter',100);

% Control bounds must be specified as a column vector, every row
% corresponds to a control
lowerBound = 0.01*ones(osimModel.getMuscles.getSize(),1);
upperBound = 0.99*ones(osimModel.getMuscles.getSize(),1);

% Run the optimization
fmincon(@(coeffs0) optimizationExampleOptimizer(coeffs0,params),...
	initialCoefficients,[],[],[],[],lowerBound,upperBound,[],options);

