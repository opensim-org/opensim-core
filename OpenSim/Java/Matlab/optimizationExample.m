%filename = optimizationExample.m
%optimizationExample calls the Matlab optimizer in order to find
%an optimal set of control variables for the arm26 muscle example.

% Import modeling classes
import org.opensim.modeling.*

% Read in osim model
modelFile = strcat('testData',filesep,'OptimizationExample',...
    filesep,'Arm26_Optimize.osim');
osimModel = Model(modelFile);

% Optimization parameters can be sent into the objective function either
% directly or as a structure.
params.initialTime = 0;
params.finalTime   = 0.25;
params.model   	   = osimModel;

% We must make variables that change on each iteration global since 
% fmincon will pass the original copy of params to subsequent 
% iterations of the objective function. 
global stepCount bestSoFar;
stepCount = 0;
bestSoFar = Inf;

% Create array for initial control values, the number of controls is equal
% to the number of muscles.
initialCoefficients = 0.01*ones(1,osimModel.getMuscles().getSize());
disp(strcat('Initial Coefficients: '))
disp(initialCoefficients);

% Set optimization parameters
% MaxIter - Maximum number of iterations
% TolFun  - Termination tolerance for function value
% Algorithm - the interior-point algorithm is recommended by Matlabs
% optimization toolbox literature.
% (see Matlab Optimization Toolbox documents for more options)
options = optimset('MaxIter',100,'TolFun',0.2,'Algorithm','interior-point');
%options = optimset('MaxIter',100,'TolFun',0.2,'Algorithm','active-set');


% Control bounds must be specified as a column vector, every row
% corresponds to a control
lowerBound = 0.01*ones(osimModel.getMuscles().getSize(),1);
upperBound = 0.99*ones(osimModel.getMuscles().getSize(),1);

% Run the optimization
fmincon(@(coeffs0) optimizationExampleOptimizer(coeffs0,params),...
	initialCoefficients,[],[],[],[],lowerBound,upperBound,[],options);

