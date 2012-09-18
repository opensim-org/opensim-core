function f = optimizationExampleOptimizer(coeffs,params)
% Runs model simulation for gien coefficients, returns integration
%	coeffs 	= initial set of control values
%	params 	= optimization parameters; simulation parameters and 
%			  pointers to instantiated OpenSim objects.
	
	% Import OpenSim modeling classes
	import org.opensim.modeling.*

	% Turn up debug level so that exceptions due to typos etc. are handled 
    % gracefully
	OpenSimObject.setDebugLevel(3);
	
	% Get access to step counter and current best velocity value.
	global stepCount bestSoFar;
	
	% Create a copy of the loaded model, passed as part of the params
    % structure.
    osimModel = Model(params.model);
    
    % Define initial muscle states
    for i = 1:osimModel.getMuscles().getSize()
        muscle = ActivationFiberLengthMuscle.safeDownCast(osimModel.getMuscles().get(i-1));
        if ~isempty(muscle)
            muscle.setDefaultActivation(0.5);
            muscle.setDefaultFiberLength(0.1);
        end
    end
    
    % Initialize system
    si = osimModel.initSystem();
    
    % Make sure muscle states are in equilibrium
    osimModel.equilibrateMuscles(si)
    
    % Create new controls structure, to be populated with values and passed
    % as a new default control set.
    newControls = ArrayDouble();
    
    % Populate controls array with control values
    for i = 1:osimModel.getMuscles().getSize()
       newControls.append(coeffs(i));
    end
    
    % Set model controls to the new controls, must get as SimTK::Vector    
    osimModel.setDefaultControls(newControls.getAsVector());
	
	% Create a manager, set manager parameters
	manager = Manager(osimModel);
	manager.setInitialTime(params.initialTime);
	manager.setFinalTime(params.finalTime);
    manager.setIntegratorAccuracy(1.0e-6)
    
    % In order to obtain velocity and acceleration information,
	% we must call OpenSimContext with a state and model pair.
	modelStatePair = OpenSimContext(si, osimModel);
	modelStatePair.realizeVelocity();
    
    % Tell the manager to run the integration
    manager.integrate(si);
    
    % Obtain velociy information
	massCenter = ArrayDouble.createVec3([0.0,0.0,0.0]);
	velocity   = ArrayDouble.createVec3([0.0,0.0,0.0]);
	osimModel.getBodySet().get('r_ulna_radius_hand').getMassCenter(massCenter);
	modelStatePair.realizeVelocity();
	osimModel.getSimbodyEngine().getVelocity(si, osimModel.getBodySet().get('r_ulna_radius_hand'), massCenter, velocity);
	velocityArray = ArrayDouble.getValuesFromVec3(velocity);
    
    % Define objective function output
	f = -velocityArray.getitem(0);
	
	% Update stepCount
	stepCount = stepCount + 1;
	
	% Check stem counter, save simulations when appropriate
    if stepCount == 23
		statesDegrees = manager.getStateStorage();
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
        randomSampleFile = strcat('testData',filesep,'OptimizationExample',...
            filesep,'Arm26_randomSample_states_degrees.sto');
		statesDegrees.print(randomSampleFile);
	elseif stepCount == 1
		statesDegrees = manager.getStateStorage();
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
        noActivationFile = strcat('testData',filesep,'OptimizationExample',...
            filesep,'Arm26_noActivation_states_degrees.sto');
		statesDegrees.print(noActivationFile);
        disp(strcat('Optimization (',num2str(stepCount),') f = ',num2str(f),', bestSoFar = ',num2str(bestSoFar)));
        disp(coeffs);
    elseif f < bestSoFar
        statesDegrees = manager.getStateStorage();
        osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
        bestSoFarFile = strcat('testData',filesep,'OptimizationExample',...
            filesep,'Arm26_bestSoFar_states_degrees.sto');
        statesDegrees.print(bestSoFarFile);
        bestSoFar = f;
        disp(strcat('Optimization (',num2str(stepCount),') f = ',num2str(f),', bestSoFar = ',num2str(bestSoFar)));
        disp(coeffs);
    end
	
end
