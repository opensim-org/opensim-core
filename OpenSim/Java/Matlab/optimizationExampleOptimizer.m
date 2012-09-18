function f = optimizationExampleOptimizer(coeffs,params)
% Runs model simulation for gien coefficients, returns integration
%	coeffs 	= initial set of coefficients, slop and intercepts for
%			  prescribed controllers in this case.
%	params 	= optimization parameters; simulation parameters and 
%			  pointers to instantiated OpenSim objects.
	
	% Import modeling classes
	import org.opensim.modeling.*

	% Turn up debug level so that exceptions due to typos etc. are handled gracefully
	OpenSimObject.setDebugLevel(3);
	
	% Get access to stepCount
	global stepCount bestSoFar;
	
	% Get parameter variables
	osimModel = Model(params.model);
    
    % Obtain muscle information
    muscles  = osimModel.getMuscles();
    nMuscles = muscles.getSize;

    % Define initial muscle states
    for i = 1:nMuscles
        muscle = ActivationFiberLengthMuscle.safeDownCast(muscles.get(i-1));
        if ~isempty(muscle)
            muscle.setDefaultActivation(0.5);
            muscle.setDefaultFiberLength(0.1);
        end
    end
    
    % Initialize system
    si = osimModel.initSystem();
    
    % Make sure muscle states are in equilibrium
    osimModel.equilibrateMuscles(si)
    
    % Update with new controls
    newControls = ArrayDouble();
    
    % Populate controls array with control values
    for i = 1:nMuscles
       newControls.append(coeffs(i));
    end
    
    newControlsVec = newControls.getAsVector();
    
    osimModel.setDefaultControls(newControlsVec)
	
	%///////////////////////////////
	%// CREATE MANAGER, INTEGRATE //
	%///////////////////////////////
	
	% Create a manager to run the simulation
    % The manager will use a default integrator
	manager = Manager(osimModel);
	manager.setInitialTime(params.initialTime);
	manager.setFinalTime(params.finalTime);
    
    % In order to obtain velocity and acceleration information,
	% we must call OpenSimContext with a state and model pair.
	modelStatePair = OpenSimContext(si, osimModel);
	modelStatePair.realizeVelocity();
    
    manager.integrate(si);
    
    %/////////////////////
	%// OBTAIN VELOCITY //
	%/////////////////////
	
	massCenter = ArrayDouble.createVec3([0.0,0.0,0.0]);
	velocity   = ArrayDouble.createVec3([0.0,0.0,0.0]);
	osimModel.getBodySet().get('r_ulna_radius_hand').getMassCenter(massCenter);
	modelStatePair.realizeVelocity();
	osimModel.getSimbodyEngine().getVelocity(si, osimModel.getBodySet().get('r_ulna_radius_hand'), massCenter, velocity);
	
	velocityArray = ArrayDouble.getValuesFromVec3(velocity);
	f = -velocityArray.getitem(0);
	
	% Update stepCount
	stepCount = stepCount + 1;
	
    % Check if this is the best optimization step
    
    
	% Check if first or last iteration, save simulations
    if stepCount == 23
		statesDegrees = manager.getStateStorage();
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.print('Arm26_randomSample_states_degrees.sto');
	elseif stepCount == 1
		statesDegrees = manager.getStateStorage();
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.print('Arm26_noActivation_states_degrees.sto');
    elseif f < bestSoFar
        statesDegrees = manager.getStateStorage();
        osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
        statesDegrees.print('Arm26_bestSoFar_states_degrees.sto');
        bestSoFar = f;
    end
    
    disp(strcat('Optimization (',num2str(stepCount),') f = ',num2str(f),', bestSoFar = ',num2str(bestSoFar)));
    disp(coeffs);
	
end