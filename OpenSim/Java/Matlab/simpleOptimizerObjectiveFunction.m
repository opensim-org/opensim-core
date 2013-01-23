function f = simpleOptimizerObjectiveFunction(coeffs,params)
% Runs model simulation for gvien coefficients, returns integration
%	coeffs 	= initial set of control values
%	params 	= optimization parameters; simulation parameters and 
%			  pointers to instantiated OpenSim objects.
	
	% Import OpenSim modeling classes
	import org.opensim.modeling.*
	
	% Get access to step counter and current best velocity value.
	%global stepCount bestSoFar;
	
	% Get a reference to the model, as the model doesn't change in this example.
    osimModel = params.model;
    s = params.state;
    
	elbowFlexCoord = osimModel.updCoordinateSet().get('r_elbow_flex');
	elbowFlexCoord.setValue(s, coeffs);
    % Now equilibriate muscles at this configuration
    muscles = osimModel.getMuscles(); 
    nMuscles = muscles.getSize();
    
    for i = 0:nMuscles-1
			muscles.get(i).setActivation(s, 1.0);
			afl = ActivationFiberLengthMuscle.safeDownCast(muscles.get(i));
            afl.setFiberLength(s, .1);
    end
    
    % Make sure the muscles states are in equilibrium
	osimModel.equilibrateMuscles(s);
	
	bicShort = osimModel.getMuscles().get('BICshort');
    % Compute moment arm of BICshort, flip sign since the optimizer tries to minimize rather than maximize
    f = -bicShort.computeMomentArm(s, elbowFlexCoord);
        
	% Update stepCount
% 	stepCount = stepCount + 1;
	
	% Check step counter, save results when appropriate
%     if f < bestSoFar
% 
%         bestSoFar = f;
%         disp(strcat('Optimization (',num2str(stepCount),') f = ',num2str(f),', bestSoFar = ',num2str(bestSoFar)));
%         disp(coeffs);
%     end
	
end
