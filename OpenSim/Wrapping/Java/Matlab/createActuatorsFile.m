function createActuatorsFile(varargin)
%createActuatorsFile   Build and Print an OpenSim Actuator File from a Model
%
%  createActuatorsFile attempts to build a template actuators file that can
%  be used in Static Optimization, RRA and CMC. This tries to identify the
%  coordinates  that are connected to ground and place point or torque
%  actuators on translational or rotational coordinates, respectively. All
%  other coordiantes will get coordinate actuators. Any locked, prescribed 
%  or constrained coordinates will be ignored.
%
%  createActuatorsFile(path2file) where path2file is the full path to a
%  model file (string). A display prompt will be generated to input optimal
%  force values. 
%  
%  createActuatorsFile(path2file, {'residual' x 'reserve' y}) where x and y
%  are optima force integers, any residual actuators will be set to x, 
%  reserve actuatorsto y. Use 'all' to set all actuators to the same value 
%  ie {'all' x}

%   Author: James Dunne
%   Tested on; Windows 7, Matlab 2014 (64bit), OpenSim 3.2 (64bit), 3.2 
%   (32 bit) 
%   Models; gait2392 (with locked joints), gait2354, gait2354 w/Patella,
%   Double Pendulum. 

% Import OpenSim Libraries
import org.opensim.modeling.*

% if NO inputs are given, open dialog boxes to select the model and input
% the optimal force values. 
for i = 1 : nargin
    % if a string, determine if its a valid file path. If not, bring up a
    % dialog box. 
    if ischar(varargin{i})
        if exist(varargin{i},'file') == 0 
            warning(['File ' varargin{i} ' is not a valid model'])
            [filename, pathname] = uigetfile('*.osim', 'Select an OpenSim Model File');
        else
            [pathname,filename,ext] = fileparts(varargin{i});
            filename = [filename ext];
        end
    end
    % if input string a numeric, next value will be a rotation cell array
    if iscell(varargin{i})
        
        if  mod(length(varargin{i}),2)
           error('input incorrect. Must be in the form of a string AND a number') 
        end
            
        for iCell = 1 : length(varargin{i})
            if ischar(varargin{i}{iCell}) 
                if strcmpi(varargin{i}{iCell},'residual')
                        if isnumeric(varargin{i}{iCell+1})
                            residualOptimalForce = varargin{i}{iCell+1};
                        else
                            error(['value after ' varargin{i}{iCell} ' must be a Optimal Force (number)'])
                        end
                elseif strcmpi(varargin{i}{iCell},'reserve')
                        if isnumeric(varargin{i}{iCell+1})
                            reserveOptimalForce = varargin{i}{iCell+1}; 
                        else
                            error(['value after ' varargin{i}{iCell} 'must be a Optimal Force (number)'])
                        end
                elseif strcmpi(varargin{i}{iCell},'all')
                        if isnumeric(varargin{i}{iCell+1})
                            residualOptimalForce = varargin{i}{iCell+1};
                            reserveOptimalForce = varargin{i}{iCell+1};
                        else
                            error(['value after ' varargin{i}{iCell} ' must be a Optimal Force (number)'])
                        end
                else
                     error([varargin{i}{iCell} ' is not a valid input argument. either residual, reserve or all'])
                end
            end                    
        end
    end
end

if ~exist('filename', 'var')
    [filename, pathname] = uigetfile('*.osim', 'Select an OpenSim Model File');
end

if ~exist('residualOptimalForce', 'var') | ~exist('reserveOptimalForce', 'var')
    % Prompt the user to input the Optimal Force.
    prompt = {'Optimal forces for any Residuals:','Optimal forces for all Reserves'};
    dlg_title = 'Input';
    num_lines = 1;
    def = {'2','1'};
    answer = inputdlg(prompt,dlg_title,num_lines,def);
    % allocate these into some variables.
    residualOptimalForce = str2num(answer{1});
    reserveOptimalForce  = str2num(answer{2});
end
  
    
% Prompt the user to select a model
modelFilePath = fullfile(pathname,filename);

% Generate an instance of the model
myModel = Model(modelFilePath);

% Get the number of coordinates  and a handle to the coordainte set
nCoord = myModel.getCoordinateSet.getSize;
coordSet = myModel.getCoordinateSet;

% Evaluate the ground body and get the mass center
groundBodyName = myModel.getGroundBody.getName;
groundJoint = myModel.getJointSet.get(0);

% Create some empty vec3's for later. 
massCenter = Vec3();
axisValues = Vec3();

% Create an empty Force set
forceSet = ForceSet();
% get the state
state = myModel.initSystem();



%% Start going through the coordinates, creating an actuator for each
for iCoord = 0 : nCoord - 1
    
    % get a reference to the current coordinate
    myCoord = coordSet.get(iCoord);
    % If the coordinate is locked don't add an actuator.
    if myCoord.getLocked(state)
        continue
    end
    % If the coordinate is prescribed, don't add an actuator. 
    if myCoord.isPrescribed(state)
        continue 
    end
    % If the coodinate is constrained, don't add an actuator
    if myCoord.isConstrained(state)
        continue
    end
    
    joint = myCoord.getJoint;
    joint.getBody.getMassCenter(massCenter)
    
    % If the coordinates parent body is connected to ground, we need to
    % add residual actuators (torque or point). 
    if strmatch(joint.getParentName, myModel.getGroundBody.getName )
        
           % if the joint type is custom or free
        if strcmp(joint.getConcreteClassName, 'CustomJoint') | strcmp(joint.getConcreteClassName, 'FreeJoint') 
               % get the coordainte motion type 
               motion = char(myCoord.getMotionType);
               % to get the axis value for the coordiante, we need to drill 
               % down into the coordinate transform axis
               eval(['concreteJoint = ' char(joint.getConcreteClassName) '.safeDownCast(joint);'])
               sptr = concreteJoint.getSpatialTransform; 
               for ip = 0 : 5
                  if strcmp(char(sptr.getCoordinateNames().get(ip)), char(myCoord.getName))
                        sptr.getTransformAxis(ip).getAxis(axisValues);
                        break
                  end
               end


               % Build a torque actuator for a rotational coordinate
               if strcmp(motion, 'Rotational')
                   newActuator = TorqueActuator(joint.getBody,...
                                         joint.getParentBody,...
                                         axisValues,...
                                         1);
                   newActuator.setName(char(myCoord.getName))                  

               % Build a point actuator for a translational coordainte.  
               elseif strcmp(motion, 'Translational')
                    % build a new Point actuator
                    newActuator = PointActuator();
                    % Set the actuator Name
                    newActuator.setName(char(myCoord.getName)) 
                    % set the body
                    newActuator.set_body(char(concreteJoint.getBody))
                    % set <point>      -0.07243760       0.00000000       0.00000000 </point>
                    newActuator.set_point(massCenter)
                    % set <point_is_global> false </point_is_global>
                    newActuator.set_point_is_global(0)
                    % set <direction>      -0.00000000       1.00000000      -0.00000000 </direction>
                    newActuator.set_direction(axisValues)
                    % set <force_is_global> true </force_is_global>
                    newActuator.set_force_is_global(1)
               end
        else % if the joint type is not free or custom, just add coordinate actuators 
                % build a new coordinate actuator for that coordinate
                newActuator = CoordinateActuator( char(myCoord.getName) );
                % set the actuator name
                newActuator.setName( char(myCoord.getName) )
                % set the optimal force for that coordinate
                newActuator.setOptimalForce(reserveOptimalForce);
                % set min and max controls
                newActuator.setMaxControl(Inf)
                newActuator.setMinControl(-Inf)
        end
    % set the optimal force for that coordinate
    newActuator.setOptimalForce(residualOptimalForce);
    % set min and max controls
    newActuator.setMaxControl(Inf)
    newActuator.setMinControl(-Inf)
    
    else
            % build a new coordinate actuator for that coordinate
            newActuator = CoordinateActuator( char(myCoord.getName) );
            % set the actuator name
            newActuator.setName( char(myCoord.getName) )
            % set the optimal force for that coordinate
            newActuator.setOptimalForce(reserveOptimalForce);
            % set min and max controls
            newActuator.setMaxControl(Inf)
            newActuator.setMinControl(-Inf)
    end

    % append the new acuator onto the empty force set
    forceSet.cloneAndAppend(newActuator);
end

%% get the parts of the file path
[pathname,filename,ext] = fileparts(modelFilePath);
% define the new print path
printPath = fullfile(pathname, [filename '_actuators.xml']);
% print the actuators xml file
forceSet.print(printPath);
% Display printed file
display(['Printed actuators to ' printPath])

end
