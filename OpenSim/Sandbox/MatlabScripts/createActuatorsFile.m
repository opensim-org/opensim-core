function createActuatorsFile
%createActuatorsFile   Build and Print an OpenSim Actuator File from a Model
%
%  createActuatorsFile attempts to make a template actuators file that can
%  be used in Static Optimization, RRA and CMC. This tries to identify the
%  coordinates  that are connected to ground and place point or torque
%  actuators on translational or rotational coordinates, respectively. All
%  other coordiantes will get coordinate actuators. Any locked, prescribed
%  or constrained coordinates will be ignored.

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2012 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
%                                                                         %
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         % 
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %

% Author: James Dunne, Chris Dembia, Tom Uchida,
% Ajay Seth, Ayman Habib, Shrinidhi K. Lakshmikanth, Jen Hicks.

% Import OpenSim Libraries
import org.opensim.modeling.*

% open dialog boxes to select the model
[filename, pathname] = uigetfile('*.osim', 'Select an OpenSim Model File');

% get the model path
modelFilePath = fullfile(pathname,filename);

% Generate an instance of the model
model = Model(modelFilePath);

% Get the number of coordinates  and a handle to the coordainte set
nCoord = model.getCoordinateSet.getSize();
coordSet = model.getCoordinateSet();

% Evaluate the ground body and get the mass center
groundBodyName = model.getGround().getName();
groundJoint = model.getJointSet.get(0);

% Create some empty vec3's for later.
massCenter = Vec3();
axisValues = Vec3();

% Create an empty Force set
forceSet = ForceSet();
% get the state
state = model.initSystem();

optimalForce = 1;

%% Start going through the coordinates, creating an actuator for each
for iCoord = 0 : nCoord - 1

    % get a reference to the current coordinate
    coordinate = coordSet.get(iCoord);
    % If the coordinate is locked don't add an actuator.
    if coordinate.getLocked(state)
        continue
    end
    % If the coordinate is prescribed, don't add an actuator.
    if coordinate.isPrescribed(state)
        continue
    end
    % If the coodinate is constrained, don't add an actuator
    if coordinate.isConstrained(state)
        continue
    end

    % get the joint, parent and child names for the coordiante
    joint = coordinate.getJoint;
    parentName = joint.getParentFrame().getName(); 
    childName = joint.getChildFrame().getName();

    % If the coordinates parent body is connected to ground, we need to
    % add residual actuators (torque or point).
    if strmatch(parentName, model.getGround.getName() )

        % if the joint type is custom or free
        if strcmp(joint.getConcreteClassName(), 'CustomJoint') | strcmp(joint.getConcreteClassName(), 'FreeJoint')
               % get the coordainte motion type
               motion = char(coordinate.getMotionType);
               % to get the axis value for the coordiante, we need to drill
               % down into the coordinate transform axis
               eval(['concreteJoint = ' char(joint.getConcreteClassName()) '.safeDownCast(joint);'])
               sptr = concreteJoint.getSpatialTransform;
               for ip = 0 : 5
                  if strcmp(char(sptr.getCoordinateNames().get(ip)), char(coordinate.getName))
                        sptr.getTransformAxis(ip).getAxis(axisValues);
                        break
                  end
               end


               % Build a torque actuator for a rotational coordinate
               if strcmp(motion, 'Rotational')
                   newActuator = TorqueActuator(joint.getParentFrame(),...
                                         joint.getParentFrame(),...
                                         axisValues,...
                                         1);
    
               % Build a point actuator for a translational coordainte.
               else 
                    % make a new Point actuator
                    newActuator = PointActuator();
                    % set the body
                    newActuator.set_body(char(joint.getChildFrame().getName()))
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
                % make a new coordinate actuator for that coordinate
                newActuator = CoordinateActuator();
        end
        
    else % the coordinate is not connected to ground, and can just be a
         % coordinate actuator.
         newActuator = CoordinateActuator( char(coordinate.getName) );
    end

    % set the optimal force for that coordinate
    newActuator.setOptimalForce(optimalForce);        
    % set the actuator name
    newActuator.setName( coordinate.getName() );
    % set the optimal force for that coordinate
    newActuator.setOptimalForce(optimalForce);
    % set min and max controls
    newActuator.setMaxControl(Inf)
    newActuator.setMinControl(-Inf)

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
