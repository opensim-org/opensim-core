function createActuatorsFile(modelpath)
%% Function to generate a generic OpenSim Actuator File from a Model by
%  identifying the coordinates that are connected to ground and placing
%  point or torque actuators on translational or rotational coordinates,
%  respectively. All other coordiantes will get coordinate actuators.
%  Any constrained coordinates will be ignored.
%  File is Printed to the same folder as the selected Model.
%
% Inputs - 'modelpath', path to an OSIM file (string)
%
% e.g. createActuatorsFile('myInputModel.osim')

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2019 Stanford University and the Authors             %
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

% Author: James Dunne, Tom Uchida.

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% If no model is input, get a path to one.
display('Loading the model...');
if nargin < 1
    [pathname,filename] = uigetfile('*.osim', 'PSelect an OpenSim Model File');
    modelpath = fullfile(filename,pathname);
elseif nargin > 1
    error('Too many inputs to function. Input is Model path');
end

%% Instantiate the model
model = Model(modelpath);

%% Instantiate the underlying computational System and return a handle to the State
state = model.initSystem();

%% Get the number of coordinates  and a handle to the coordainte set
coordSet = model.getCoordinateSet();
nCoord = coordSet.getSize();

%% Instantiate some empty vec3's for later.
massCenter = Vec3();
axisValues = Vec3();

%% Instantiate an empty Force set
forceSet = ForceSet();

%% Set the optimal force
optimalForce = 1;

%% Start going through the coordinates, creating an actuator for each
for iCoord = 0 : nCoord - 1

    % get a reference to the current coordinate
    coordinate = coordSet.get(iCoord);
    % If the coodinate is constrained (locked or prescribed), don't
    % add an actuator
    if coordinate.isConstrained(state)
        continue
    end

    % get the joint, parent and child names for the coordiante
    joint = coordinate.getJoint();
    parentName = joint.getParentFrame().getName();
    childName = joint.getChildFrame().getName();

    % If the coordinates parent body is connected to ground, we need to
    % add residual actuators (torque or point).
    if strcmp(parentName, model.getGround.getName() )

        % Custom and Free Joints have three translational and three
        % rotational coordinates.
        if strcmp(joint.getConcreteClassName(), 'CustomJoint') || strcmp(joint.getConcreteClassName(), 'FreeJoint')
               % get the coordainte motion type
               motion = char(coordinate.getMotionType());
               % to get the axis value for the coordinate, we need to drill
               % down into the coordinate transform axis
               eval(['concreteJoint = ' char(joint.getConcreteClassName()) '.safeDownCast(joint);'])
               sptr = concreteJoint.getSpatialTransform();
               for ip = 0 : 5
                  if strcmp(char(sptr.getCoordinateNames().get(ip)), char(coordinate.getName))
                        sptr.getTransformAxis(ip).getAxis(axisValues);
                        break
                  end
               end


               % make a torque actuator if a rotational coordinate
               if strcmp(motion, 'Rotational')
                   newActuator = TorqueActuator(joint.getParentFrame(),...
                                         joint.getParentFrame(),...
                                         axisValues);

               % make a point actuator if a translational coordinate.
             elseif strcmp(motion, 'translational')
                    % make a new Point actuator
                    newActuator = PointActuator();
                    % set the body
                    newActuator.set_body(char(joint.getChildFrame().getName()))
                    % set point that forces acts at
                    newActuator.set_point(massCenter)
                    % the point is expressed in the local
                    newActuator.set_point_is_global(false)
                    % set the direction that actuator will act in
                    newActuator.set_direction(axisValues)
                    % the force is expressed in the global
                    newActuator.set_force_is_global(true)
              else % something else that we don't support right now
                    newActuator = CoordinateActuator();
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
    % set min and max controls
    newActuator.setMaxControl(Inf)
    newActuator.setMinControl(-Inf)

    % append the new actuator onto the empty force set
    forceSet.cloneAndAppend(newActuator);
end

%% Print Actuators to file.
% Get the file parts
[pathname,filename,ext] = fileparts(modelpath);
% Make the print path
printPath = fullfile(pathname, [filename '_actuators.xml']);
%% Print the actuators xml file
forceSet.print(printPath);
%% Display printed file
display(['Printed actuators to ' printPath])

end
