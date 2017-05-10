function createActuatorsFile(varargin)
%  Make a template actuators file
%  Function identifies the coordinates that are connected to ground and 
%  places point or torque actuators on translational or rotational 
%  coordinates, respectively. All other coordinates will get coordinate 
%  actuators. Any constrained coordinates will be ignored.

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% Check the inputs
if isempty(varargin)
    % open dialog boxes to select the model
    [filename, pathname] = uigetfile('*.osim', 'Select an OpenSim Model File');
elseif nargin == 1
    if exist(varargin{1}, 'file') == 2
        [pathname,filename,ext] = fileparts(varargin{1});
        filename = [filename ext];
    else 
        error(['Input file is invalid or does not exist']);
    end
else
    error(['Number of inputs is > 1. Function only takes a single filepath']);
end

%% Get the model path
modelFilePath = fullfile(pathname,filename);

%% Generate an instance of the model
model = Model(modelFilePath);

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

%% Get the parts of the file path
[pathname,filename,ext] = fileparts(modelFilePath);
%% Define the new print path
printPath = fullfile(pathname, [filename '_actuators.xml']);
%% Print the actuators xml file
forceSet.print(printPath);
%% Display printed file
display(['Printed actuators to ' printPath])

end
