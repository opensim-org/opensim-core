% This script builds a 2-DOF leg model to simulate a knee jerk reflex.

%% Part 1A: Create a Model object.
import org.opensim.modeling.*;
model = Model();
model.setName('leg');
model.setUseVisualizer(true);

%% Part 2: Create the thigh body.
hipHeight = 1.0;
linkLength = 0.5;
linkMass = 5;

thigh = Body('thigh', linkMass, Vec3(0), Inertia(1));
thigh.attachGeometry(Ellipsoid(linkLength/10, linkLength/2, linkLength/10));
model.addBody(thigh);

%% Part 3: Create the hip joint.
hip = PinJoint('hip', ...
    model.getGround(), ...        % parent body
    Vec3(0, hipHeight, 0), ...    % location in parent
    Vec3(0), ...                  % orientation in child
    thigh, ...                    % child body
    Vec3(0, linkLength/2, 0), ... % location in child
    Vec3(0));                     % orientation in child 
model.addJoint(hip);

%% Part 4: Set the default value of the hip coordinate to +90 degrees.
hip.getCoordinate().setDefaultValue(0.5*pi);

%% Part 8A: Lock the hip coordinate.
hip.getCoordinate().setDefaultLocked(true);

%% Part 5: Create the shank body and knee joint.
% - Create a body named 'shank', with the same mass properties as the thigh.
% - Attach the same ellipsoid geometry as for the thigh.
% - Add the shank body to the model.
shank = Body('shank', linkMass, Vec3(0), Inertia(1));
shank.attachGeometry(Ellipsoid(linkLength/10, linkLength/2, linkLength/10));
model.addBody(shank);

% - Create a PinJoint named 'knee' Parent: thigh; child: shank.
% - The location of the joint in the parent body is (0, -linkLength/2, 0).
% - The location of the joint in the child body is (0, +linkLength/2, 0).
knee = PinJoint('knee', ...
    thigh, Vec3(0, -linkLength/2, 0), Vec3(0), ...
    shank, Vec3(0, linkLength/2, 0), Vec3(0));
model.addJoint(knee);

%% Part 8B: Set the default value of the knee coordinate.
knee.getCoordinate().setDefaultValue(-0.5*pi);

%% Part 6A: Add a vastus muscle (actuator).
vastus = Millard2012EquilibriumMuscle();
vastus.setName('vastus');
vastus.setMaxIsometricForce(500);
vastus.setOptimalFiberLength(0.19);
vastus.setTendonSlackLength(0.19);

vastus.addNewPathPoint('origin', thigh, Vec3(linkLength/10, 0, 0));
insertion = Vec3(0.75 * linkLength/10, 0.7 * linkLength/2, 0);
vastus.addNewPathPoint('insertion', shank, insertion);

model.addForce(vastus);

%% Part 7: The vastus muscle wraps over the knee cap.
patella = WrapCylinder();
patella.setName('patella');
patella.set_translation(Vec3(0, -linkLength/2, 0));
patella.set_radius(0.04);
patella.set_length(0.1);
patella.set_quadrant('x');
thigh.addWrapObject(patella);
vastus.updGeometryPath().addPathWrap(patella);

%% Part 9: Add an open-loop controller for the muscle.
brain = PrescribedController();
brain.addActuator(vastus);
% Between 0.3 and 0.35 seconds, excitation transitions from 0.05 to 0.5.
brain.prescribeControlForActuator('vastus', ...
    StepFunction(0.3, 0.35, 0.05, 0.5));
model.addController(brain);

%% Part 10A: Add reflex control (and disable the open-loop control).
brain.setEnabled(false);

reflex = ToyReflexController();
reflex.addActuator(vastus);
reflex.set_gain(40.0);
model.addController(reflex);

%% Part 11A: Add a reporter to obtain muscle behavior.
reporter = TableReporter();
reporter.set_report_time_interval(0.01);
reporter.addToReport(vastus.getOutput('normalized_fiber_length'), 'lm');
reporter.addToReport(...
    vastus.getOutput('active_force_length_multiplier'), 'fl');
model.addComponent(reporter);

%% Part 1B: Build the model and obtain its default state.
state = model.initSystem();

%% Part 10B: Set the initial speed of the knee coordinate.
% Units: radians per second.
knee.getCoordinate().setSpeedValue(state, -3.0);

%% Part 6B: Initialize the muscle fiber length state.
model.equilibrateMuscles(state);

%% Part 1C: Simulate the model.
osimSimulate(model, state, 3.0);

%% Part 11B: Plot muscle behavior.
table = reporter.getTable();
results = osimTableToStruct(table);
fieldnames(results);
hold on;
plot(results.lm, results.fl);
xlabel('normalized fiber length');
ylabel('active force-length multiplier');
axis([0, 1.5, 0, 1])
