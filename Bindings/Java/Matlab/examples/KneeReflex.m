%% Part 1A: Create a Model object.

%% Part 2: Create the thigh body.

%% Part 3: Create the hip joint.

%% Part 4: Set the default value of the hip coordinate to +90 degrees.

%% Part 8A: Lock the hip coordinate.

%% Part 5: Create the shank body and knee joint.
% - Create a body named 'shank', with the same mass properties as the thigh.
% - Attach the same ellipsoid geometry as for the thigh.
% - Add the shank body to the model.

% - Create a PinJoint named 'knee' Parent: thigh; child: shank.
% - The location of the joint in the parent body is (0, -linkLength/2, 0).
% - The location of the joint in the child body is (0, +linkLength/2, 0).

%% Part 8B: Set the default value of the knee coordinate.

%% Part 6A: Add a vastus muscle (actuator).

%% Part 7: The vastus muscle wraps over the knee cap.

%% Part 9: Add an open-loop controller for the muscle.

%% Part 10A: Add reflex control (and disable the open-loop control).

%% Part 11A: Add a reporter to obtain muscle behavior.

%% Part 1B: Build the model and obtain its default state.

%% Part 10B: Set the initial speed of the knee coordinate.

%% Part 6B: Initialize the muscle fiber length state.

%% Part 1C: Simulate the model.

%% Part 11B: Plot muscle behavior.
