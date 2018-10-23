
import org.opensim.modeling.*

% Create a model.
model = Model();
model.setName('leg');
ground = model.getGround();
block = Body('block', 2.0, Vec3(1, 0, 0), Inertia(1));
joint = PinJoint('pin', ground, block);
offset = PhysicalOffsetFrame();
offset.setName('offset');

source = TableSource();
source.setName('source');
table = TimeSeriesTable();
labels = StdVectorString(4);
labels.set(0, 'c1'); labels.set(1, 'c2'); labels.set(2, 'c3'); labels.set(3, 'c4');
table.setColumnLabels(labels);
row = RowVector(4, 1.0);
table.appendRow(0.0, row);
row.setTo(2.0);
table.appendRow(1.0, row);
source.setTable(table);

rep = ConsoleReporter();
rep.setName('c_rep');

model.addBody(block)
model.addJoint(joint)
model.addComponent(offset);
model.addComponent(source);
model.addComponent(rep);


% Sockets
% =======

% Access (and iterate through) a component's AbstractSockets, using names.
names = joint.getSocketNames();
for i = 0:(names.size() - 1)
    typeName = joint.getSocket(names.get(i)).getConnecteeTypeName();
    assert(strcmp(typeName, 'PhysicalFrame'));
end

% Access (and iterate through) a component's connectees as objects.
expectedNames = {'block', 'ground'};
for i = 0:(names.size() - 1)
    obj = joint.getConnectee(names.get(i)); % this is an object.
    assert(strcmp(obj.getName(), expectedNames{i+1}));
end

% Access a specific concrete connectee.
body = Body.safeDownCast(joint.getConnectee('child_frame'));
assert(body.getMass() == 2);

% Connect a socket. Try the different methods to ensure they all work.
offset.connectSocket_parent(ground);
offset.updSocket('parent').connect(ground);
model.finalizeConnections()
assert(strcmp(offset.getSocket('parent').getConnecteePath(), '/ground'));



% Outputs
% =======
state = model.initSystem();
model.realizeAcceleration(state);

% Access (and iterate through) the AbstractOutputs, using names.
coord = joint.get_coordinates(0);
names = coord.getOutputNames();
for i = 0:(names.size() - 1)
    assert(coord.getOutput(names.get(i)).isListOutput() == 0);
    coord.getOutput(names.get(i)).getValueAsString(state);
end

% Access the value of a concrete Output.
concreteOutput = OutputDouble.safeDownCast(coord.getOutput('speed'));
concreteOutput.getValue(state);

% Channels
% --------
% Access AbstractChannels.
assert(strcmp(coord.getOutput('speed').getChannel('').getPathName(), ...
              '/jointset/pin/pin_coord_0|speed'));

% Access the value of a concrete Channel.
% TODO Concrete channels are not wrapped yet.
% TODO OutputDouble.safeDownCast(comp.getOutput(name)).getChannel(name).getValue(s);
% TODO ChannelDouble.safeDownCast(comp.getOutput(name).getChannel(name)).getValue(s);


% Connect inputs and outputs
% ==========================
% Only need the abstract types in order to connect.
rep.updInput('inputs').connect(coord.getOutput('value'));
% With alias:
rep.connectInput_inputs(coord.getOutput('speed'), 'target');
% These commands use the AbstractChannel.
rep.addToReport(source.getOutput('column').getChannel('c1'));
rep.updInput('inputs').connect(source.getOutput('column').getChannel('c2'), ...
                               'second_col');
model.finalizeConnections()

% Inputs
% ======

% Access (and iterate through) the AbstractInputs, using names.
names = rep.getInputNames();
expectedAliases = {'', 'target', '', 'second_col'};
expectedLabels  = {'/jointset/pin/pin_coord_0|value', 'target', ...
                   '/source|column:c1', 'second_col'};
for i = 0:(names.size() - 1)
    % Actually, there is only one Input, named 'inputs'.
    % We connected it to 4 channels.
    numConnectees = rep.getInput(names.get(i)).getNumConnectees();
    assert(numConnectees == 4);
    for j = 0:(numConnectees - 1)
        assert(strcmp(rep.getInput(names.get(i)).getAlias(j), ...
                      expectedAliases{j+1}));
        assert(strcmp(rep.getInput(names.get(i)).getLabel(j), ...
                      expectedLabels{j+1}));
    end
end

% Access the value of a concerete Input.
% The value 1 comes from column 2 of the TableSource.
concreteInput = InputDouble.safeDownCast(rep.getInput('inputs'));
assert(concreteInput.getValue(state, 3)==1);
% TODO Concrete channels are not wrapped yet.
% TODO InputDouble.safeDownCast(comp.getInput(name)).getChannel(0).getValue(s);


