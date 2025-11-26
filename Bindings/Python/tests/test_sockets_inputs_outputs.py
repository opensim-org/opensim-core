"""
Test that sockets, inputs, and outputs are functional in python.
"""
import os, unittest
import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Silence warning messages if mesh (.vtp) files cannot be found.
osim.Model.setDebugLevel(0)

# TODO in Component:
#
# SimTK::IteratorPair<...> Component::getSockets()
# SimTK::IteratorPair<...> Component::getOutputs()
# Component::Socket can give you the concrete Socket or Input (using a
# fancy SWIG typemap). Not possible for outputs (well, you can manually
# downcast), since we do not have a registry for those types.
# Wrap common Input types: InputDouble or InputDbl or Input, InputVec3, InputVector, etc.
# 
# Component::getComponentList() and Component::getComponentList(type)
#
# TODO ClonePtr<Output> in swig doc3.0: 25.3.15.2
#
# SimTK::IteratorPair<...> Component::getInputs()

class TestSockets(unittest.TestCase):
    def test_accessing_sockets(self):
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        ground = model.getGround()
        shoulder = model.getJointSet().get("r_shoulder")

        # Connect up the model.
        model.initSystem()

        # With an AbstractSocket, we can call AbstractSocket's methods.
        assert shoulder.getSocket("parent_frame").getNumConnectees() == 1
        assert (shoulder.getSocket("parent_frame").getConnecteeTypeName() ==
                "PhysicalFrame")

        # Check that the connectees point to the correct objects.
        assert (shoulder.getConnectee("child_frame").this ==
                shoulder.getComponent("r_humerus_offset").this)

        assert (
            type(shoulder.getSocket("child_frame").getConnecteeAsObject())
            == osim.OpenSimObject)
        # In Python, we are able to get the concrete type from this method.
        # by using a SWIG typemap(out).
        assert type(shoulder.getConnectee("child_frame")) == osim.PhysicalOffsetFrame

    def test_iterate_sockets(self):
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        shoulder = model.getJointSet().get("r_shoulder")

        # Connect up the model.
        model.initSystem()

        names = ["child_frame", "parent_frame"]

        # By name.
        count_by_name = 0
        for name in shoulder.getSocketNames():
            assert shoulder.getSocket(name).getName() == names[count_by_name]
            count_by_name += 1
        assert count_by_name == 2

        # By iterator.
        # TODO doesn't exist yet.
        # TODO count_by_iter = 0
        # TODO for socket in shoulder.getSockets():
        # TODO     count_by_iter += 1
        # TODO     assert socket.getName() == names[count_by_iter]
        # TODO assert count_by_iter == 2

    def test_connecting(self):
        # We'll create a model from scratch and set up its joints with
        # the socket interface.
        model = osim.Model()
        b1 = osim.Body("b1", 1, osim.Vec3(1), osim.Inertia(1))
        b2 = osim.Body("b2", 2, osim.Vec3(1), osim.Inertia(1))

        j1 = osim.PinJoint()
        j1.setName("j1")
        j1.updSocket("parent_frame").connect(model.getGround())
        j1.connectSocket_child_frame(b1)

        j2 = osim.PinJoint()
        j2.setName("j2")
        j2.connectSocket_parent_frame(b1)
        j2.updSocket("child_frame").connect(b2)

        model.addBody(b1)
        model.addBody(b2)
        model.addJoint(j1)
        model.addJoint(j2)

        state = model.initSystem()

        # Check that the connectees point to the correct object.
        assert j1.getConnectee("parent_frame").this == model.getGround().this
        assert j1.getConnectee("child_frame").this == b1.this
        assert j2.getConnectee("parent_frame").this == b1.this
        assert j2.getConnectee("child_frame").this == b2.this

        # Make sure we can call methods of the concrete connectee type
        # (that the downcast succeeded).
        assert j1.getConnectee("child_frame").getMass() == 1
        assert j2.getConnectee("child_frame").getMass() == 2


class TestInputsOutputs(unittest.TestCase):
    def test_output_values(self):
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        s = model.initSystem()

        out = model.getOutput("com_position")
        self.assertEqual(out.getTypeName(), "Vec3")
        print(out.getValueAsString(s))

        # Users should just call the method connected to this output, but
        # it may be nice for users to still be able to call this method.
        print(osim.OutputVec3.safeDownCast(out).getValue(s))
        # TODO print(out.getOutputValue(s, "com_position"))

        model.realizeDynamics(s)
        for musc in model.getMuscles():
            exc = osim.OutputDouble.safeDownCast(musc.getOutput("excitation"))
            assert exc.getValue(s) == 0
            act = osim.OutputDouble.safeDownCast(musc.getOutput("activation"))
            assert act.getValue(s) == 0.05

        # AbstractChannel.
        coord = model.getCoordinateSet().get(0)
        self.assertEqual(coord.getOutput('speed').getChannel('').getPathName(),
                '/jointset/r_shoulder/r_shoulder_elev|speed')

        # Access the value of a concrete Channel.
        # TODO Concrete channels are not wrapped yet.
        # TODO OutputChannelDouble.safeDownCast(comp.getOutput(name).getChannel()).getValue(s)

        # TODO deal with overloaded template and non-template methods like
        # getInput() and getOutput().

        # TODO no components have inputs yet.
        # When they exist, test connecting inputs and outputs.

    def test_iterate_outputs(self):
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        s = model.initSystem()

        musc = model.getMuscles().get(0)

        num_muscle_outputs = 32

        # By name.
        count_by_name = 0
        for name in musc.getOutputNames():
            count_by_name += 1
            assert len(musc.getOutput(name).getName()) > 0
        # We may add more outputs to Muscle in the future, but it is unlikely
        # that we will reduce the number.
        assert count_by_name >= 32

        # By iterator.
        # TODO doesn't exist yet.
        # TODO count_by_iter = 0
        # TODO for out in musc.getOutputs():
        # TODO     count_by_iter += 1
        # TODO     assert len(out.getName()) > 0
        # TODO assert count_by_iter == 32

    def test_connecting_and_iterate_inputs(self):
        m = osim.Model()
        b = osim.Body('b1', 2.0, osim.Vec3(1, 0, 0), osim.Inertia(1))
        j = osim.PinJoint('pin', m.getGround(), b)

        # Source.
        source = osim.TableSource()
        source.setName("source")

        table = osim.TimeSeriesTable()
        table.setColumnLabels(('col1', 'col2', 'col3', 'col4'))
        row = osim.RowVector([1, 2, 3, 4])
        table.appendRow(0.0, row)
        row = osim.RowVector([2, 3, 4, 5])
        table.appendRow(1.0, row)
        source.setTable(table)

        # Reporter.
        rep = osim.ConsoleReporter()
        rep.setName("rep")

        m.addBody(b)
        m.addJoint(j)
        m.addComponent(source)
        m.addComponent(rep)

        # Connect.
        # There are multiple ways to perform the connection, especially
        # for reporters.
        coord = j.get_coordinates(0)
        rep.updInput('inputs').connect(coord.getOutput('value'))
        rep.connectInput_inputs(coord.getOutput('speed'), 'spd')
        rep.connectInput_inputs(
                source.getOutput('column').getChannel('col1'))
        rep.addToReport(
                source.getOutput('column').getChannel('col2'), 'second_col')

        s = m.initSystem()

        # Access and iterate through AbstractInputs, using names.
        expectedLabels = ['/jointset/pin/pin_coord_0|value', 'spd',
                          '/source|column:col1', 'second_col']
        i = 0
        for name in rep.getInputNames():
            # Actually, there is only one input, which we connected to 4
            # channels.
            assert rep.getInput(name).getNumConnectees() == 4
            for j in range(4):
                assert (rep.getInput(name).getLabel(j) == expectedLabels[j])
            i += 1

        # Access concrete Input.
        # Input value is column 2 at time 0.
        assert (osim.InputDouble.safeDownCast(
                rep.getInput('inputs')).getValue(s, 3) == 2.0)


    def test_input_alias(self):
        model_filename = 'test_input_alias.osim'

        # This function creates and prints the model to a .osim file. We invoke
        # this function below.
        def print_model():
            model = osim.Model()
            model.setName('model')

            # Create a body with name 'body', mass of 1 kg, center of mass at
            # the origin of the body, and unit inertia
            # (Ixx = Iyy = Izz = 1 kg-m^2).
            body = osim.Body('body', 1.0, osim.Vec3(0), osim.Inertia(1))

            # Create a free joint (all 6 degrees of freedom) with Ground as
            # the parent body and 'body' as the child body.
            joint = osim.FreeJoint('joint', model.getGround(), body)

            # Add the body and joint to the model.
            model.addComponent(body)
            model.addComponent(joint)

            # Create a TableReporter to save quantities to a file after
            # simulating.
            reporter = osim.TableReporterVec3()
            reporter.setName('reporter')
            reporter.set_report_time_interval(0.1)

            reporter.addToReport(model.getOutput('com_position'))

            model.addComponent(reporter)
            model.finalizeConnections()

            reporter.getInput('inputs').setAlias(0, 'com_pos')

            # Display what input-output connections look like in XML
            # (in .osim files).
            print("Reporter input-output connections in XML:\n" + \
                  reporter.dump())

            model.printToXML(model_filename)

        # Create and print the model file.
        print_model()
        # Load the model file.
        deserialized_model = osim.Model(model_filename)
        state = deserialized_model.initSystem()

        # We can fetch the TableReporter from within the deserialized model.
        reporter = osim.TableReporterVec3.safeDownCast(
                deserialized_model.getComponent('reporter'))

        assert reporter.getInput('inputs').getAlias(0) == 'com_pos'
