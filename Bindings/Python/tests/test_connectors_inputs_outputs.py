"""
Test that connectors, inputs, and outputs are functional in python.
"""
import os, unittest
import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Silence warning messages if mesh (.vtp) files cannot be found.
osim.Model.setDebugLevel(0)

# TODO in Component:
#
# SimTK::IteratorPair<...> Component::getConnectors()
# SimTK::IteratorPair<...> Component::getOutputs()
# Component::Connector can give you the concrete Connector or Input (using a
# fancy SWIG typemap). Not possible for outputs (well, you can manually
# downcast), since we do not have a registry for those types.
# Wrap common Input types: InputDouble or InputDbl or Input, InputVec3, InputVector, etc.
# 
# Component::getComponentList() and Component::getComponentList(type)
#
# TODO ClonePtr<Output> in swig doc3.0: 25.3.15.2
#
# SimTK::IteratorPair<...> Component::getInputs()

class TestConnectors(unittest.TestCase):
    def test_accessing_connectors(self):
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        ground = model.getGround()
        shoulder = model.getJointSet().get("r_shoulder")

        # Connect up the model.
        model.initSystem()

        # With an AbstractConnector, we can call AbstractConnector's methods.
        assert shoulder.getConnector("parent_frame").getNumConnectees() == 1
        assert (shoulder.getConnector("parent_frame").getConnecteeTypeName() ==
                "PhysicalFrame")

        # Check that the connectees point to the correct objects.
        assert (shoulder.getConnectee("child_frame").this ==
                model.getBodySet().get("r_humerus").this)

        assert (
            type(shoulder.getConnector("child_frame").getConnecteeAsObject())
            == osim.OpenSimObject)
        # In Python, we are able to get the concrete type from this method.
        # by using a SWIG typemap(out).
        assert type(shoulder.getConnectee("child_frame")) == osim.Body

    def test_iterate_connectors(self):
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        shoulder = model.getJointSet().get("r_shoulder")

        # Connect up the model.
        model.initSystem()

        names = ["child_frame", "parent_frame"]

        # By name.
        count_by_name = 0
        for name in shoulder.getConnectorNames():
            assert shoulder.getConnector(name).getName() == names[count_by_name]
            count_by_name += 1
        assert count_by_name == 2

        # By iterator.
        # TODO doesn't exist yet.
        # TODO count_by_iter = 0
        # TODO for conn in shoulder.getConnectors():
        # TODO     count_by_iter += 1
        # TODO     assert conn.getName() == names[count_by_iter]
        # TODO assert count_by_iter == 2

    def test_connecting(self):
        # We'll create a model from scratch and set up its joints with
        # the connector interface.
        model = osim.Model()
        b1 = osim.Body("b1", 1, osim.Vec3(1), osim.Inertia(1))
        b2 = osim.Body("b2", 2, osim.Vec3(1), osim.Inertia(1))

        j1 = osim.PinJoint()
        j1.setName("j1")
        j1.updConnector("parent_frame").connect(model.getGround())
        j1.connectConnector_child_frame(b1)

        j2 = osim.PinJoint()
        j2.setName("j2")
        j2.connectConnector_parent_frame(b1)
        j2.updConnector("child_frame").connect(b2)

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
        self.assertEqual(out.getTypeName(), "SimTK::Vec<3,double,1>")
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
        self.assertEquals(coord.getOutput('speed').getChannel('').getPathName(),
                '/arm26/r_shoulder/r_shoulder_elev|speed')

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
        expectedLabels = ['/model_/pin/pin_coord_0|value', 'spd',
                          '/model_/source|column:col1', 'second_col']
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





















