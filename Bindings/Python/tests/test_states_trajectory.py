import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# TODO append
# TODO __setitem__

class TestStatesTrajectory(unittest.TestCase):
    states_sto_fname = "test_states_trajectory_gait1018_states.sto"
    def test_index_and_iterator(self):

        if os.path.exists(self.states_sto_fname):
            os.remove(self.states_sto_fname)

        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
        model.initSystem()

        forward = osim.ForwardTool()
        forward.setModel(model)
        forward.setName('test_states_trajectory_gait1018')
        forward.run()

        states = osim.StatesTrajectory.createFromStatesStorage(
                model, self.states_sto_fname)

        # Test indexing into the states container.
        model.getTotalMass(states[0])

        for i in range(states.getSize()):
            model.calcMassCenterVelocity(states.get(i))

        # Test iterator.
        for state in states:
            model.calcMassCenterPosition(state)

#    def test_modify_states(self):
#        assert False
#
#    def test_states_storage_optional_arguments(self):
#        assert False
