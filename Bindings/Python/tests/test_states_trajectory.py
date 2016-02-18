import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# TODO add more tests of the integrity checks.

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
        forward.setFinalTime(0.1)
        forward.run()

        states = osim.StatesTrajectory.createFromStatesStorage(
                model, self.states_sto_fname)

        # Test indexing into the states container.
        model.getTotalMass(states[0])

        count = 0
        for i in range(states.getSize()):
            model.calcMassCenterVelocity(states.get(i))
            count += 1

        # Test iterator.
        count_iter = 0
        for state in states:
            model.calcMassCenterPosition(state)
            count_iter += 1
        assert count == count_iter

    def test_modify_states(self):
        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
        model.initSystem()

        states = osim.StatesTrajectory.createFromStatesStorage(
                model, self.states_sto_fname)

        states[0].setTime(4)
        assert states[0].getTime() == 4

        self.assertNotAlmostEqual(model.getStateVariableValue(states[2],
                "ground_pelvis/pelvis_tilt/value"), 8)
        model.setStateVariableValue(states[2],
                "ground_pelvis/pelvis_tilt/value", 8)
        self.assertAlmostEqual(model.getStateVariableValue(states[2],
                "ground_pelvis/pelvis_tilt/value"), 8)

        # Assigning is not allowed, since it easily allows people to violate
        # the ordering of the trajectory.
        # Also, the assignment `states.upd(5) = states[2]` is not possible in
        # Python ('can't assign to function call').
        def test_setitem():
            states[5] = states[2]
        self.assertRaises(TypeError, test_setitem)


    def test_states_storage_optional_arguments(self):
        # Try all combinations of optional arguments, just to ensure the
        # wrapping works.
        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
        sto = osim.Storage(self.states_sto_fname)
        states = osim.StatesTrajectory.createFromStatesStorage(
                model, sto)
        states = osim.StatesTrajectory.createFromStatesStorage(
                model, sto, False, False)
        states = osim.StatesTrajectory.createFromStatesStorage(
                model, sto, False, True)
        states = osim.StatesTrajectory.createFromStatesStorage(
                model, sto, True, False)
        states = osim.StatesTrajectory.createFromStatesStorage(
                model, sto, True, True)

    def test_populate_trajectory(self):
        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
        state = model.initSystem()
        states = osim.StatesTrajectory()
        states.append(state)
        state.setTime(1.0)
        states.append(state)

        self.assertEqual(states.getSize(), 2)
        self.assertEqual(states[1].getTime(), 1.0)

    def test_out_of_range(self):
        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
        state = model.initSystem()
        states = osim.StatesTrajectory()
        states.append(state)
        state.setTime(1.0)
        states.append(state)

        # TODO this exception message could be better...
        self.assertRaises(RuntimeError, lambda: states[2].getTime())

    def test_integrity_checks(self):
        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
        state = model.initSystem()
        states = osim.StatesTrajectory()
        states.append(state)
        state.setTime(1.0)
        states.append(state)
        self.assertTrue(states.isNondecreasingInTime())
        self.assertTrue(states.isConsistent())
        self.assertTrue(states.hasIntegrity())

        # Cannot append a state with an earlier time than the last one.
        state.setTime(0.5)
        self.assertRaises(RuntimeError, states.append, state)

        # However, since python doesn't have constness, we can edit the time of
        # a state in the trajectory.
        state.setTime(1.5)
        states.append(state)
        self.assertTrue(states.isNondecreasingInTime())
        states.back().setTime(0.25)
        self.assertFalse(states.isNondecreasingInTime())
        self.assertTrue(states.isConsistent())
        self.assertFalse(states.hasIntegrity())

        # TODO check violating isConsistent() (might need a different model).
    
    def test_access_by_time(self):

        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
        state = model.initSystem()
        states = osim.StatesTrajectory()
        state.setTime(0.5)
        states.append(state)
        state.setTime(0.6)
        states.append(state)
        state.setTime(0.7)
        states.append(state)
        state.setTime(0.8)
        states.append(state)
        state.setTime(0.9)
        states.append(state)
        state.setTime(0.9)
        states.append(state)
        state.setTime(0.9)
        states.append(state)
        state.setTime(1.2)
        states.append(state)

        assert states.getIndexBefore(0.55) == 0
        assert states.getIndexAfter(0.55) == 1
        self.assertRaises(RuntimeError, states.getIndexBefore, -1.5)
        self.assertRaises(RuntimeError, states.getIndexAfter, 10)

        # getBetween.
        # -----------
        # Iterate across the states between two times.
        indices = [1, 2, 3]
        index = 0
        for state in states.getBetween(0.55, 0.85):
            assert state.getTime() == states.get(indices[index]).getTime()
            index += 1
        del indices
        del index

        # Iterate over a small interval that doesn't include any states.
        count = 0
        for state in states.getBetween(0.55, 0.56):
            count += 1
        assert count == 0
        del count

        # Iterate over all states with the same time.
        count_same_time = 0;
        for state in states.getBetween(0.9, 0.9):
            count_same_time += 1
        assert count_same_time == 3
        del count_same_time

        # Test iterating with a tolerance.
        indices = [2, 3, 4, 5, 6]
        index = 0
        for state in states.getBetween(0.701, 0.899, 0.01):
            assert state.getTime() == states.get(indices[index]).getTime()
            index += 1
        del indices
        del index

        # Test iterating over an empty range.
        count_empty = 0
        for state in states.getBetween(1000.0, 1010.0):
            count_empty += 1
        assert count_empty == 0
        count_empty = 0
        for state in states.getBetween(-10.5, -5.3):
            count_empty += 1
        assert count_empty == 0

        # Test on an empty trajectory.
        count = 0;
        empty_traj = osim.StatesTrajectory()
        for state in empty_traj.getBetween(1.6, 6.2):
            count += 1
        assert count == 0

        # Test trying to use endTime > startTime.
        def func_that_raises():
            for state in states.getBetween(4.5, 1.1):
                print state.getTime()
        self.assertRaises(RuntimeError, func_that_raises)

        # Test trying to use endTime > startTime beyond the valid times.
        def func_that_raises_2():
            for state in states.getBetween(2000.0, 1580.0):
                print state.getTime()
        self.assertRaises(RuntimeError, func_that_raises_2)












