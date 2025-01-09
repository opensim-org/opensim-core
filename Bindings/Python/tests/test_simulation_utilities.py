
import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

class TestSimulationUtilities(unittest.TestCase):
    def test_update_kinematics(self):
        model = osim.Model(
                os.path.join(test_dir, 'gait10dof18musc_subject01.osim'))
        kinematics_file = os.path.join(test_dir, 'std_subject01_walk1_ik.mot')

        # updatePre40KinematicsStorageFor40MotionType() is not wrapped.
        osim.updatePre40KinematicsFilesFor40MotionType(model,
                [kinematics_file])
