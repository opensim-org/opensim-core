"""These are basic tests to make sure that C++ classes were wrapped properly.
There shouldn't be any python-specifc tests here.

"""

import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Silence warning messages if mesh (.vtp) files cannot be found.
osim.Model.setDebugLevel(0)

class TestBasics(unittest.TestCase):
    def test_version(self):
        print(osim.__version__)

    def test_muscle_helper_classes(self):
        # This test exists because some classes that Thelen2003Muscle used were
        # not accessibly in the bindings.
        muscle = osim.Thelen2003Muscle()

        fwpm = muscle.getPennationModel()
        fwpm.get_optimal_fiber_length()

        adm = muscle.getActivationModel()
        adm.get_activation_time_constant()

        muscle = osim.Millard2012EquilibriumMuscle()

        tendonFL = osim.TendonForceLengthCurve()
        muscle.setTendonForceLengthCurve(tendonFL)

    def test_SimTKArray(self):
        # Initally created to test the creation of a separate simbody module.
        ad = osim.SimTKArrayDouble()
        ad.push_back(1)

        av3 = osim.SimTKArrayVec3()
        av3.push_back(osim.Vec3(8))
        assert av3.at(0).get(0) == 8

    def test_ToolAndModel(self):
        # Test tools module.
        cmc = osim.CMCTool()
        model = osim.Model()
        model.setName('alphabet')
        cmc.setModel(model)
        assert cmc.getModel().getName() == 'alphabet'

    def test_AnalysisToolModel(self):
        # Test analyses module.
        cmc = osim.CMCTool()
        model = osim.Model()
        model.setName('eggplant')
        fr = osim.ForceReporter()
        fr.setName('strong')
        cmc.setModel(model)
        cmc.getAnalysisSet().adoptAndAppend(fr)

        assert cmc.getModel().getName() == 'eggplant'
        assert cmc.getAnalysisSet().get(0).getName() == 'strong'

    def test_ManagerConstructorCreatesIntegrator(self):
        # Make sure that the Manager is able to create a default integrator.
        # This tests a bug fix: previously, it was impossible to use the
        # Manager to integrate from MATLAB/Python, since it was not possible
        # to provide an Integrator to the Manager.
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        state = model.initSystem()

        manager = osim.Manager(model)
        state.setTime(0);
        manager.initialize(state);
        state = manager.integrate(0.00001)

    def test_WrapObject(self):
        # Make sure the WrapObjects are accessible.
        model = osim.Model()

        sphere = osim.WrapSphere()
        model.getGround().addWrapObject(sphere)

        cylinder = osim.WrapCylinder()
        cylinder.set_radius(0.5)
        model.getGround().addWrapObject(cylinder)

        torus = osim.WrapTorus()
        model.getGround().addWrapObject(torus)

        ellipsoid = osim.WrapEllipsoid()
        model.getGround().addWrapObject(ellipsoid)

    def test_ToyReflexController(self):
        controller = osim.ToyReflexController()
        
    def test_GCVSplineSet(self):
        splineset = osim.GCVSplineSet(os.path.join(test_dir,
            'std_subject01_walk1_ik.mot'))
        splineset = osim.GCVSplineSet(
                osim.TimeSeriesTable(os.path.join(test_dir,
                    'std_subject01_walk1_ik.mot')), [], 5, 0)

    def test_deserialize_tool_with_empty_model_file(self):
        # Ensure an exception is thrown when loading an AbstractTool (e.g.,
        # ForwardTool) setup file with an empty model_file. In particular, we
        # want to check the case where force_set_files is not empty.
        with self.assertRaises(RuntimeError):
            rra = osim.ForwardTool(os.path.join(test_dir,
                'gait2392_setup_forward_empty_model.xml'))

        # No exception if we pass loadModel=False
        rra = osim.ForwardTool(
                os.path.join(test_dir,
                    'gait2392_setup_forward_empty_model.xml'),
                True, # updateFromXMLNode
                False, # loadModel
                )

    def test_property_helper(self):
        muscle = osim.Thelen2003Muscle()
        # set max_isometric_force property using PropertyHelper
        # then retrieve using get_max_isometric_force native method
        property = muscle.getPropertyByName('max_isometric_force')
        osim.PropertyHelper.setValueDouble(200, property);
        assert muscle.get_max_isometric_force()==200