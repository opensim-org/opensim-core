"""
Test that sockets, inputs, and outputs are functional in python.
"""
import os, unittest
import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Silence warning messages if mesh (.vtp) files cannot be found.
osim.Model.setDebugLevel(0)

class TestAccessSubcomponents(unittest.TestCase):
    def test_individual_components(self):
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        muscle = model.getComponent('forceset/BICshort')
        assert muscle.getName() == 'BICshort'
        # No downcasting necessary!
        muscle.get_max_isometric_force() # Method on Muscle.
        muscle = model.updComponent('forceset/BICshort')
        muscle.set_max_isometric_force(100)

    def test_component_list(self):
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))

        num_components = 0
        for comp in model.getComponentsList():
            num_components += 1
        assert num_components > 0

        num_bodies = 0
        for body in model.getBodyList():
            num_bodies += 1
            body.getMass()
        assert num_bodies == 2

        num_joints = 0
        for joint in model.getJointList():
            num_joints += 1
            joint.numCoordinates()
        assert num_joints == 2

        # Custom filtering.
        num_bodies = 0
        for frame in model.getFrameList():
            body = osim.Body.safeDownCast(frame)
            if body != None:
                num_bodies += 1
                print(body.getName())
                body.getInertia()
        assert num_bodies == 2

        model = osim.Model()
        ground = model.getGround()

        thelenMuscle = osim.Thelen2003Muscle("Darryl", 1, 0.5, 0.5, 0)
        thelenMuscle.addNewPathPoint("muscle1-point1", ground, osim.Vec3(0.0,0.0,0.0))
        thelenMuscle.addNewPathPoint("muscle1-point2", ground, osim.Vec3(1.0,0.0,0.0))
        millardMuscle = osim.Millard2012EquilibriumMuscle("Matt", 1, 0.5,
                                                          0.5, 0)
        millardMuscle.addNewPathPoint("muscle1-point1", ground, osim.Vec3(0.0,0.0,0.0))
        millardMuscle.addNewPathPoint("muscle1-point2", ground, osim.Vec3(1.0,0.0,0.0))

        model.addComponent(thelenMuscle)
        model.addComponent(millardMuscle)

        # Total number of muscles is 2.
        assert len(set(model.getMuscleList())) == 2
        for muscle in model.getMuscleList():
            assert (isinstance(muscle, osim.Thelen2003Muscle) or
                    isinstance(muscle, osim.Millard2012EquilibriumMuscle))

        # There is exactly 1 Thelen2003Muscle.
        assert len(set(model.getThelen2003MuscleList())) == 1
        for muscle in model.getThelen2003MuscleList():
            assert isinstance(muscle, osim.Thelen2003Muscle)

        # There is exactly 1 Millard2012EquilibriumMuscle.
        assert len(set(model.getMillard2012EquilibriumMuscleList())) == 1
        for muscle in model.getMillard2012EquilibriumMuscleList():
            assert isinstance(muscle, osim.Millard2012EquilibriumMuscle)

    def test_component_filter(self):
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        comps = model.getMuscleList()
        comps.setFilter(osim.ComponentFilterAbsolutePathNameContainsString('BIC'))
        count = 0
        BICnames = ['BIClong', 'BICshort']
        for comp in comps:
            assert comp.getName() == BICnames[count]
            # The ComponentList iterator does the downcasting for us!
            assert type(comp) == osim.Thelen2003Muscle
            count += 1
        assert count == 2
