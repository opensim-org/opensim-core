"""The tests here ensure the proper functionality of modifications/additions we
make to the C++ API, via the SWIG interface (*.i) file.

"""

import os
import unittest

import opensim as osim

# TODO __str__() for numeric types.
# TODO iterators for Vec3, Vector, Set's, and ComponentList's.
# TODO typemaps for Vec3 and Vector.
# TODO indexing for Vec3 and Vector.
# TODO operator overloading operator+().
# TODO size() -> __len__()

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

class TestSwigAddtlInterface(unittest.TestCase):
    def test_markAdopted1(self):
        """Ensures that we can tell an object that some other object is managing
        its memory.
        """
        a = osim.Model()
        assert a.this
        assert a.thisown
        a._markAdopted()
        assert a.this
        assert not a.thisown
    
    def test_markAdopted2(self):
        a = osim.Model()
    
        # We just need the following not to cause a segfault.
    
        # Model add*
        pa = osim.PathActuator()
        pa.setName('pa')
        a.addForce(pa)

        probe = osim.Umberger2010MuscleMetabolicsProbe()
        probe.setName('probe')
        a.addProbe(probe)

        ma = osim.MuscleAnalysis()
        ma.setName('ma')
        a.addAnalysis(ma)

        pc = osim.PrescribedController()
        pc.setName('pc')
        a.addController(pc)
        
        body = osim.Body('body1',
                1.0,
                osim.Vec3(0, 0, 0),
                osim.Inertia(0, 0, 0)
                )
    
        loc_in_parent = osim.Vec3(0, 0, 0)
        orient_in_parent = osim.Vec3(0, 0, 0)
        loc_in_body = osim.Vec3(0, 0, 0)
        orient_in_body = osim.Vec3(0, 0, 0)
        print "creating Weld Joint.."
        joint = osim.WeldJoint("weld_joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        print "adding a body .."
        a.addBody(body)
        print "adding a joint .."
        a.addJoint(joint)
        print "Creating a ConstantDistanceConstraint.."
        constr = osim.ConstantDistanceConstraint()
        constr.setBody1ByName("ground")
        constr.setBody1PointLocation(osim.Vec3(0, 0, 0))
        constr.setBody2ByName("body")
        constr.setBody2PointLocation(osim.Vec3(1, 0, 0))
        constr.setConstantDistance(1)
        a.addConstraint(constr)
    
        f = osim.BushingForce("bushing", "ground", "body",
                osim.Vec3(2, 2, 2), osim.Vec3(1, 1, 1),
                osim.Vec3(0, 0, 0), osim.Vec3(0, 0, 0))
        a.addForce(f)
    
        f2 = osim.BushingForce()
        a.addForce(f2)
    
        f3 = osim.SpringGeneralizedForce()
        a.addForce(f3)
    
        model = osim.Model(os.path.join(test_dir, "arm26.osim"))
        g = osim.CoordinateActuator('r_shoulder_elev')
        model.addForce(g)
    
    def test_Joint(self):
        a = osim.Model()
        
        body = osim.Body('body',
                1.0,
                osim.Vec3(0, 0, 0),
                osim.Inertia(0, 0, 0)
                )
    
        loc_in_parent = osim.Vec3(0, -0, 0)
        orient_in_parent = osim.Vec3(0, 0, 0)
        loc_in_body = osim.Vec3(0, 0, 0)
        orient_in_body = osim.Vec3(0, 0, 0)
    
        joint = osim.FreeJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        del joint
        spatialTransform = osim.SpatialTransform()
        joint = osim.CustomJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent, spatialTransform)
        del joint
        joint = osim.EllipsoidJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent, osim.Vec3(1, 1, 1))
        del joint
        joint = osim.BallJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        del joint
        joint = osim.PinJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        del joint
        joint = osim.SliderJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        del joint
        joint = osim.WeldJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        del joint
        joint = osim.GimbalJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        del joint
        joint = osim.UniversalJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        del joint
        joint = osim.PlanarJoint("joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        del joint
    
    
    def test_markAdoptedSets(self):
    
        # Set's.
        fus = osim.FunctionSet()
        fu1 = osim.Constant()
        fus.adoptAndAppend(fu1)
        del fus
        del fu1
    
        s = osim.ScaleSet()
        o = osim.Scale()
        s.adoptAndAppend(o)
        del s
        del o
    
        s = osim.ControlSet()
        o = osim.ControlLinear()
        s.adoptAndAppend(o)
        del s
        del o
    
        s = osim.BodyScaleSet()
        o = osim.BodyScale()
        s.adoptAndAppend(o)
        del s
        del o
    
        s = osim.PathPointSet()
        o = osim.PathPoint()
        s.adoptAndAppend(o)
        del s
        del o
    
        s = osim.IKTaskSet()
        o = osim.IKMarkerTask()
        s.adoptAndAppend(o)
        del s
        del o
    
        s = osim.MarkerPairSet()
        o = osim.MarkerPair()
        s.adoptAndAppend(o)
        del s
        del o
    
        s = osim.MeasurementSet()
        o = osim.Measurement()
        s.adoptAndAppend(o)
        del s
        del o

        s = osim.FrameSet()
        o = osim.Body()
        s.adoptAndAppend(o)
        del s
        del o
    
        s = osim.ForceSet()
        o = osim.CoordinateLimitForce()
        s.adoptAndAppend(o)
        del s
        del o
         
        s = osim.ForceSet()
        o = osim.SpringGeneralizedForce()
        s.append(o)
     
        s = osim.ProbeSet()
        o = osim.Umberger2010MuscleMetabolicsProbe()
        s.adoptAndAppend(o)
        del s
        del o
    
        a = osim.Model()
        body = osim.Body('body',
                1.0,
                osim.Vec3(0, 0, 0),
                osim.Inertia(0, 0, 0)
                )
    
        loc_in_parent = osim.Vec3(0, -0, 0)
        orient_in_parent = osim.Vec3(0, 0, 0)
        loc_in_body = osim.Vec3(0, 0, 0)
        orient_in_body = osim.Vec3(0, 0, 0)
        joint = osim.WeldJoint("weld_joint",
                a.getGround(),
                loc_in_parent, orient_in_parent,
                body,
                loc_in_body, orient_in_parent)
        a.addBody(body)
    
    
        constr = osim.ConstantDistanceConstraint()
        constr.setBody1ByName("ground")
        constr.setBody1PointLocation(osim.Vec3(0, 0, 0))
        constr.setBody2ByName("body")
        constr.setBody2PointLocation(osim.Vec3(1, 0, 0))
        constr.setConstantDistance(1)
        a.addConstraint(constr)

    def test_PrescribedController_prescribeControlForActuator(self):
        # Test memory management for
        # PrescribedController::prescribeControlForActuator().
        model = osim.Model()
        # Body.
        body = osim.Body('b1', 1.0, osim.Vec3(0, 0, 0), osim.Inertia(0, 0, 0))
        model.addBody(body)
        # Joint.
        joint = osim.PinJoint('j1', model.getGround(), body)
        model.addJoint(joint)
        # Actuator.
        actu = osim.CoordinateActuator()
        actu.setName('actu')
        actu.setCoordinate(joint.get_coordinates(0))
        model.addForce(actu)
        # Controller.
        contr = osim.PrescribedController()
        contr.addActuator(actu)
        self.assertRaises(RuntimeError,
                contr.prescribeControlForActuator, 1, osim.Constant(3))
        # The following calls should not cause a memory leak:
        contr.prescribeControlForActuator(0, osim.Constant(2))
        contr.prescribeControlForActuator('actu', osim.Constant(4))
    
    def test_vec3_operators(self):
        v1 = osim.Vec3(1, 2, 3)
        # Tests __getitem__().
        assert v1[0] == 1
        assert v1[1] == 2
    
        # Out of bounds.
        with self.assertRaises(RuntimeError):
            v1[-1]
        with self.assertRaises(RuntimeError):
            v1[3]
        with self.assertRaises(RuntimeError):
            v1[5]
    
        # Tests __setitem__().
        v1[0] = 5
        assert v1[0] == 5
    
        # Out of bounds.
        with self.assertRaises(RuntimeError):
            v1[-1] = 5
        with self.assertRaises(RuntimeError):
            v1[3] = 1.3

        # Add. TODO removed for now.
        v2 = osim.Vec3(5, 6, 7)
        #v3 = v1 + v2
        #assert v3[0] == 10
        #assert v3[1] == 8
        #assert v3[2] == 10

        # Length.
        assert len(v2) == 3


    def test_vector_operators(self):
        v = osim.Vector(5, 3)

        # Tests __getitem__()
        assert v[0] == 3
        assert v[4] == 3

        # Out of bounds.
        with self.assertRaises(RuntimeError):
            v[-1]
        with self.assertRaises(RuntimeError):
            v[5]

        # Tests __setitem__()
        v[0] = 15
        assert v[0] == 15

        with self.assertRaises(RuntimeError):
            v[-1] = 12
        with self.assertRaises(RuntimeError):
            v[5] = 14
        with self.assertRaises(RuntimeError):
            v[9] = 18

        # Size.
        assert len(v) == 5

    def test_exceptions(self):
        with self.assertRaises(RuntimeError):
            osim.Model("NONEXISTANT_FILE_NAME")
        with self.assertRaises(RuntimeError):
            m = osim.Model()
            # Asking for the visualizer just after constructing a model
            # throws an exception.
            m.getVisualizer()

#    def test_typemaps(self):
        # TODO disabled for now
        #m = osim.Model()
        #m.setGravity(osim.Vec3(1, 2, 3))
        #m.setGravity([1, 2, 3])

        #with self.assertRaises(ValueError):
        #    m.setGravity(['a', 2, 3])
        #with self.assertRaises(ValueError):
        #    m.setGravity([1, 2])
        #with self.assertRaises(ValueError):
        #    m.setGravity([1, 2, 6, 3])

    def test_printing(self):
        v1 = osim.Vec3(1, 3, 2)
        assert v1.__str__() == "~[1,3,2]"

        v2 = osim.Vector(7, 3)
        assert v2.__str__() == "~[3 3 3 3 3 3 3]"

    def test_set_iterator(self):
        fs = osim.FunctionSet()
        f1 = osim.Constant()
        f1.setName("myfunc1")
        fs.adoptAndAppend(f1)
        f2 = osim.Constant()
        f2.setName("myfunc2")
        fs.adoptAndAppend(f2)
        f3 = osim.Constant()
        f3.setName("myfunc3")
        fs.adoptAndAppend(f3)

        names = ['myfunc1', 'myfunc2', 'myfunc3']
        i = 0
        for func in fs:
            assert func.getName() == names[i]
            i += 1

        # Test key-value iterator.
        j = 0
        for k, v in fs.items():
            assert k == names[j]
            assert k == v.getName()
            j += 1
