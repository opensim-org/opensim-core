"""The tests here ensure the proper functionality of modifications/additions we
make to the C++ API, via the SWIG interface (*.i) file.

"""

import inspect
import os

this_file_dir = os.path.dirname(
                os.path.abspath(inspect.getfile(inspect.currentframe())))

import opensim as osim

def test_markAdopted1():
    """Ensures that we can tell an object that some other object is managing
    its memory.
    """
    a = osim.Model()
    assert a.this
    assert a.thisown
    a._markAdopted()
    assert a.this
    assert not a.thisown

def test_markAdopted2():
    a = osim.Model()

    # We just need the following not to cause a segfault.

    # Model add*
    a.addForce(osim.PathActuator())
    a.addProbe(osim.Umberger2010MuscleMetabolicsProbe())
    a.addAnalysis(osim.MuscleAnalysis())
    a.addController(osim.PrescribedController())
    
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

    f = osim.BushingForce("ground", "body",
            osim.Vec3(2, 2, 2), osim.Vec3(1, 1, 1),
            osim.Vec3(0, 0, 0), osim.Vec3(0, 0, 0))
    a.addForce(f)

    f2 = osim.BushingForce()
    a.addForce(f2)

    f3 = osim.SpringGeneralizedForce()
    a.addForce(f3)

    model = osim.Model(os.path.join(this_file_dir, "arm26.osim"))
    g = osim.CoordinateActuator('r_shoulder_elev')
    model.addForce(g)

def test_Joint():
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


def test_markAdoptedSets():

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

