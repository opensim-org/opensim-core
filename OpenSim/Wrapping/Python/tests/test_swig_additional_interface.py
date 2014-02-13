"""The tests here ensure the proper functionality of modifications/additions we
make to the C++ API, via the SWIG interface (*.i) file.

"""

import opensim as osim

def test_markAdopted():
    """Ensures that we can tell an object that some other object is managing
    its memory.
    """
    a = osim.Model()
    assert a.this
    assert a.thisown
    a._markAdopted()
    assert a.this
    assert not a.thisown

    # We just need the following not to not cause a segfault.

    # Model add*
    a.addComponent(osim.PathActuator())
    a.addProbe(osim.Umberger2010MuscleMetabolicsProbe())
    try:
        # It's okay if throws an exception (it does); we just need
        # to avoid the segfault.
        a.addConstraint(osim.ConstantDistanceConstraint())
    except:
        pass
    a.addAnalysis(osim.MuscleAnalysis())
    #a.addForce(osim.BushingForce())
    #a.addForce(osim.CoordinateActuator())
    a.addController(osim.PrescribedController())

    # Set's.
    #fus = osim.FunctionSet()
    #fu1 = osim.Constant()
    #fus.adoptAndAppend(fu1)

    gs = osim.GeometrySet()
    dg = osim.DisplayGeometry()
    gs.adoptAndAppend(dg)

    s = osim.ScaleSet()
    o = osim.Scale()
    s.adoptAndAppend(o)

    s = osim.ForceSet()
    o = osim.BushingForce()
    s.adoptAndAppend(o)

    cs = osim.ControllerSet()
    csc = osim.PrescribedController()
    cs.adoptAndAppend(csc)

    s = osim.ContactGeometrySet()
    o = osim.ContactHalfSpace()
    s.adoptAndAppend(o)

    s = osim.AnalysisSet()
    o = osim.MuscleAnalysis()
    s.adoptAndAppend(o)

    s = osim.ControlSet()
    o = osim.ControlLinear()
    s.adoptAndAppend(o)

    s = osim.MarkerSet()
    o = osim.Marker()
    s.adoptAndAppend(o)

    s = osim.BodySet()
    o = osim.Body()
    s.adoptAndAppend(o)

    s = osim.BodyScaleSet()
    o = osim.BodyScale()
    s.adoptAndAppend(o)

    s = osim.CoordinateSet()
    o = osim.Coordinate()
    s.adoptAndAppend(o)

    s = osim.JointSet()
    o = osim.BallJoint()
    s.adoptAndAppend(o)

    s = osim.ConstraintSet()
    o = osim.ConstantDistanceConstraint()
    s.adoptAndAppend(o)

    s = osim.PathPointSet()
    o = osim.PathPoint()
    s.adoptAndAppend(o)

    s = osim.IKTaskSet()
    o = osim.IKMarkerTask()
    s.adoptAndAppend(o)

    s = osim.MarkerPairSet()
    o = osim.MarkerPair()
    s.adoptAndAppend(o)

    s = osim.MeasurementSet()
    o = osim.Measurement()
    s.adoptAndAppend(o)

