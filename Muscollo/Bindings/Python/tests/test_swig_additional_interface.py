"""The tests here ensure the proper functionality of modifications/additions we
make to the C++ API, via the SWIG interface (*.i) file.

"""

import os
import unittest
from math import isnan

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

class TestSwigAddtlInterface(unittest.TestCase):
    def test_bounds(self):
        model = osim.Model()
        model.setName('sliding_mass')
        model.set_gravity(osim.Vec3(0, 0, 0))
        body = osim.Body('body', 2.0, osim.Vec3(0), osim.Inertia(0))
        model.addComponent(body)
        
        joint = osim.SliderJoint('slider', model.getGround(), body)
        coord = joint.updCoordinate()
        coord.setName('position')
        model.addComponent(joint)
        
        actu = osim.CoordinateActuator()
        actu.setCoordinate(coord)
        actu.setName('actuator')
        actu.setOptimalForce(1)
        model.addComponent(actu)
        
        muco = osim.MucoTool()
        muco.setName('sliding_mass')
        
        mp = muco.updProblem()
        
        mp.setModel(model)
        ph0 = mp.getPhase()
        
        mp.setTimeBounds(osim.MucoInitialBounds(0.), 
                osim.MucoFinalBounds(0.1, 5.))
        assert ph0.getTimeInitialBounds().getLower() == 0
        assert ph0.getTimeInitialBounds().getUpper() == 0
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getLower(), 0.1)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getUpper(), 5.0)
        
        mp.setTimeBounds([0.2, 0.3], [3.5])
        self.assertAlmostEqual(ph0.getTimeInitialBounds().getLower(), 0.2)
        self.assertAlmostEqual(ph0.getTimeInitialBounds().getUpper(), 0.3)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getLower(), 3.5)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getUpper(), 3.5)
        
        # Use setter on MucoPhase.
        ph0.setTimeBounds([2.2, 2.3], [4.5])
        self.assertAlmostEqual(ph0.getTimeInitialBounds().getLower(), 2.2)
        self.assertAlmostEqual(ph0.getTimeInitialBounds().getUpper(), 2.3)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getLower(), 4.5)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getUpper(), 4.5)
        
        
        mp.setStateInfo('slider/position/value', osim.MucoBounds(-5, 5),
            osim.MucoInitialBounds(0))
        assert-5 == ph0.getStateInfo('slider/position/value').getBounds().getLower()
        assert 5 == ph0.getStateInfo('slider/position/value').getBounds().getUpper()
        assert isnan(ph0.getStateInfo('slider/position/value').getFinalBounds().getLower())
        assert isnan(ph0.getStateInfo('slider/position/value').getFinalBounds().getUpper())
        mp.setStateInfo('slider/position/speed', [-50, 50], [-3], 1.5)
        assert -50 == ph0.getStateInfo('slider/position/speed').getBounds().getLower()
        assert  50 == ph0.getStateInfo('slider/position/speed').getBounds().getUpper()
        assert -3 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getLower()
        assert -3 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getUpper()
        self.assertAlmostEqual(1.5,
            ph0.getStateInfo('slider/position/speed').getFinalBounds().getLower())
        self.assertAlmostEqual(1.5,
            ph0.getStateInfo('slider/position/speed').getFinalBounds().getUpper())
        
        # Use setter on MucoPhase.
        ph0.setStateInfo('slider/position/speed', [-6, 10], [-4, 3], [0])
        assert -6 == ph0.getStateInfo('slider/position/speed').getBounds().getLower()
        assert 10 == ph0.getStateInfo('slider/position/speed').getBounds().getUpper()
        assert -4 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getLower()
        assert  3 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getUpper()
        assert 0 == ph0.getStateInfo('slider/position/speed').getFinalBounds().getLower()
        assert 0 == ph0.getStateInfo('slider/position/speed').getFinalBounds().getUpper()
        
        # Controls.
        mp.setControlInfo('actuator', osim.MucoBounds(-50, 50))
        assert -50 == ph0.getControlInfo('actuator').getBounds().getLower()
        assert  50 == ph0.getControlInfo('actuator').getBounds().getUpper()
        mp.setControlInfo('actuator', [18])
        assert 18 == ph0.getControlInfo('actuator').getBounds().getLower()
        assert 18 == ph0.getControlInfo('actuator').getBounds().getUpper()

    def test_MucoIterate(self):
        time = osim.Vector(3, 0)
        time.set(0, 0)
        time.set(1, 0.1)
        time.set(2, 0.2)
        st = osim.Matrix(3, 2)
        ct = osim.Matrix(3, 3)
        p = osim.RowVector(2, 0.0)
        it = osim.MucoIterate(time, ['s0', 's1'], ['c0', 'c1', 'c2'],
                              ['p0', 'p1'], st, ct, p)
        
        it.setTime([15, 25, 35])
        assert(it.getTime().get(0) == 15)
        assert(it.getTime().get(1) == 25)
        assert(it.getTime().get(2) == 35)

        it.setState('s0', [5, 3, 10])
        s0traj = it.getState('s0')
        assert(s0traj[0] == 5)
        assert(s0traj[1] == 3)
        assert(s0traj[2] == 10)
        it.setState('s1', [2, 6, 1])
        s1traj = it.getState('s1')
        assert(s1traj[0] == 2)
        assert(s1traj[1] == 6)
        assert(s1traj[2] == 1)

        it.setControl('c0', [10, 46, -5])
        c0traj = it.getControl('c0')
        assert(c0traj[0] == 10)
        assert(c0traj[1] == 46)
        assert(c0traj[2] == -5)
        it.setControl('c2', [5, 12, -1])
        c2traj = it.getControl('c2')
        assert(c2traj[0] == 5)
        assert(c2traj[1] == 12)
        assert(c2traj[2] == -1)

        it.setParameter('p0', 25)
        it.setParameter('p1', 30)
        p = it.getParameters()
        assert(p[0] == 25)
        assert(p[1] == 30)
        p0 = it.getParameter('p0')
        assert(p0 == 25)
