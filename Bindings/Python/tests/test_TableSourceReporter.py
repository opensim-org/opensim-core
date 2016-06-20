import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Tests TableSource and Reporter (including TableReporter) classes.
class TestTableSourceReporter(unittest.TestCase):
    def test_ConsoleReporter(self):
        m = osim.Model()
        b1 = osim.Body("b1", 1, osim.Vec3(1, 0, 0), osim.Inertia(1))
        j1 = osim.PinJoint("j1", m.getGround(), osim.Vec3(0), osim.Vec3(0),
                                b1, osim.Vec3(0), osim.Vec3(0))
        coord = j1.getCoordinateSet().get(0)
        coord.setDefaultValue(0.5)
        rep = osim.ConsoleReporter()
        m.addBody(b1)
        m.addJoint(j1)
        # TODO this first call to initSystem() is here b/c of a bug where
        # Ground's MobilizedBodyIndex is not available b/c extendAddtoSystem()
        # has not yet been invoked on it.
        s = m.initSystem()
        m.addComponent(rep)
        rep.updInput("inputs").connect(coord.getOutput("value"))
        s = m.initSystem()
        m.realizeReport(s)


