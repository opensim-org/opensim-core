"""The tests here ensure the proper functionality of modifications/additions we
make to the C++ API, via the SWIG interface (*.i) file.

"""

import os
import unittest
from math import isnan

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

def createSlidingMassModel():
    model = osim.Model()
    model.setName("sliding_mass")
    model.set_gravity(osim.Vec3(0, 0, 0))
    body = osim.Body("body", 10.0, osim.Vec3(0), osim.Inertia(0))
    model.addComponent(body)

    # Allows translation along x.
    joint = osim.SliderJoint("slider", model.getGround(), body)
    coord = joint.updCoordinate(osim.SliderJoint.Coord_TranslationX)
    coord.setName("position")
    model.addComponent(joint)

    actu = osim.CoordinateActuator()
    actu.setCoordinate(coord)
    actu.setName("actuator")
    actu.setOptimalForce(1)
    actu.setMinControl(-10)
    actu.setMaxControl(10)
    model.addComponent(actu)

    return model

def createDoubleSlidingMassModel():
    model = createSlidingMassModel()
    body = osim.Body("body2", 10.0, osim.Vec3(0), osim.Inertia(0))
    model.addComponent(body)

    joint = osim.SliderJoint("slider2", model.getGround(), body)
    coord = joint.updCoordinate(osim.SliderJoint.Coord_TranslationX)
    coord.setName("position");
    model.addComponent(joint);

    actu = osim.CoordinateActuator()
    actu.setCoordinate(coord)
    actu.setName("actuator2")
    actu.setOptimalForce(1)
    model.addComponent(actu)

    model.finalizeConnections()
    model.initSystem()
    return model


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
        
        study = osim.MocoStudy()
        study.setName('sliding_mass')
        
        mp = study.updProblem()
        
        mp.setModel(model)
        ph0 = mp.getPhase()
        
        mp.setTimeBounds(osim.MocoInitialBounds(0.), 
                osim.MocoFinalBounds(0.1, 5.))
        assert ph0.getTimeInitialBounds().getLower() == 0
        assert ph0.getTimeInitialBounds().getUpper() == 0
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getLower(), 0.1)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getUpper(), 5.0)
        
        mp.setTimeBounds([0.2, 0.3], [3.5])
        self.assertAlmostEqual(ph0.getTimeInitialBounds().getLower(), 0.2)
        self.assertAlmostEqual(ph0.getTimeInitialBounds().getUpper(), 0.3)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getLower(), 3.5)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getUpper(), 3.5)
        
        # Use setter on MocoPhase.
        ph0.setTimeBounds([2.2, 2.3], [4.5])
        self.assertAlmostEqual(ph0.getTimeInitialBounds().getLower(), 2.2)
        self.assertAlmostEqual(ph0.getTimeInitialBounds().getUpper(), 2.3)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getLower(), 4.5)
        self.assertAlmostEqual(ph0.getTimeFinalBounds().getUpper(), 4.5)


        mp.setStateInfo('slider/position/value', osim.MocoBounds(-5, 5),
            osim.MocoInitialBounds(0))
        assert -5 == ph0.getStateInfo('slider/position/value').getBounds().getLower()
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
        
        # Use setter on MocoPhase.
        ph0.setStateInfo('slider/position/speed', [-6, 10], [-4, 3], [0])
        assert -6 == ph0.getStateInfo('slider/position/speed').getBounds().getLower()
        assert 10 == ph0.getStateInfo('slider/position/speed').getBounds().getUpper()
        assert -4 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getLower()
        assert  3 == ph0.getStateInfo('slider/position/speed').getInitialBounds().getUpper()
        assert 0 == ph0.getStateInfo('slider/position/speed').getFinalBounds().getLower()
        assert 0 == ph0.getStateInfo('slider/position/speed').getFinalBounds().getUpper()
        
        # Controls.
        mp.setControlInfo('actuator', osim.MocoBounds(-50, 50))
        assert -50 == ph0.getControlInfo('actuator').getBounds().getLower()
        assert  50 == ph0.getControlInfo('actuator').getBounds().getUpper()
        mp.setControlInfo('actuator', [18])
        assert 18 == ph0.getControlInfo('actuator').getBounds().getLower()
        assert 18 == ph0.getControlInfo('actuator').getBounds().getUpper()

    def test_MocoTrajectory(self):
        time = osim.Vector(3, 0)
        time.set(0, 0)
        time.set(1, 0.1)
        time.set(2, 0.2)
        st = osim.Matrix(3, 2)
        ct = osim.Matrix(3, 3)
        mt = osim.Matrix(3, 1)
        dt = osim.Matrix(3, 1)
        p = osim.RowVector(2, 0.0)
        it = osim.MocoTrajectory(time, ['s0', 's1'], ['c0', 'c1', 'c2'],
                              ['m0'], ['d0'],
                              ['p0', 'p1'], st, ct, mt, dt, p)
        
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

        it.setMultiplier('m0', [326, 1, 42])
        m0traj = it.getMultiplier('m0')
        assert(m0traj[0] == 326)
        assert(m0traj[1] == 1)
        assert(m0traj[2] == 42)

        it.setDerivative('d0', [-10, 477, 125])
        d0traj = it.getDerivative('d0')
        assert(d0traj[0] == -10)
        assert(d0traj[1] == 477)
        assert(d0traj[2] == 125)

        it.setParameter('p0', 25)
        it.setParameter('p1', 30)
        p = it.getParameters()
        assert(p[0] == 25)
        assert(p[1] == 30)
        p0 = it.getParameter('p0')
        assert(p0 == 25)

    def test_MocoTrajectory_numpy(self):
        try:
            import numpy as np
        except ImportError as e:
            print("Could not import numpy; skipping test.")
            return

        time = np.linspace(0, 0.2, 3)
        st = np.random.rand(3, 2)
        ct = np.random.rand(3, 3)
        mt = np.random.rand(3, 1)
        p = np.random.rand(2)
        it = osim.MocoTrajectory(time, ['s0', 's1'], ['c0', 'c1', 'c2'],
                              ['m0'],
                              ['p0', 'p1'], st, ct, mt, p)
        assert (it.getTimeMat() == time).all()
        assert (it.getStatesTrajectoryMat() == st).all()
        assert (it.getControlsTrajectoryMat() == ct).all()
        assert (it.getMultipliersTrajectoryMat() == mt).all()
        assert (it.getParametersMat() == p).all()

        # With derivatives.
        time = np.linspace(0, 0.2, 3)
        st = np.random.rand(3, 2)
        ct = np.random.rand(3, 3)
        mt = np.random.rand(3, 1)
        dt = np.random.rand(3, 4)
        p = np.random.rand(2)
        it = osim.MocoTrajectory(time, ['s0', 's1'], ['c0', 'c1', 'c2'],
                              ['m0'], ['d0', 'd1', 'd2', 'd3'],
                              ['p0', 'p1'], st, ct, mt, dt, p)
        np.allclose(it.getTimeMat(), time)
        assert (it.getTimeMat() == time).all()
        assert (it.getStatesTrajectoryMat() == st).all()
        assert (it.getControlsTrajectoryMat() == ct).all()
        assert (it.getMultipliersTrajectoryMat() == mt).all()
        assert (it.getDerivativesTrajectoryMat() == dt).all()
        assert (it.getParametersMat() == p).all()

    def test_createRep(self):
        model = osim.Model()
        model.setName('sliding_mass')
        model.set_gravity(osim.Vec3(0, 0, 0))
        body = osim.Body('body', 2.0, osim.Vec3(0), osim.Inertia(0))
        model.addComponent(body)

        joint = osim.SliderJoint('slider', model.getGround(), body)
        coord = joint.updCoordinate()
        coord.setName('position')
        model.addComponent(joint)
        model.finalizeConnections()

        study = osim.MocoStudy()
        study.setName('sliding_mass')

        mp = study.updProblem()
        mp.setModel(model)

        pr = mp.createRep();
        assert(len(pr.createStateInfoNames()) == 2);

    def test_StdVectorMocoBounds(self):
        bounds = osim.StdVectorMocoBounds()
        bounds.append(osim.MocoBounds(1.23, 4.56))
        assert(bounds[0].getLower() == 1.23)
        assert(bounds[0].getUpper() == 4.56)

    def test_PositionMotion(self):
        # Test that the PositionMotion class can be created and used.
        model = createSlidingMassModel()
        model.initSystem()

        table = osim.TimeSeriesTable()
        for i in range(10):
            row = osim.RowVector(1, i)
            table.appendRow(i, row)
        table.setColumnLabels(['/slider/position/value'])
        table.addTableMetaDataString('inDegrees', 'no')

        posmot = osim.PositionMotion.createFromTable(model, table)
        assert(posmot.get_functions().getSize() == 1)

class TestWorkflow(unittest.TestCase):

    def test_default_bounds(self):
        study = osim.MocoStudy()
        problem = study.updProblem()
        model = createSlidingMassModel()
        model.finalizeFromProperties()
        coord = model.updComponent("slider/position")
        coord.setRangeMin(-10)
        coord.setRangeMax(15)
        actu = model.updComponent("actuator")
        actu.setMinControl(35)
        actu.setMaxControl(56)
        problem.setModel(model)
        phase0 = problem.getPhase(0)
        # User did not specify state info explicitly.
        with self.assertRaises(RuntimeError):
            phase0.getStateInfo("/slider/position/value")

        rep = problem.createRep()
        info = rep.getStateInfo("/slider/position/value")
        self.assertEqual(info.getBounds().getLower(), -10)
        self.assertEqual(info.getBounds().getUpper(),  15)

        # Default speed bounds.
        info = rep.getStateInfo("/slider/position/speed")
        self.assertEqual(info.getBounds().getLower(), -50)
        self.assertEqual(info.getBounds().getUpper(),  50)

        # No control info stored in the Problem.
        with self.assertRaises(RuntimeError):
            phase0.getControlInfo("/actuator")

        # Obtained from controls.
        info = rep.getControlInfo("/actuator")
        self.assertEqual(info.getBounds().getLower(), 35)
        self.assertEqual(info.getBounds().getUpper(), 56)

        problem.setControlInfo("/actuator", [12, 15])
        probinfo = phase0.getControlInfo("/actuator")
        self.assertEqual(probinfo.getBounds().getLower(), 12)
        self.assertEqual(probinfo.getBounds().getUpper(), 15)

        rep = problem.createRep();
        info = rep.getControlInfo("/actuator")
        self.assertEqual(info.getBounds().getLower(), 12)
        self.assertEqual(info.getBounds().getUpper(), 15)

    def test_changing_time_bounds(self):
        study = osim.MocoStudy()
        problem = study.updProblem()
        problem.setModel(createSlidingMassModel())
        problem.setTimeBounds(0, [0, 10])
        problem.setStateInfo("/slider/position/value", [0, 1], 0, 1)
        problem.setStateInfo("/slider/position/speed", [-100, 100], 0, 0)
        problem.setControlInfo("/actuator", [-10, 10])
        problem.addGoal(osim.MocoFinalTimeGoal())

        if osim.MocoCasADiSolver.isAvailable():
            solver = study.initCasADiSolver()
            solver.set_transcription_scheme("trapezoidal")
            solver.set_num_mesh_intervals(19)
            guess = solver.createGuess("random")
            guess.setTime(osim.createVectorLinspace(20, 0.0, 3.0))
            solver.setGuess(guess)
            solution0 = study.solve()

            problem.setTimeBounds(0, [5.8, 10])
            # Editing the problem does not affect information in the Solver; the
            # guess still exists.
            assert(not solver.getGuess().empty())

            solution = study.solve()
            self.assertAlmostEqual(solution.getFinalTime(), 5.8, places=5)

    def test_changing_model(self):
        study = osim.MocoStudy()
        problem = study.updProblem()
        model = createSlidingMassModel()
        problem.setModel(model)
        problem.setTimeBounds(0, [0, 10])
        problem.setStateInfo("/slider/position/value", [0, 1], 0, 1)
        problem.setStateInfo("/slider/position/speed", [-100, 100], 0, 0)
        problem.addGoal(osim.MocoFinalTimeGoal())
        if osim.MocoCasADiSolver.isAvailable():
            solver = study.initCasADiSolver()
            solver.set_num_mesh_intervals(19)
            solver.set_transcription_scheme("trapezoidal");
            finalTime0 = study.solve().getFinalTime()

            self.assertAlmostEqual(finalTime0, 2.00, places=2)

            body = model.updComponent("body")
            body.setMass(2 * body.getMass())
            finalTime1 = study.solve().getFinalTime()
            assert(finalTime1 > 1.1 * finalTime0)

    def test_order(self):
        # Can set the cost and model in any order.
        study = osim.MocoStudy()
        problem = study.updProblem()
        problem.setTimeBounds(0, [0, 10])
        problem.setStateInfo("/slider/position/value", [0, 1], 0, 1)
        problem.setStateInfo("/slider/position/speed", [-100, 100], 0, 0)
        problem.addGoal(osim.MocoFinalTimeGoal())
        problem.setModel(createSlidingMassModel())
        if osim.MocoCasADiSolver.isAvailable():
            solver = study.initCasADiSolver()
            solver.set_num_mesh_intervals(19)
            solver.set_transcription_scheme("trapezoidal")
            finalTime =  study.solve().getFinalTime()
            self.assertAlmostEqual(finalTime, 2.0, places=2)

    def test_changing_costs(self):
        # Changes to the costs are obeyed.
        study = osim.MocoStudy()
        problem = study.updProblem()
        problem.setModel(createSlidingMassModel())
        problem.setTimeBounds(0, [0, 10])
        problem.setStateInfo("/slider/position/value", [0, 1], 0, 1)
        problem.setStateInfo("/slider/position/speed", [-100, 100], 0, 0)
        problem.updPhase().addGoal(osim.MocoFinalTimeGoal())
        effort = osim.MocoControlGoal("effort")
        problem.updPhase().addGoal(effort)
        if osim.MocoCasADiSolver.isAvailable():
            solver = study.initCasADiSolver()
            solver.set_transcription_scheme("trapezoidal");
            finalTime0 = study.solve().getFinalTime()

            # Change the weights of the costs.
            effort.setWeight(0.1)
            assert(study.solve().getFinalTime() < 0.8 * finalTime0)

    def test_expression_based_parameter_goal(self):
        study = osim.MocoStudy()
        mp = study.updProblem()
        mp.setModel(createDoubleSlidingMassModel())
        mp.setTimeBounds(0, 1)
        mp.setStateInfo("/slider/position/value", [-5, 5], 0, [0.2, 0.3])
        mp.setStateInfo("/slider/position/speed", [-20, 20])
        mp.setStateInfo("/slider2/position/value", [-5, 5], 1, [1.2, 1.3])
        mp.setStateInfo("/slider2/position/speed", [-20, 20])

        parameter = osim.MocoParameter("sphere_mass", "body", "mass",
                                    osim.MocoBounds(0, 10))
        mp.addParameter(parameter)
        parameter2 = osim.MocoParameter("sphere2_mass", "body2", "mass",
                                     osim.MocoBounds(0, 10))
        mp.addParameter(parameter2)
        total_weight = 7
        mass_goal = osim.MocoExpressionBasedParameterGoal()
        mp.addGoal(mass_goal)
        mass_goal.setExpression(f"(p+q-{total_weight})^2")
        mass_goal.addParameter(parameter, "p")
        mass_goal.addParameter(parameter2, "q")

        ms = study.initCasADiSolver()
        ms.set_num_mesh_intervals(25)
        sol = study.solve()

        self.assertAlmostEqual(sol.getParameter("sphere_mass") + sol.getParameter("sphere2_mass"),
                               total_weight)
