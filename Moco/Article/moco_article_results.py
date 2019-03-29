import opensim as osim
import numpy as np
import pylab as pl

pl.ion()


def Kirk():
    model = osim.Model()
    body = osim.Body("b", 1, osim.Vec3(0), osim.Inertia(0))
    model.addBody(body);

    joint = osim.SliderJoint("j", model.getGround(), body)
    joint.updCoordinate().setName("coord")
    model.addJoint(joint);

    damper = osim.SpringGeneralizedForce("coord")
    damper.setViscosity(-1.0)
    model.addForce(damper)

    actu = osim.CoordinateActuator("coord")
    model.addForce(actu)
    model.finalizeConnections()

    moco = osim.MocoTool()
    problem = moco.updProblem()

    problem.setModel(model)
    problem.setTimeBounds(0, 2)
    problem.setStateInfo("/jointset/j/coord/value", [-10, 10], 0, 5)
    problem.setStateInfo("/jointset/j/coord/speed", [-10, 10], 0, 2)
    problem.setControlInfo("/forceset/coordinateactuator", [-50, 50])

    problem.addCost(osim.MocoControlCost("effort", 0.5))

    solver = moco.initCasADiSolver();
    solver.set_optim_hessian_approximation("limited-memory");
    solver.set_verbosity(0)
    solver.set_parallel(0)

    def expectedSolution(time):
        A = np.matrix([[-2.0 - 0.5 * np.exp(-2.0) + 0.5 * np.exp(2.0),
                        1.0 - 0.5 * np.exp(-2.0) - 0.5 * np.exp(2.0)],
                       [-1.0 + 0.5 * np.exp(-2.0) + 0.5 * np.exp(2.0),
                        0.5 * np.exp(-2.0) - 0.5 * np.exp(2.0)]])
        b = np.matrix([[5], [2]])
        c = np.linalg.solve(A, b)
        c2 = c[0]
        c3 = c[1]

        def y0(t):
            return (c2 * (-t - 0.5 * np.exp(-t) + 0.5 * np.exp(t)) +
                    c3 * (1.0 - 0.5 * np.exp(-t) - 0.5 * np.exp(t)))

        def y1(t):
            return (c2 * (-1.0 + 0.5 * np.exp(-t) + 0.5 * np.exp(t)) +
                    c3 * (0.5 * np.exp(-t) - 0.5 * np.exp(t)))

        sol = np.empty((len(time), 2))
        for i in range(len(time)):
            sol[i, 0] = y0(time[i])
            sol[i, 1] = y1(time[i])
        return sol

    N = 10
    N_list = []
    error_list = []
    solver_duration = []
    while N < 10000:
        solver.set_num_mesh_points(N)
        solution = moco.solve()
        actual_y0 = solution.getStateMat('/jointset/j/coord/value')
        actual_y1 = solution.getStateMat('/jointset/j/coord/speed')
        actual = np.empty((N, 2))
        actual[:, 0] = np.array(actual_y0)
        actual[:, 1] = np.array(actual_y1)
        diff = actual - expectedSolution(solution.getTimeMat())
        error = np.sqrt(np.mean(np.square(diff.flatten())))
        print("N: {}. Error: {}".format(N, error))
        N_list += [N]
        error_list += [error]
        solver_duration += [solution.getSolverDuration()]
        N *= 2

    pl.figure()
    pl.plot(N_list, np.log10(np.array(error_list)))
    pl.figure()
    pl.plot(N_list, solver_duration)
    pl.show()


def brachistochrone():
    model = osim.ModelFactory.createBrachistochrone()

    moco = osim.MocoTool()
    problem = moco.updProblem()

    problem.setModel(model)
    problem.setTimeBounds(0, [0, 10])
    problem.setStateInfo("/brachistochrone/x", [-10, 10], 0, 1)
    problem.setStateInfo("/brachistochrone/y", [-10, 10], 0)
    problem.setStateInfo("/brachistochrone/v", [-10, 10], 0)
    problem.setControlInfo("/brachistochrone", [-100, 100])

    problem.addCost(osim.MocoFinalTimeCost())

    # problem.addCost(osim.MocoControlCost('effort', 0.01))

    solver = moco.initCasADiSolver();
    solver.set_optim_hessian_approximation("limited-memory");
    solver.set_verbosity(0)
    solver.set_parallel(0)

    # solution = moco.solve()
    # print(solution.getControlsTrajectoryMat())
    # pl.plot(solution.getStateMat('/brachistochrone/x'),
    #         solution.getStateMat('/brachistochrone/y'))
    # pl.figure();
    # pl.plot(solution.getTimeMat(), solution.getControlsTrajectoryMat())
    # pl.show()

    def expectedSolution(time):
        return np.nan

    N = 10
    N_list = []
    error_list = []
    solver_duration = []
    while N < 1000:
        solver.set_num_mesh_points(N)
        solution = moco.solve()
        # actual_y0 = solution.getStateMat('/jointset/j/coord/value')
        # actual_y1 = solution.getStateMat('/jointset/j/coord/speed')
        # actual = np.empty((N, 2))
        # actual[:, 0] = np.array(actual_y0)
        # actual[:, 1] = np.array(actual_y1)
        # diff = actual - expectedSolution(solution.getTimeMat())
        # error = np.sqrt(np.mean(np.square(diff.flatten())))
        error = 1000.0
        duration = solution.getSolverDuration()
        print("N: {}. Error: {}. Duration: {}".format(N, error, duration))
        N_list += [N]
        error_list += [error]
        solver_duration += [duration]
        N *= 2


    pl.figure()
    pl.plot(N_list, np.log10(np.array(error_list)))
    pl.figure()
    pl.plot(N_list, solver_duration)
    pl.show()


if __name__ == "__main__":
    brachistochrone()
# TODO linear tangent steering has analytical solution Example 4.1 Betts, and Example 4.5

# 2 dof 3 muscles, predict, time-stepping, and track. add noise!!!
