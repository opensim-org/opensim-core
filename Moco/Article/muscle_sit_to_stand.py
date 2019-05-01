import opensim as osim
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

plt.ion()
mpl.rcParams.update({'font.size': 10,
                     'axes.titlesize': 'medium',
                     'font.sans-serif': ['Helvetica', 'Arial']})

def professional_spines(axes):
    axes.spines['right'].set_visible(False)
    axes.yaxis.set_ticks_position('left')
    axes.spines['top'].set_visible(False)
    axes.xaxis.set_ticks_position('bottom')


def muscle_sit_to_stand(run, plot):
    def create_tool(model):
        moco = osim.MocoTool()
        solver = moco.initCasADiSolver()
        solver.set_dynamics_mode('implicit')
        solver.set_optim_convergence_tolerance(1e-4)
        solver.set_optim_constraint_tolerance(1e-4)
        solver.set_optim_solver('ipopt')
        solver.set_transcription_scheme('hermite-simpson')
        solver.set_enforce_constraint_derivatives(True)
        solver.set_optim_hessian_approximation('limited-memory')
        solver.set_optim_finite_difference_scheme('forward')

        problem = moco.updProblem()
        problem.setModelCopy(model)
        problem.setTimeBounds(0, 1)
        # TODO remove these for tracking:
        # The position bounds specify that the model should start in a crouch and
        # finish standing up.
        problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value',
                             [-2, 0.5], -2, 0)
        problem.setStateInfo('/jointset/knee_r/knee_angle_r/value',
                             [-2, 0], -2, 0)
        problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value',
                             [-0.5, 0.7], -0.5, 0)
        # The velocity bounds specify that the model coordinates should start and
        # end at zero.
        problem.setStateInfo('/jointset/hip_r/hip_flexion_r/speed',
                             [-50, 50], 0, 0)
        problem.setStateInfo('/jointset/knee_r/knee_angle_r/speed',
                             [-50, 50], 0, 0)
        problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/speed',
                             [-50, 50], 0, 0)

        for muscle in model.getMuscles():
            if not muscle.get_ignore_activation_dynamics():
                muscle_path = muscle.getAbsolutePathString()
                problem.setStateInfo(muscle_path + '/activation', [0, 1], 0)
                problem.setControlInfo(muscle_path, [0, 1], 0)
        return moco

    def add_CoordinateActuator(model, coord_name, optimal_force):
        coord_set = model.updCoordinateSet()
        actu = osim.CoordinateActuator()
        actu.setName('tau_' + coord_name)
        actu.setCoordinate(coord_set.get(coord_name))
        actu.setOptimalForce(optimal_force)
        actu.setMinControl(-1)
        actu.setMaxControl(1)
        model.addForce(actu)

    def muscle_driven_model():
        model = osim.Model('sitToStand_3dof9musc.osim')
        model.finalizeConnections()
        osim.DeGrooteFregly2016Muscle.replaceMuscles(model)
        for muscle in model.getMuscles():
            muscle.set_ignore_tendon_compliance(True)
            muscle.set_max_isometric_force(2 * muscle.get_max_isometric_force())
            dgf = osim.DeGrooteFregly2016Muscle.safeDownCast(muscle)
            dgf.set_active_force_width_scale(1.5)
            if muscle.getName() == 'soleus_r':
                dgf.set_ignore_passive_fiber_force(True)
        add_CoordinateActuator(model, 'hip_flexion_r', 10)
        add_CoordinateActuator(model, 'knee_angle_r', 10)
        add_CoordinateActuator(model, 'ankle_angle_r', 10)
        return model

    def predict():
        moco = create_tool(muscle_driven_model())
        problem = moco.updProblem()
        problem.addCost(osim.MocoControlCost('effort'))

        solver = osim.MocoCasADiSolver.safeDownCast(moco.updSolver())
        solver.resetProblem(problem)

        solver.set_num_mesh_points(10)
        solution = moco.solve()
        solution.write('predictSolution0.sto')
        # solver.setGuess(predictSolution0)
        # solver.set_num_mesh_points(20)
        # # solver.set_parallel(0)
        # predictSolution1 = moco.solve().unseal()
        # predictSolution1.write('predictSolution1.sto')
        # moco.visualize(predictSolution1)

        # TODO: Does not track well!
        # timeStepSolution = osim.simulateIterateWithTimeStepping(
        #     predictSolution, problem.getPhase(0).getModel(),
        #     1e-8)
        # timeStepSolution.write('timeStepSolution.sto')
        # moco.visualize(timeStepSolution)

        return solution

    if run:
        predict_solution = predict()

    if plot:
        predict_solution = osim.MocoIterate('predictSolution0.sto')
        fig = plt.figure(figsize=(6, 7))
        values = [
            '/jointset/hip_r/hip_flexion_r/value',
            '/jointset/knee_r/knee_angle_r/value',
            '/jointset/ankle_r/ankle_angle_r/value',
        ]
        # TODO negate sign of hip flexion
        coord_names = {
            '/jointset/hip_r/hip_flexion_r/value': 'hip flexion',
            '/jointset/knee_r/knee_angle_r/value': 'knee flexion',
            '/jointset/ankle_r/ankle_angle_r/value': 'ankle plantarflexion',
        }
        coord_signs = {
            '/jointset/hip_r/hip_flexion_r/value': -1.0,
            '/jointset/knee_r/knee_angle_r/value': -1.0,
            '/jointset/ankle_r/ankle_angle_r/value': 1.0,

        }

        grid = plt.GridSpec(9, 2, hspace=0.7,
                            left=0.1, right=0.98, bottom=0.07, top=0.96,
                            )
        time = predict_solution.getTimeMat()
        for ic, coordvalue in enumerate(values):
            ax = fig.add_subplot(grid[3 * ic: 3 * (ic + 1), 0])
            y = (coord_signs[coordvalue] * np.rad2deg(
                predict_solution.getStateMat(coordvalue)))

            ax.plot(time, y, color='k')
            ax.set_ylabel('%s (degrees)' % coord_names[coordvalue])
            ax.set_xlim(time[0], time[-1])
            if ic == len(values) - 1:
                ax.set_xlabel('time (s)')
            else:
                ax.set_xticklabels([])
            professional_spines(ax)

        muscles = [
            'glut_max2_r',
            'psoas_r',
            'semimem_r',
            'rect_fem_r',
            'bifemsh_r',
            'vas_int_r',
            'med_gas_r',
            'soleus_r',
            'tib_ant_r',
        ]
        for im, muscle in enumerate(muscles):
            ax = fig.add_subplot(grid[im, 1])
            ax.plot(time,
                    predict_solution.getStateMat(
                        '/forceset/%s/activation' % muscle),
                    color='k')
            ax.set_title('%s activation' % muscle[:-2])
            ax.set_xlim(time[0], time[-1])
            ax.set_ylim([0, 1])
            if im == len(muscles) - 1:
                ax.set_xlabel('time (s)')
            else:
                ax.set_xticklabels([])
            professional_spines(ax)
        fig.savefig('muscle_sit_to_stand.png', dpi=600)


if __name__ == "__main__":
    import sys
    import optparse

    parser = optparse.OptionParser()
    parser.add_option("-r", "--run", dest="run",
                      action="store_true",
                      help="Only run the optimizations.", default=False)
    parser.add_option("-p", "--plot", dest="plot",
                      action="store_true",
                      help="Only plot results.", default=False)
    (options, args) = parser.parse_args()
    run = True
    plot = True
    if options.run and options.plot:
        raise RuntimeError("Cannot pass both --run and --plot.")
    if options.run: plot = False
    if options.plot: run = False
    muscle_sit_to_stand(run, plot)

# 2 dof 3 muscles, predict, time-stepping, and track. add noise!!!
