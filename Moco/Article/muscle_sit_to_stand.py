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
    predict_solution_file = 'predict_solution.sto'
    predict_assisted_solution_file = 'predict_assisted_solution.sto'
    track_solution_file = 'track_solution.sto'
    inverse_solution_file = 'inverse_solution.sto'
    inverse_assisted_solution_file = 'inverse_assisted_solution.sto'
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
        solution.write(predict_solution_file)
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

    def predict_assisted():
        model = muscle_driven_model()
        device = osim.SpringGeneralizedForce('knee_angle_r')
        device.setName('spring')
        device.setStiffness(50)
        device.setRestLength(0)
        device.setViscosity(0)
        model.addForce(device)

        moco = create_tool(model)
        problem = moco.updProblem()
        problem.addCost(osim.MocoControlCost('effort'))

        problem.addParameter(
            osim.MocoParameter('stiffness', '/forceset/spring',
                               'stiffness', osim.MocoBounds(10, 80)))

        solver = osim.MocoCasADiSolver.safeDownCast(moco.updSolver())
        solver.resetProblem(problem)

        solver.set_num_mesh_points(10)
        solver.set_parameters_require_initsystem(False)
        solution = moco.solve()
        solution.write(predict_assisted_solution_file)

    def track():
        moco = create_tool(muscle_driven_model())
        problem = moco.updProblem()
        tracking = osim.MocoStateTrackingCost()
        tracking.setName('tracking')
        tracking.setReferenceFile(predict_solution_file)
        tracking.setAllowUnusedReferences(True)
        problem.addCost(tracking)

        problem.addCost(osim.MocoControlCost('effort', 0.001))

        solver = osim.MocoCasADiSolver.safeDownCast(moco.updSolver())
        solver.resetProblem(problem)

        solver.set_num_mesh_points(10)
        solution = moco.solve()
        solution.write(track_solution_file)

    def inverse():
        inverse = osim.MocoInverse()
        inverse.setModel(muscle_driven_model())
        inverse.setKinematicsFile(predict_solution_file)
        inverse.set_kinematics_allow_extra_columns(True)
        inverse.set_lowpass_cutoff_frequency_for_kinematics(6)
        inverse.set_mesh_interval(0.05)
        inverse.set_create_reserve_actuators(2)
        inverse.set_minimize_sum_squared_states(True)
        inverse.set_tolerance(1e-4)
        # inverse.append_output_paths('.*normalized_fiber_length')
        # inverse.append_output_paths('.*passive_force_multiplier')
        inverseSolution = inverse.solve()
        inverseSolution.getMocoSolution().write(inverse_solution_file)
        # inverseOutputs = inverseSolution.getOutputs()
        # osim.STOFileAdapter.write(inverseOutputs, 'muscle_outputs.sto')
        # print('Cost without device: %f' %
        #       inverseSolution.getMocoSolution().getObjective())

    def inverse_assisted():
        model = muscle_driven_model()
        device = osim.SpringGeneralizedForce('knee_angle_r')
        device.setStiffness(50)
        device.setRestLength(0)
        device.setViscosity(0)
        model.addForce(device)

        inverse = osim.MocoInverse()
        inverse.setModel(model)
        inverse.setKinematicsFile(predict_solution_file)
        inverse.set_kinematics_allow_extra_columns(True)
        inverse.set_lowpass_cutoff_frequency_for_kinematics(6)
        inverse.set_mesh_interval(0.05)
        inverse.set_create_reserve_actuators(2)
        inverse.set_minimize_sum_squared_states(True)
        inverse.set_tolerance(1e-4)
        inverseAssistedSolution = inverse.solve()
        inverseAssistedSolution.getMocoSolution().write(
            inverse_assisted_solution_file)

    if run:
        predict_solution = predict()
        predict_assisted()
        track_solution = track()
        inverse()
        inverse_assisted()

    if plot:
        fig = plt.figure(figsize=(6, 7))
        values = [
            '/jointset/hip_r/hip_flexion_r/value',
            '/jointset/knee_r/knee_angle_r/value',
            '/jointset/ankle_r/ankle_angle_r/value',
        ]
        # TODO hip flexion is misnamed.
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
        grid = plt.GridSpec(9, 2, hspace=0.7,
                            left=0.1, right=0.98, bottom=0.07, top=0.96,
                            )
        coord_axes = []
        for ic, coordvalue in enumerate(values):
            ax = fig.add_subplot(grid[3 * ic: 3 * (ic + 1), 0])
            ax.set_ylabel('%s (degrees)' % coord_names[coordvalue])
            if ic == len(values) - 1:
                ax.set_xlabel('time (s)')
            else:
                ax.set_xticklabels([])
            professional_spines(ax)
            coord_axes += [ax]
        muscle_axes = []
        for im, muscle in enumerate(muscles):
            ax = fig.add_subplot(grid[im, 1])
            ax.set_title('%s activation' % muscle[:-2])
            ax.set_ylim([0, 1])
            if im == len(muscles) - 1:
                ax.set_xlabel('time (s)')
            else:
                ax.set_xticklabels([])
            professional_spines(ax)
            muscle_axes += [ax]
        def plot_solution(sol, label, linestyle='-'):
            time = sol.getTimeMat()
            for ic, coordvalue in enumerate(values):
                ax = coord_axes[ic]
                if ic == 0:
                    use_label = label
                else:
                    use_label = None
                if coordvalue in sol.getStateNames():
                    y = (coord_signs[coordvalue] * np.rad2deg(
                        sol.getStateMat(coordvalue)))
                    ax.plot(time, y, linestyle=linestyle, label=use_label)
                    ax.set_xlim(time[0], time[-1])
                else:
                    if ic == 0:
                        ax.plot(0, 0, label=use_label)
            for im, muscle in enumerate(muscles):
                ax = muscle_axes[im]
                ax.plot(time,
                        sol.getStateMat(
                            '/forceset/%s/activation' % muscle),
                        linestyle=linestyle)
                ax.set_xlim(time[0], time[-1])


        predict_solution = osim.MocoIterate(predict_solution_file)
        predict_assisted_solution = osim.MocoIterate(predict_assisted_solution_file)
        track_solution = osim.MocoIterate(track_solution_file)
        inverse_solution = osim.MocoIterate(inverse_solution_file)
        inverse_assisted_solution = osim.MocoIterate(inverse_assisted_solution_file)
        plot_solution(predict_solution, 'predict')
        plot_solution(predict_assisted_solution, 'predict assisted')
        plot_solution(track_solution, 'track', linestyle='--')
        plot_solution(inverse_solution, 'inverse', linestyle='--')
        plot_solution(inverse_assisted_solution, 'inverse assisted', linestyle='--')
        coord_axes[0].legend(frameon=False)
        fig.savefig('muscle_sit_to_stand.png', dpi=600)

        # TODO compute reserve forces RMS.


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
