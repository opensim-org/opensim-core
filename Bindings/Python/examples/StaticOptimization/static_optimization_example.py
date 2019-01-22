import os
import ipopt
import numpy as np

from static_optim.dynamic_models import ClassicalStaticOptimization, ClassicalOptimizationLinearConstraints

setup = {
    "model": "data/arm26.osim",
    "mot": "data/arm26_InverseKinematics.mot",
    "filter_param": None,
    "muscle_physiology": True,
}
show_results = False  # Necessitate that pyomeca and matplotlib are installed on the computer

# non-linear static optimization model
if os.path.isfile('data_non_linear_all.npy'):
    # If possible load the data since non-linear is very slow
    calc_non_linear = False
    x0_non_linear_all = np.load('data_non_linear_all.npy')
    info_non_linear = dict()
    info_non_linear["obj_val"] = 0
    info_non_linear["g"] = 0

else:
    calc_non_linear = True
    x0_non_linear_all = []
    model_non_linear = ClassicalStaticOptimization(setup["model"], setup["mot"], setup["filter_param"])

    # optimization options
    activation_initial_guess_non_linear = np.zeros([model_non_linear.n_muscles])
    lb_non_linear, ub_non_linear = model_non_linear.get_bounds()

    # problem
    problem_non_linear = ipopt.problem(
        n=model_non_linear.n_muscles,  # Nb of variables
        lb=lb_non_linear,  # Variables lower bounds
        ub=ub_non_linear,  # Variables upper bounds
        m=model_non_linear.n_dof,  # Nb of constraints
        cl=np.zeros(model_non_linear.n_dof),  # Lower bound constraints
        cu=np.zeros(model_non_linear.n_dof),  # Upper bound constraints
        problem_obj=model_non_linear,  # Class that defines the problem
    )
    problem_non_linear.addOption("tol", 1e-7)
    problem_non_linear.addOption("print_level", 0)


# linear static optimization model
model_linear = ClassicalOptimizationLinearConstraints(
    setup["model"],
    setup["mot"],
    setup["filter_param"],
    muscle_physiology=setup["muscle_physiology"],
)

# optimization options
activation_initial_guess_linear = np.zeros([model_linear.n_muscles])
lb_linear, ub_linear = model_linear.get_bounds()

# problem
problem_linear = ipopt.problem(
    n=model_linear.n_muscles,  # Nb of variables
    lb=lb_linear,  # Variables lower bounds
    ub=ub_linear,  # Variables upper bounds
    m=model_linear.n_dof,  # Nb of constraints
    cl=np.zeros(model_linear.n_dof),  # Lower bound constraints
    cu=np.zeros(model_linear.n_dof),  # Upper bound constraints
    problem_obj=model_linear,  # Class that defines the problem
)
problem_linear.addOption("tol", 1e-7)
problem_linear.addOption("print_level", 0)

# optimization
activations_non_linear = []
activations_linear = []
for iframe in range(0, int(model_linear.n_frame)):
    print(f'frame: {iframe} | time: {model_linear.get_time(iframe)}')

    # Reference
    if calc_non_linear:
        model_non_linear.upd_model_kinematics(iframe)
        try:
            x_non_linear, info_non_linear = problem_non_linear.solve(activation_initial_guess_non_linear)
        except RuntimeError:
            print(f"Error while computing the frame #{iframe}")
        x0_non_linear_all.append(x_non_linear)
    else:
        x_non_linear = x0_non_linear_all[iframe]
    # the output is the initial guess for next frame
    activation_initial_guess_non_linear = x_non_linear
    activations_non_linear.append(x_non_linear)
    print(f'x_non_linear = {x_non_linear}')

    # New optim
    if hasattr(model_linear, 'previous_activation'):
        model_linear.set_previous_activation(activation_initial_guess_linear)
    model_linear.upd_model_kinematics(iframe)
    try:
        x_linear, info_linear = problem_linear.solve(activation_initial_guess_linear)
    except RuntimeError:
        print(f"Error while computing the frame #{iframe}")

    # the output is the initial guess for next frame
    activation_initial_guess_linear = x_linear
    activations_linear.append(x_linear)
    print(f'x = {x_linear}')
    print(f'Diff = {x_non_linear - x_linear}')
    print('')

# Save the non linear data
np.save('data_non_linear_all', x0_non_linear_all)

# Show the data
if show_results:
    import matplotlib.pyplot as plt
    from pyomeca import Analogs3d

    data = {
        "non_linear": Analogs3d(np.array(activations_non_linear)),
        "linear": Analogs3d(np.array(activations_linear))
    }
    ax = data["linear"].plot(fmt='')
    data["non_linear"].plot(fmt='.', ax=ax)
    (data["non_linear"] - data["linear"]).plot(fmt='')
    plt.show()
