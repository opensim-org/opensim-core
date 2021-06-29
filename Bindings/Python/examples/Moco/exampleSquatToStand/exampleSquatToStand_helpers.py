import opensim as osim
import numpy as np
import matplotlib.pyplot as plt

def addCoordinateActuator(model, coordName, optForce):
    coordSet = model.updCoordinateSet()
    actu = osim.CoordinateActuator()
    actu.setName('tau_' + coordName)
    actu.setCoordinate(coordSet.get(coordName))
    actu.setOptimalForce(optForce)
    actu.setMinControl(-1)
    actu.setMaxControl(1)
    model.addComponent(actu)

def getTorqueDrivenModel():
    # Load the base model.
    model = osim.Model('squatToStand_3dof9musc.osim')

    # Remove the muscles in the model.
    model.updForceSet().clearAndDestroy()
    model.initSystem()

    # Add CoordinateActuators to the model degrees-of-freedom.
    addCoordinateActuator(model, 'hip_flexion_r', 150)
    addCoordinateActuator(model, 'knee_angle_r', 300)
    addCoordinateActuator(model, 'ankle_angle_r', 150)

    return model

def getMuscleDrivenModel():

    # Load the base model.
    model = osim.Model('squatToStand_3dof9musc.osim')
    model.finalizeConnections()

    # Replace the muscles in the model with muscles from DeGroote, Fregly,
    # et al. 2016, "Evaluation of Direct Collocation Optimal Control Problem
    # Formulations for Solving the Muscle Redundancy Problem". These muscles
    # have the same properties as the original muscles but their characteristic
    # curves are optimized for direct collocation (i.e. no discontinuities,
    # twice differentiable, etc).
    osim.DeGrooteFregly2016Muscle().replaceMuscles(model)

    # Make problems easier to solve by strengthening the model and widening the
    # active force-length curve.
    for m in np.arange(model.getMuscles().getSize()):
        musc = model.updMuscles().get(int(m))
        musc.setMinControl(0.0)
        musc.set_ignore_activation_dynamics(False)
        musc.set_ignore_tendon_compliance(False)
        musc.set_max_isometric_force(2.0 * musc.get_max_isometric_force())
        dgf = osim.DeGrooteFregly2016Muscle.safeDownCast(musc)
        dgf.set_active_force_width_scale(1.5)
        dgf.set_tendon_compliance_dynamics_mode('implicit')
        if str(musc.getName()) == 'soleus_r':
            # Soleus has a very long tendon, so modeling its tendon as rigid
            # causes the fiber to be unrealistically long and generate
            # excessive passive fiber force.
            dgf.set_ignore_passive_fiber_force(True)

    return model


def compareInverseSolutions(unassistedSolution, assistedSolution):
    unassistedSolution = unassistedSolution.getMocoSolution()
    assistedSolution = assistedSolution.getMocoSolution()
    stateNames = unassistedSolution.getStateNames()
    numStates = len(stateNames)

    fig = plt.figure()
    dim = 3
    iplot = 1
    for i in np.arange(numStates):
        if 'activation' in str(stateNames[i]):
            ax = fig.add_subplot(dim, dim, iplot);
            ax.plot(unassistedSolution.getTimeMat(),
                    unassistedSolution.getStateMat(stateNames[i]), 
                    color='red', linewidth=3, label='unassisted')
            ax.plot(assistedSolution.getTimeMat(),
                    assistedSolution.getStateMat(stateNames[i]), 
                    color='blue', linewidth=2.5, label='assisted')
            plotTitle = str(stateNames[i])
            plotTitle.replace('/forceset/', '')
            plotTitle.replace('/activation', '')
            ax.set_title(plotTitle)
            ax.set_xlabel('time (s)')
            ax.set_ylabel('activation (-)')
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            if iplot == 1:
               ax.legend()
            iplot += 1

    fig.tight_layout()
    plt.show()
    plt.close()
