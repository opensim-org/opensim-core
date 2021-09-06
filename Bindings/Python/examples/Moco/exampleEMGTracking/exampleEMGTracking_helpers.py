import opensim as osim
import numpy as np
import matplotlib.pyplot as plt

# Add a CoordinateActuator to the provided model with a specified optimal
# force.
def addCoordinateActuator(model, coordinateName, optForce):

    coordSet = model.getCoordinateSet();

    actu = osim.CoordinateActuator();
    actu.setName('torque_' + coordinateName);
    actu.setCoordinate(coordSet.get(coordinateName));
    actu.setOptimalForce(optForce);
    actu.setMinControl(-1);
    actu.setMaxControl(1);

    # Add to ForceSet
    model.addForce(actu);


def getWalkingModel():

    # Load the 19 DOF, 18 muscle model from file.
    model = osim.Model('subject_walk_armless_18musc.osim');

    # Add actuators representing the pelvis residual actuators and lumbar
    # torques. These actuators are only as strong as they need to be for the
    # problem to converge.
    addCoordinateActuator(model, 'pelvis_tx', 60)
    addCoordinateActuator(model, 'pelvis_ty', 300)
    addCoordinateActuator(model, 'pelvis_tz', 35)
    addCoordinateActuator(model, 'pelvis_tilt', 60)
    addCoordinateActuator(model, 'pelvis_list', 35)
    addCoordinateActuator(model, 'pelvis_rotation', 25)
    addCoordinateActuator(model, 'lumbar_bending', 40)
    addCoordinateActuator(model, 'lumbar_extension', 40)
    addCoordinateActuator(model, 'lumbar_rotation', 25)

    # We need additional actuators for hip rotation and hip adduction since the
    # existing muscle act primarily in the sagittal plane.
    addCoordinateActuator(model, 'hip_rotation_r', 100)
    addCoordinateActuator(model, 'hip_rotation_l', 100)
    addCoordinateActuator(model, 'hip_adduction_r', 100)
    addCoordinateActuator(model, 'hip_adduction_l', 100)

    # Create a ModelProcessor to make additional modifications to the model.
    modelProcessor = osim.ModelProcessor(model)
    # Weld the subtalar and toe joints.
    jointsToWeld = osim.StdVectorString()
    jointsToWeld.append('subtalar_r')
    jointsToWeld.append('subtalar_l')
    jointsToWeld.append('mtp_r')
    jointsToWeld.append('mtp_l')
    modelProcessor.append(osim.ModOpReplaceJointsWithWelds(jointsToWeld))
    # Apply the ground reaction forces to the model.
    modelProcessor.append(osim.ModOpAddExternalLoads('external_loads.xml'))
    # Update the muscles: ignore tendon compliance and passive forces, replace
    # muscle types with the DeGrooteFregly2016Muscle type, and scale the width
    # of the active force length curve.
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    # Add a set a weak reserves to the sagittal-plane joints in the model.
    modelProcessor.append(osim.ModOpAddReserves(1.0))

    return modelProcessor


def compareSolutionToEMG(*args):

    # Retrieve the inputs.
    args_list = list(args)
    emgReference = args_list[0]
    effortSolution = osim.MocoTrajectory(args_list[1])
    if len(args) > 2:
        trackingSolution = osim.MocoTrajectory(args_list[2])

    # Create a time vector for the EMG data that is consistent with the problem
    # time range.
    time = effortSolution.getTimeMat()
    startIndex = emgReference.getNearestRowIndexForTime(time[0]) + 1
    endIndex = emgReference.getNearestRowIndexForTime(time[-1]) + 1
    emgTime = np.linspace(time[0], time[-1], endIndex - startIndex + 1)

    # Plot results from the muscles in the left leg.
    titles = ['soleus', 'gastroc.', 'tib. ant.', 'hamstrings', 
              'bi. fem. sh.', 'vastus', 'rec. fem.', 'gluteus', 'psoas']
    emgSignals = ['soleus',	'gastrocnemius', 'tibialis_anterior',
                  'hamstrings', 'biceps_femoris', 'vastus', 
                  'rectus_femoris', 'gluteus', 'none']
    muscles = ['soleus_l', 'gasmed_l', 'tibant_l', 'semimem_l', 'bfsh_l', 
               'vasint_l', 'recfem_l', 'glmax2_l', 'psoas_l']
    
    fig = plt.figure()
    for i in np.arange(len(titles)):
        ax = fig.add_subplot(3,3,i+1)

        # If it exists, plot the EMG signal for this muscle.
        if not emgSignals[i] == 'none':
            col = emgReference.getDependentColumn(emgSignals[i])
            emg = np.zeros_like(emgTime)
            for iemg in np.arange(len(emg)):
                emg[iemg] = col[int(iemg)]
            ax.fill_between(emgTime, emg, np.zeros_like(emg), alpha=0.5, 
                color='darkgray', zorder=0, label='EMG')

        # Plot the control from the muscle effort solution.
        control = effortSolution.getControlMat('/forceset/' + muscles[i])
        ax.plot(time, control, color='black', linewidth=2, label='effort')

        # If provided, plot the control from the tracking solution.
        if len(args) > 2:
            control = trackingSolution.getControlMat('/forceset/' + muscles[i]);
            ax.plot(time, control, color='red', linewidth=2, label='tracking')

        # Plot formatting.
        ax.set_title(titles[i])
        ax.set_xlim(0.83, 2.0)
        ax.set_ylim(0, 1)
        if i == 0:
            ax.legend()

        if i > 5:
            ax.set_xlabel('time (s)')
        else:
            ax.set_xticklabels([])
        
        if not i % 3:
            ax.set_ylabel('excitation')
        else:
            ax.set_yticklabels([])

    plt.show()
    plt.close()