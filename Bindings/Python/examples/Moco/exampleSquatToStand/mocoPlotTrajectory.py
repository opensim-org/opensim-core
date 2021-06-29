import opensim as osim
import matplotlib.pyplot as plt
import numpy as np

# Plot a MocoTrajectory. Optionally, specify a second trajectory and names
# for the trajectories.
def mocoPlotTrajectory(*args):
    args_list = list(args)

    if isinstance(args_list[0], str):
        trajA = osim.MocoTrajectory(args_list[0])
    
    if len(args) > 1 and isinstance(args_list[1], str):
        trajB = osim.MocoTrajectory(args_list[1])

    labelA = args_list[2] if len(args) == 4 else 'A'
    labelB = args_list[3] if len(args) == 4 else 'B'
    
    ## States.
    stateNames = list(trajA.getStateNames())
    numStates = len(stateNames)
    dim = np.sqrt(numStates)
    if dim == np.ceil(dim):
        numRows = int(dim)
        numCols = int(dim)
    else:
        numCols = int(np.min([numStates, 4]))
        numRows = int(np.floor(numStates / 4))
        if not numStates % 4 == 0:
            numRows += 1
    
    fig = plt.figure()
    for i in np.arange(numStates):
        ax = fig.add_subplot(numRows, numCols, int(i+1))
        ax.plot(trajA.getTimeMat(), 
             trajA.getStateMat(stateNames[i]), color='red',
             linewidth=3, label=labelA)

        if len(args) > 1:
            ax.plot(trajB.getTimeMat(), 
                 trajB.getStateMat(stateNames[i]), 
                 color='blue', linestyle='--',
                 linewidth=2.5, label=labelB)
        
        stateName = stateNames[i]
        ax.set_title(stateName[11:])
        ax.set_xlabel('time (s)')
        ax.set_xlim(0, 1)
        if 'value' in stateName:
            ax.set_ylabel('position (rad)')
        elif 'speed' in stateName:
            ax.set_ylabel('speed (rad/s)')
        elif 'activation' in stateName:
            ax.set_ylabel('activation (-)')
            ax.set_ylim(0, 1)
        
        if i == 0 and len(args) > 1:
            ax.legend(loc='best')

    plt.show()

    ## Controls.
    controlNames = list(trajA.getControlNames())
    numControls = len(controlNames)
    dim = np.sqrt(numControls)
    if dim == np.ceil(dim):
        numRows = int(dim)
        numCols = int(dim)
    else:
        numCols = int(np.min([numControls, 4]))
        numRows = int(np.floor(numControls / 4.0))
        if not numControls % 4 == 0:
            numRows += 1

    fig = plt.figure()
    for i in np.arange(numControls):
        ax = fig.add_subplot(numRows, numCols, int(i+1))
        yA = trajA.getControlMat(controlNames[i])
        ax.plot(trajA.getTimeMat(), yA, color='red', 
            linewidth=3, label=labelA)
        if len(args) > 1:
            yB = trajB.getControlMat(controlNames[i])
            ax.plot(trajB.getTimeMat(), yB, color='blue',
                linestyle='--', linewidth=2.5, 
                label=labelB)
       
        ax.set_title(controlNames[i])
        ax.set_xlabel('time (s)')
        ax.set_ylabel('value')
        ax.set_xlim(0, 1)
        if np.max(yA) <= 1 and np.min(yA) >= 0:
            fixYLim = True
            if len(args) > 1 and (np.max(yB) > 1 or 
                                  np.min(yB) < 0):
                fixYLim = False
            
            if fixYLim:
                ax.set_ylim(0, 1)

        if i == 0 and len(args) > 1:
            ax.legend(loc='best')

    fig.tight_layout()
    plt.show()
    plt.close()
