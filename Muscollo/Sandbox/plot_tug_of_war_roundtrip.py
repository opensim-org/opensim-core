import pylab as pl
import pandas as pd

unfiltered = pd.read_csv('DEBUG_desiredMoments_unfiltered.sto', index_col=0,
        skiprows=6, delimiter='\t')
filtered = pd.read_csv('DEBUG_desiredMoments.csv', index_col=0, header=None)

actual = pd.read_csv('DEBUG_testTugOfWar_INDYGO_actualInvDyn.csv', index_col=0, skiprows=3)

fig = pl.figure(figsize=(6, 5))
num_columns = len(filtered.columns)
for i in range(num_columns):

    ax = fig.add_subplot(num_columns, 1, i + 1)
    ax.plot(actual.index, actual[actual.columns[i]], label='original')
    ax.plot(unfiltered.index, unfiltered[unfiltered.columns[i]], label='ID, unfiltered')
    ax.plot(filtered.index, filtered[filtered.columns[i]], label='ID, filtered')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('net force (N)')
    pl.legend()

pl.savefig('tug_of_war_inverse_dynamics.png', dpi=300)



traj = pd.read_csv('testTugOfWarDeGrooteFregly2016_INDYGO_trajectory.csv',
                   index_col=0, skiprows=2)
sol = pd.read_csv('INDYGO_OCP_solution.csv', index_col=0,
        skiprows=2)
fig = pl.figure(figsize=(6, 8))
ax = fig.add_subplot(4, 1, 1)
ax.plot(traj.index, traj['position'])
ax.set_title('position (m)')
ax.set_xticklabels([])
ax = fig.add_subplot(4, 1, 2)
ax.plot(traj.index, traj['speed'])
ax.set_title('speed (m/s)')
ax.set_xticklabels([])
ax = fig.add_subplot(4, 1, 3)
ax.plot(traj.index, traj['activation_l'], label='original')
ax.plot(sol.index, sol['/tug_of_war/left_activation'], label='solution')
ax.legend(loc='upper left')
ax.set_title('left activation')
ax.set_xticklabels([])
ax = fig.add_subplot(4, 1, 4)
ax.plot(traj.index, traj['activation_r'], label='original')
ax.plot(sol.index, sol['/tug_of_war/right_activation'], label='INDYGO')
ax.set_xlabel('time (s)')
ax.set_title('right activation')
fig.savefig('tug_of_war_activation.png', dpi=300)

