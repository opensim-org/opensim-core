import opensim
import numpy as np

model = opensim.Model('subject01_adjusted.osim')
states = opensim.StatesTrajectory('subject01_fast_trial01_cmc.osimstates')
# A 1-D numpy array of times for the states.
time = states.getTimes()
duration = time[-1] - time[0]

# Generic metabolics model.
met = opensim.Umberger2010MuscleMetabolics('Rajagopal2015_metabolics.xml')
met.setName('metabolics')

# Append metabolics model to model.
model.append(met)

# Invoke connecting.
model.initSystem()

# Allocate the quantities I want to compute.
power = np.empty(states.size())
activ_maint = np.empty(states.size())
short_length = np.empty(states.size())
total = np.empty(states.size())

# Evaluate metabolics.
# --------------------
i = 0
for state in states:
    # Summed over all muscles.
    power[i] = model.get('metabolics').getOutput(state, 'total_work_rate')
    # Potential shorthand for getting components: get().
    activ_maint[i] = model.get('metabolics').getOutput(state, 'total_activation_maintenance_rate')
    # Our initial `met` is still a reference to the one in the model.
    short_length[i] = met.getOutput(state, 'total_shortening_lengthening_rate')
    # Nested path to the output (questionably useful).
    total[i] = model.getOutput(state, 'metabolics/total_rate')

    i += 1

print('Average total rate: ', np.trapz(total, x=time) / duration)
print('Average work rate: ', np.trapz(power, x=time) / duration)

