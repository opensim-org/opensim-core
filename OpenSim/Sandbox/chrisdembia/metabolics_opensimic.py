import opensim

model = opensim.Model('subject01_adjusted.osim')
states = opensim.StatesTrajectory('subject01_fast_trial01_cmc.osimstates')

# InverseStudy drives time from states traj.
# Here, using convenience constructor.
study = opensim.InverseStudy(model, states)

met = opensim.Umberger2010MuscleMetabolics('Rajagopal2015_metabolics.xml')
met.setName('metabolics')

# Append metabolics model to model.
model.append(met)

reporter = opensim.FileReporter()
# Alternatively, `opensim.DataSink()`. I think the two concepts are actually
# the same.
# Can only set file prefix, since multiple files may actually be written if the
# inputs are of multiple types?
reporter.setFilePrefix('subject01_fast_trial01_metabolics')
# Print CSV files.
reporter.setFileFormat('csv')

# The Reporter must have a flexible number of inputs.
# With just one argument, the name of the first element of the input is the
# full path of the associated output ('metabolics/total_rate').
reporter.getMultiInput('input').append(met.getOutput('total_rate'))
# Optionally, a distinct name can be provided as well.
reporter.getMultiInput('input').append(met.getOutput('total_work_rate'),
                                       'met_total_rate')

study.append(reporter)

# The Study writes to the file incrementally so that all is not lost if
# we get an error midway.
# TODO It may be the case that the DataAdapters don't handle such a scenario
# yet. That's fine; as long as the writing happens when handling the
# exception.
study.run()

# Alternative: record the data in-memory, not writing to a file.
# ==============================================================

reporter = opensim.Reporter()
reporter.getMultiInput('input').append(met.getOutput('total_rate'))
study.append(reporter)
study.run()
table = reporter.getData()
time = table.getTimeColumn()
duration = time[-1] - time[0]
total = table.getColumn('metabolics/total_rate')
print('Average total rate: ', np.trapz(total, x=time) / duration)

# DataTable prints to the appropriate format based on file extension.
table.print('subject01_fast_trial01_metabolics.csv')
