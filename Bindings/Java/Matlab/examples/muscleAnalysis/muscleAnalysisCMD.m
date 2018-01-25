function muscleAnalysisCMD(modelFile, statesStorageFile, musclesList, outputsList)

% % quick testing
% modelFile = 'subject01_simbody_adjusted.osim';
% statesStorageFile = 'subject01_walk1_states_truncated.sto';
% musclesList = {'tfl_r', 'sar_r', 'vas_int_r'};
% outputsList = {'activation', 'normalized_fiber_length'};
import org.opensim.modeling.*

% Load model and storage into states trajectory
model = Model(modelFile);
statesStorage = Storage(statesStorageFile);
statesTrajectory = StatesTrajectory.createFromStatesStorage(model, statesStorage);

numStates = statesTrajectory.getSize();
numMuscles = numel(musclesList);
numOutputs = numel(outputsList);

reporter = TableReporter();
% Loop over each muscle
for indMuscle = 1:numMuscles       
    % Loop over every output
    for indOutput = 1:numOutputs
        muscleName = musclesList(indMuscle);
        outputName = outputsList(indOutput);
        % no form of "getOutputValue<T>" in scripting, use a reporter
        % instead
        %outputsTemp(indOutput, model.getComponent(muscleName).getOutput(outputName));
        thisMusc = model.getComponent(muscleName);
        reporter.addToReport(thisMusc.getOutput(outputName));

    end
end

model.addComponent(reporter);
state = model.initSystem();

for indState = 0:numStates-1
    trajState = statesTrajectory.get(indState);
    
    % workaround to stateTrajectory bug
    state.setY(trajState.getY());
    state.setTime(trajState.getTime());
    
    model.realizeReport(state);
end

table = reporter.getTable();
sto = STOFileAdapter();
sto.write(table, 'muscleAnalysis.sto');