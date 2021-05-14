function exampleEMGDrivenWalking
close all;

import org.opensim.modeling.*;

model = Model('subject_walk_armless_80musc.osim');
addCoordinateActuator(model, 'pelvis_tx', 1000)
addCoordinateActuator(model, 'pelvis_ty', 1000)
addCoordinateActuator(model, 'pelvis_tz', 1000)
addCoordinateActuator(model, 'pelvis_tilt', 1000)
addCoordinateActuator(model, 'pelvis_list', 1000)
addCoordinateActuator(model, 'pelvis_rotation', 1000)
addCoordinateActuator(model, 'lumbar_bending', 100)
addCoordinateActuator(model, 'lumbar_extension', 100)
addCoordinateActuator(model, 'lumbar_rotation', 100)

inverse = MocoInverse();
modelProcessor = ModelProcessor(model);
modelProcessor.append(ModOpAddExternalLoads('grf_walk.xml'));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
% modelProcessor.append(ModOpScaleMaxIsometricForce(2.0));
modelProcessor.append(ModOpAddReserves(1.0));

inverse.setModel(modelProcessor);

coordinates = TableProcessor('coordinates.mot');
coordinates.append(TabOpLowPassFilter(6));
coordinates.append(TabOpUseAbsoluteStateNames());

inverse.setKinematics(coordinates);
inverse.set_initial_time(0.83);
inverse.set_final_time(2.0);
inverse.set_mesh_interval(0.035);
inverse.set_kinematics_allow_extra_columns(true);

study = inverse.initialize();
problem = study.updProblem();

% problem.updGoal('excitation_effort').setWeight(1);

% Add electromyography tracking.
emgTracking = MocoControlTrackingGoal('emg_tracking');
emgTracking.setWeight(50.0);
% Each column in electromyography.sto is normalized so the maximum value in
% each column is 1.0.
controlsRef = TimeSeriesTable('emg.sto');

% Scale down the tracked muscle activity based on peak levels from
% "Gait Analysis: Normal and Pathological Function" by
% Perry and Burnfield, 2010 (digitized by Carmichael Ong).
soleus = controlsRef.updDependentColumn('soleus');
gasmed = controlsRef.updDependentColumn('gastrocnemius');
tibant = controlsRef.updDependentColumn('tibialis_anterior');
hamstr = controlsRef.updDependentColumn('hamstrings');
bifem  = controlsRef.updDependentColumn('biceps_femoris');
vaslat = controlsRef.updDependentColumn('vastus_lateralis');
vasmed = controlsRef.updDependentColumn('vastus_medialis');
recfem = controlsRef.updDependentColumn('rectus_femoris');
glmed  = controlsRef.updDependentColumn('gluteus_medius');

for t = 0:controlsRef.getNumRows() - 1
    soleus.set(t, 0.8 * soleus.get(t));
    gasmed.set(t, 0.6 * gasmed.get(t));
    tibant.set(t, 0.4 * tibant.get(t));
    hamstr.set(t, 0.4 * hamstr.get(t));
    bifem.set(t, 0.25 * bifem.get(t));
    vaslat.set(t, 0.2 * vaslat.get(t));
    vasmed.set(t, 0.1 * vasmed.get(t));
    recfem.set(t, 0.3 * recfem.get(t));
    glmed.set(t, 0.9 * glmed.get(t));
end
emgTracking.setReference(TableProcessor(controlsRef));
% Associate actuators in the model with columns in electromyography.sto.
emgTracking.setReferenceLabel('/forceset/soleus_l', 'soleus')
emgTracking.setReferenceLabel('/forceset/gasmed_l', 'gastrocnemius')
emgTracking.setReferenceLabel('/forceset/tibant_l', 'tibialis_anterior')
emgTracking.setReferenceLabel('/forceset/semimem_l', 'hamstrings')
emgTracking.setReferenceLabel('/forceset/bfsh_l', 'biceps_femoris')
emgTracking.setReferenceLabel('/forceset/vaslat_l', 'vastus_lateralis')
emgTracking.setReferenceLabel('/forceset/vasmed_l', 'vastus_medialis')
emgTracking.setReferenceLabel('/forceset/recfem_l', 'rectus_femoris')
emgTracking.setReferenceLabel('/forceset/glmed1_l', 'gluteus_medius')
problem.addGoal(emgTracking)

% Solve the problem and write the solution to a Storage file.
if ~exist('solution.sto', 'file')
    solution = study.solve();
    solution.write('solution.sto');
end



compareSolutionToEMG(controlsRef);


end

function addCoordinateActuator(model, coordName, optForce)

import org.opensim.modeling.*;

coordSet = model.updCoordinateSet();

actu = CoordinateActuator();
actu.setName(['tau_' coordName]);
actu.setCoordinate(coordSet.get(coordName));
actu.setOptimalForce(optForce);
actu.setMinControl(-1);
actu.setMaxControl(1);

% Add to ForceSet
model.addForce(actu);

end

function compareSolutionToEMG(controlsRef)

import org.opensim.modeling.*;

solution = MocoTrajectory('solution.sto');

time = solution.getTimeMat();
startIndex = controlsRef.getNearestRowIndexForTime(time(1)) + 1;
endIndex = controlsRef.getNearestRowIndexForTime(time(end)) + 1;
refTime = linspace(time(1), time(end), endIndex - startIndex + 1);

titles = {'soleus', 'gastroc.', 'tib. ant.', 'hamstrings', 'bi. fem.', ...
          'vas. lat.', 'vas. med.', 'rec. fem.', 'glut. med.'};
emgSignals = {'soleus',	'gastrocnemius', 'tibialis_anterior', 'hamstrings', ...	
              'biceps_femoris',	'vastus_lateralis',	'vastus_medialis', ...
              'rectus_femoris', 'gluteus_medius'};
muscles = {'soleus_l', 'gasmed_l', 'tibant_l', 'semimem_l', 'bfsh_l', ...
           'vaslat_l', 'vasmed_l', 'recfem_l', 'glmed1_l'};
       
for i = 1:length(titles)
    subplot(3,3,i)
    col = controlsRef.getDependentColumn(emgSignals{i}).getAsMat();
    emg = col(startIndex:endIndex)';
    patch([refTime fliplr(refTime)], [emg fliplr(zeros(size(emg)))], ...
        [0.7 0.7 0.7])
    alpha(0.5);
    hold on
    control = solution.getControlMat(['/forceset/' muscles{i}]);
    plot(time, control, '-k', 'linewidth', 2)
    title(titles{i})
    xlim([0.83 2.0])
    ylim([0 1])
    if i == 1
            legend('EMG', 'excitation', 'location', 'best')
    end
    if i > 6
        xlabel('time (s)')
    else
        xticklabels([])
    end
    if ~mod(i-1, 3)
        ylabel('excitation')
    else
        yticklabels([])
    end
end


end