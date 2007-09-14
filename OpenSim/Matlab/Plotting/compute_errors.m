function compute_errors(subject, trial)

ikKinematicsFileName = sprintf('%s_%s_ik.mot', subject, trial); % Kinematics after IK
cmcKinematicsFileName = sprintf('ResultsCMC/%s_%s_Kinematics_q.sto', subject, trial); % Kinematics after CMC

pelvisTranslationLabels = { 'pelvis_tx' 'pelvis_ty' 'pelvis_tz' };
pelvisOrientationLabels = { 'pelvis_list' 'pelvis_rotation' 'pelvis_tilt' };
hipLabels = { 'hip_flexion_r' 'hip_adduction_r' 'hip_rotation_r' ...
              'hip_flexion_l' 'hip_adduction_l' 'hip_rotation_l' };
kneeLabels = { 'knee_angle_r' 'knee_angle_l' };
ankleLabels = { 'ankle_angle_r' 'subtalar_angle_r' 'ankle_angle_l' 'subtalar_angle_l' };
footLabels = { 'mtp_angle_r' 'mtp_angle_l' };
backLabels = { 'lumbar_extension' 'lumbar_bending' 'lumbar_rotation' };
labelsOfCoordinatesToCompare = union( pelvisTranslationLabels, pelvisOrientationLabels );
labelsOfCoordinatesToCompare = union( labelsOfCoordinatesToCompare, hipLabels );
labelsOfCoordinatesToCompare = union( labelsOfCoordinatesToCompare, kneeLabels );
labelsOfCoordinatesToCompare = union( labelsOfCoordinatesToCompare, ankleLabels );
labelsOfCoordinatesToCompare = union( labelsOfCoordinatesToCompare, footLabels );
labelsOfCoordinatesToCompare = union( labelsOfCoordinatesToCompare, backLabels );
numCoordinatesToCompare = length( labelsOfCoordinatesToCompare );
             
if exist(ikKinematicsFileName,'file') && exist(cmcKinematicsFileName,'file')
	disp('IK vs. CMC: Peak and RMS tracking errors.');
	q1 = read_motionFile(ikKinematicsFileName);
	q2 = read_motionFile(cmcKinematicsFileName);
	tmin = max(q1.data(1,1),q2.data(1,1));
	tmax = min(q1.data(end,1),q2.data(end,1));
	T = tmin:0.0001:tmax;

    numRowsInQ1 = length( q1.data(:,1) );
    numRowsInQ2 = length( q2.data(:,1) );
    ikGeneralizedCoordinatesOriginal = zeros( numRowsInQ1, numCoordinatesToCompare );
    cmcGeneralizedCoordinatesOriginal = zeros( numRowsInQ2, numCoordinatesToCompare );
    for i = 1:numCoordinatesToCompare
        coordinateToCompare = labelsOfCoordinatesToCompare(i);

        indicesOfCoordinateToCompareInQ1Labels = find( strcmp( q1.labels, coordinateToCompare ) );
        indexOfCoordinateToCompareInQ1 = indicesOfCoordinateToCompareInQ1Labels(1);
        ikGeneralizedCoordinatesOriginal(:,i) = q1.data(:, indexOfCoordinateToCompareInQ1);

        indicesOfCoordinateToCompareInQ2Labels = find( strcmp( q2.labels, coordinateToCompare ) );
        indexOfCoordinateToCompareInQ2 = indicesOfCoordinateToCompareInQ2Labels(1);
        cmcGeneralizedCoordinatesOriginal(:,i) = q2.data(:, indexOfCoordinateToCompareInQ2);
    end

    ikGeneralizedCoordinates = interp1(q1.data(:,1),ikGeneralizedCoordinatesOriginal,T);
	cmcGeneralizedCoordinates = interp1(q2.data(:,1),cmcGeneralizedCoordinatesOriginal,T);
	errors = abs(ikGeneralizedCoordinates - cmcGeneralizedCoordinates);
    pelvisTxColumnNumber = find( strcmp( labelsOfCoordinatesToCompare, 'pelvis_tx' ) );
    pelvisTyColumnNumber = find( strcmp( labelsOfCoordinatesToCompare, 'pelvis_ty' ) );
    pelvisTzColumnNumber = find( strcmp( labelsOfCoordinatesToCompare, 'pelvis_tz' ) );
    translationalCoordinateColumnNumbers = [ pelvisTxColumnNumber pelvisTyColumnNumber pelvisTzColumnNumber ];
    allColumnNumbers = 1:numCoordinatesToCompare;
    rotationalCoordinateColumnNumbers = setdiff( allColumnNumbers, translationalCoordinateColumnNumbers );
    disp(sprintf('Max translation error (m) = %s', max_column_info(labelsOfCoordinatesToCompare, errors, translationalCoordinateColumnNumbers)));
    disp(sprintf('Max RMS translation error (m) = %s', rms_column_info(labelsOfCoordinatesToCompare, errors, translationalCoordinateColumnNumbers)));
    disp(sprintf('Mean RMS translation error (m) = %s', mean_rms_column_info(errors, translationalCoordinateColumnNumbers)));
    disp(sprintf('Max angle error (deg) = %s', max_column_info(labelsOfCoordinatesToCompare, errors, rotationalCoordinateColumnNumbers)));
    disp(sprintf('Max RMS angle error (deg) = %s', rms_column_info(labelsOfCoordinatesToCompare, errors, rotationalCoordinateColumnNumbers)));
    disp(sprintf('Mean RMS angle error (deg) = %s', mean_rms_column_info(errors, rotationalCoordinateColumnNumbers)));
end
