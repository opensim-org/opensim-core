function compute_netMuscleMoments( resultsMuscleAnalysisDir, name )

% These are the column labels for all generalized coordinates.
gencoordColumnLabels = { 'pelvis_tx' 'pelvis_ty' 'pelvis_tz' ...
    'pelvis_list' 'pelvis_rotation' 'pelvis_tilt' 'hip_flexion_r' ...
    'hip_adduction_r' 'hip_rotation_r' 'hip_flexion_l' ...
    'hip_adduction_l' 'hip_rotation_l' 'knee_angle_r' 'knee_angle_l' ...
    'ankle_angle_r' 'subtalar_angle_r' 'ankle_angle_l' ...
    'subtalar_angle_l' 'mtp_angle_r' 'mtp_angle_l' 'lumbar_extension' ...
    'lumbar_bending' 'lumbar_rotation' };

% Read first MuscleAnalysis .sto file.
label = gencoordColumnLabels{1};
fname = [ name '_MuscleAnalysis_Moment_' label '.sto' ];
filename = fullfile( resultsMuscleAnalysisDir, fname );
q = read_motionFile( filename );

% The data tends to be so finely sampled that MATLAB can't even distinguish
% between the time values.  Downsample the data to avoid interpolation
% problems later on.

% Create output motion file structure.
outQ.labels = [ 'time' gencoordColumnLabels ];
% We assume every MuscleAnalysis .sto file has the same time column.
outQ.nr = size( q.data, 1 );
outQ.nc = length( outQ.labels );
outQ.data = zeros( outQ.nr, outQ.nc );

% We assume the first column of q.data is the time column.
outQ.data( :, 1 ) = q.data( :, 1 );
outQ.data( :, 2 ) = sum( q.data( :, 2 : end ), 2 );

% Compute the net moments for all other coordinates, and record them in the
% output motion file structure.
for i = 2 : length( gencoordColumnLabels )
    label = gencoordColumnLabels{i};
    fname = [ name '_MuscleAnalysis_Moment_' label '.sto' ];
    filename = fullfile( resultsMuscleAnalysisDir, fname );
    q = read_motionFile( filename );
    outQ.data( :, i + 1 ) = sum( q.data( :, 2 : end ), 2 );
end

% Write the output motion file.
ofname = [ name '_NET_Muscle_Moments.sto' ];
outputFileName = fullfile( resultsMuscleAnalysisDir, ofname );
write_motionFile( outQ, outputFileName );
