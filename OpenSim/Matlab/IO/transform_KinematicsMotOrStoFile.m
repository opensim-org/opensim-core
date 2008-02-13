function transform_KinematicsMotOrStoFile( fileName, newFileName )
%
% Usage: compute_transformedKinematicsMotOrStoFile( fileName, newFileName )
%
% fileName is name of current .mot or .sto file to transform, newFileName
% is name of the file that will be output by this function after making the
% following changes to the file, which will transform generalized
% coordinates expressed in OpenSim's standard frames to the conventional
% frames used in biomechanics research papers.
%
% The following changes are made to columns with the specified headings:
% pelvis_list:   column = -1 * column
% pelvis_tilt:   column = -1 * ( column - 12 )
% hip_flexion_r: column = column + 12
% knee_angle_r:  column = -1 * column
% hip_flexion_l: column = column + 12
% knee_angle_l:  column = -1 * column
%
% If there is more than one column with any of these headings, all of those
% columns will be transformed.
%
% We assume the input file's angles are all in degrees.  The output file's
% angles will also be in degrees.
%

if nargin < 2
    error( 'Usage: compute_transformedKinematicsMotOrStoFile( fileName, newFileName )' );
end

% Read the input file.
q = read_motionFile( fileName );

% Get indices of all columns with any of the headings of interest.
pelvisListColumns = find( strcmp( 'pelvis_list', q.labels ) );
pelvisTiltColumns = find( strcmp( 'pelvis_tilt', q.labels ) );
hipFlexionRColumns = find( strcmp( 'hip_flexion_r', q.labels ) );
kneeAngleRColumns = find( strcmp( 'knee_angle_r', q.labels ) );
hipFlexionLColumns = find( strcmp( 'hip_flexion_l', q.labels ) );
kneeAngleLColumns = find( strcmp( 'knee_angle_l', q.labels ) );

% Combine hip flexion column indices into one hip index array, and combine
% knee angle column indices and pelvis list column indices into one index
% array, since all columns in each of these groups will undergo the same
% transformation.
hipIndices = union( hipFlexionRColumns, hipFlexionLColumns );
kneeIndices = union( kneeAngleRColumns, kneeAngleLColumns );
kneeAndPelvisListIndices = union( kneeIndices, pelvisListColumns );

% Transform these columns.
q.data( :, kneeAndPelvisListIndices ) = -1 * q.data( :, kneeAndPelvisListIndices );
q.data( :, pelvisTiltColumns ) = -1 * ( q.data( :, pelvisTiltColumns ) - 12 );
q.data( :, hipIndices ) = q.data( :, hipIndices ) + 12;

% Write the transformed file.
write_motionFile( q, newFileName );
