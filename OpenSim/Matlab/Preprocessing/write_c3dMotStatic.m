function [] = write_c3dMotStatic(mStatic, mkrMatrix, cStatic, fname)
% Purpose:  Writes markers, joint centers, and joint angles to a 
%           motion file (fname) for input into the UWGait simulation 
%           workflow.
%
% Input:   mStatic is a structure containing the following data, 
%            formatted as follows:
%               *.markers    .pelvisTranslations(nVideoFrames, XYZ coordinates)
%               *.jntcenters .dataName(nVideoFrames, XYZ coordinates)
%               *.jntangles  .dataName(nVideoFrames)
%           mkrMatrix is a matrix containing the columns of marker data
%             as written to the *.trc file in units of meters; 
%             these data must be written to the motion file in the same 
%             column order.
%           cStatic is a structure that contains the following data, 
%             in addition to other information:
%                   *.video   .rate,  .nframes
%           fname is the name of the file to be written.
%
% Output:   The file 'fname' is written to the current directory.
% ASA, 10-05


% Generate column labels for time and pelvis translations.
label{1}  = 'time';
label{2}  = 'pelvis_tx';
label{3}  = 'pelvis_ty';
label{4}  = 'pelvis_tz';
translationsIndex = length(label);

% Generate column labels for joint angles.
label{5}  = 'pelvis_tilt';
label{6}  = 'pelvis_list';
label{7}  = 'pelvis_rotation';
label{8}  = 'hip_flexion_r';
label{9}  = 'hip_adduction_r';
label{10} = 'hip_rotation_r';
label{11} = 'knee_angle_r';
label{12} = 'ankle_angle_r';
label{13} = 'subtalar_angle_r';
label{14} = 'mtp_angle_r';
label{15} = 'hip_flexion_l';
label{16} = 'hip_adduction_l';
label{17} = 'hip_rotation_l';
label{18} = 'knee_angle_l';
label{19} = 'ankle_angle_l';
label{20} = 'subtalar_angle_l';
label{21} = 'mtp_angle_l';
label{22} = 'lumbar_extension';
label{23} = 'lumbar_bending';
label{24} = 'lumbar_rotation';
jntangleIndex = length(label);

% Generate column labels for markers.  
% NOTE: each marker in mkrMatrix has xyz components.
[nMkrRows, nMkrCols] = size(mkrMatrix);
xCol = (1:3:nMkrCols) + jntangleIndex;
yCol = (2:3:nMkrCols) + jntangleIndex;
zCol = (3:3:nMkrCols) + jntangleIndex;
for colIndex = xCol
    label{colIndex} = 'ground_marker_px';
end
for colIndex = yCol
    label{colIndex} = 'ground_marker_py';
end
for colIndex = zCol
    label{colIndex} = 'ground_marker_pz';
end
mkrIndex = length(label);
    
% Initialize 'motion file data matrix' for writing data of interest.
nRows = nMkrRows;
nCols = length(label);
motData = zeros(nRows, nCols);

% Write time array to data matrix.
motData(:, 1) = mStatic.time;              

% Write pelvis translations to data matrix.
motData(:, 2:translationsIndex) = mStatic.markers.pelvisTranslations;

% Write joint angle data to data matrix.
for colNum = (translationsIndex + 1):jntangleIndex
    eval(['motData(:, colNum) = mStatic.jntangles.', label{colNum}, ';']);
end

% Write marker data to data matrix.
colNum = (jntangleIndex + 1):mkrIndex;
motData(:, colNum) = mkrMatrix;

% Open file for writing.
fid = fopen(fname, 'w');
if fid == -1
    error(['unable to open ', fname])
end

% Write header.
fprintf(fid, 'name %s\n', fname);
fprintf(fid, 'datacolumns %d\n', nCols);
fprintf(fid, 'datarows %d\n', nRows);
fprintf(fid, 'range %d %d\n', mStatic.time(1), mStatic.time(nRows));
fprintf(fid, 'endheader\n\n');

% Write column labels.
for i = 1:length(label)
	fprintf(fid, '%20s\t', label{i});
end

% Write data.
for i = 1:nRows
    fprintf(fid, '\n'); 
	for j = 1:nCols
        fprintf(fid, '%20.8f\t', motData(i, j));
    end
end

fclose(fid);
return;

