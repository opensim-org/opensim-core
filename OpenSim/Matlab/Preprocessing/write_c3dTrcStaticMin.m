function mkrMatrix = write_c3dTrcStaticMin(mStatic, cStatic, fname)
% Purpose:  Writes marker data to a file (fname) for input into the
%           UWGait simulation workflow.
%
% Input:    mStatic is a structure containing the marker data from a 
%             static trial, formatted as follows, in addition to other 
%             information:
%               *.markers     .dataName(nVideoFrames, XYZ coordinates)
%               *.jntcenters  .dataName(nVideoFrames, XYZ coordinates)
%               *.time(nVideoFrames)
%           cStatic is a structure that contains the following data, 
%             in addition to other information:
%                   *.video   .rate,  .nframes
%           fname is the name of the file to be written.
%
% Output:   The file 'fname' is written to the current directory.
%           mkrMatrix returns a matrix containing the columns of 
%             marker data in units of m (consistent with model), 
%             and in the column order as written to the *.trc file; 
%             these data must be written to the motion file in the 
%             same column order.
%
%           NOTE:  I tried re-writing this function so that the columns
%                  of the *.trc file would line up when opened in Excel;
%                  however, Darryl's executables would not read a 
%                  tab-delimited file. 
%
% ASA, 10-05


% Generate mapping between marker names in SIMM joint file (column 1), 
% and data in mCS (column 2).
label{1}  = {'midhip',      'midHip'};          % Joint Centers
label{2}  = {'hip_r',       'RFEP'};            % RFEP, LFEP Vicon PIG
label{3}  = {'hip_l',       'LFEP'};
label{4}  = {'knee_r',      'RFEO'};            % RFEO, LFEO Vicon PIG
label{5}  = {'knee_l',      'LFEO'}; 
label{6}  = {'ankle_r',     'RTIO'};
label{7}  = {'ankle_l',     'LTIO'};
jntcenterIndex = length(label);

label{8} = {'midasis',      'pelvisTranslations'};
pelvTransIndex = length(label);

label{9}  = {'asis_r',          'RASI'};        % Anatomical Markers
label{10} = {'asis_l',          'LASI'}; 
label{11} = {'psis_r',          'RPSI'}; 
label{12} = {'psis_l',          'LPSI'}; 
label{13} = {'epicondLat_r',    'RKNE'};
label{14} = {'epicondLat_l',    'LKNE'};
label{15} = {'malleolusLat_r',  'RANK'};
label{16} = {'malleolusLat_l',  'LANK'};
label{17} = {'heel_r',          'RHEE'};
label{18} = {'heel_l',          'LHEE'};
anatomicalIndex = length(label);

label{19} = {'neck',            'N'};           % Tracking Markers
label{20} = {'subclavicle_r',   'RAC'};
label{21} = {'subclavicle_l',   'LAC'};
label{22} = {'sternalnotch',    'STRN'};
label{23} = {'thighwand_r',     'RTHI'};
label{24} = {'thighwand_l',     'LTHI'};
label{25} = {'shankwand_r',     'RTIB'};
label{26} = {'shankwand_l',     'LTIB'};
label{27} = {'toe_r',           'RTOE'};       
label{28} = {'toe_l',           'LTOE'};
trackingIndex = length(label);
nMkrs = trackingIndex;
 
   
% Generate matrix for writing marker data of interest.
nRows = cStatic.video.nframes;            
nCols = nMkrs*3;
mkrMatrix = zeros(nRows, nCols);
for mkrNum = 1:jntcenterIndex
    colIndex = mkrNum*3 - 2;
    eval(['mkrMatrix(:, colIndex:colIndex+2) = mStatic.jntcenters.', ...
                                                label{mkrNum}{2}, ';']);
end
for mkrNum = (jntcenterIndex + 1):pelvTransIndex
    colIndex = mkrNum*3 - 2;
    mkrMatrix(:, colIndex:colIndex+2) = mStatic.markers.pelvisTranslations;
end
for mkrNum = (pelvTransIndex + 1):nMkrs
    colIndex = mkrNum*3 - 2;
    eval(['mkrMatrix(:, colIndex:colIndex+2) = mStatic.markers.', ...
                                                label{mkrNum}{2}, ';']);
end

% Open file for writing.
fid = fopen(fname, 'w');
if fid == -1
    error(['unable to open ', fname])
end

% Write header line 1.
fprintf(fid, ['PathFileType   4   (X/Y/Z)\n']);

% Write header line 2.  
header2{1} = 'DataRate';
header2{2} = 'CameraRate';
header2{3} = 'NumFrames';
header2{4} = 'NumMarkers';
header2{5} = 'Units';
header2{6} = 'OrigDataRate';
header2{7} = 'OrigDataStartFrame';
header2{8} = 'OrigNumFrames';
for i = 1:length(header2)
	fprintf(fid, '%s  ', header2{i});
end
fprintf(fid, '\n');

% Write header line 3.
% NOTE: It is my understanding that Darryl's executables require that the 
%       units of the marker data in the *.trc file be mm (ASA, 10-05).
f = cStatic.video.rate;                    % sampling frequency of data
units = 'mm';                              % units for output file    
fprintf(fid, ['%7.1f    %7.1f    %7d    %7d    %7s    %7.1f    %7d    %7d\n'], ...
              f, f, nRows, nMkrs, units, f, 1, nRows);

% Write header line 4.
header4{1} = 'Frame#';
header4{2} = 'Time';
fprintf(fid, ['%s      %s      '], header4{1}, header4{2});
for i = 1:nMkrs
    fprintf(fid, '%20s            ', label{i}{1});
end
fprintf(fid, '\n');

% Write header line 5.
header5{1} = ' ';
header5{2} = ' ';
fprintf(fid, ['%s      %s      '], header5{1}, header5{2});
for mkrNum = 1:nMkrs
    colIndex = mkrNum*3 - 2;
    xyzLabel{colIndex}   = strcat('X', num2str(mkrNum));
    xyzLabel{colIndex+1} = strcat('Y', num2str(mkrNum));
    xyzLabel{colIndex+2} = strcat('Z', num2str(mkrNum));
end
for i = 1:length(xyzLabel)
    fprintf(fid, '%s          ', xyzLabel{i});
end
fprintf(fid, '\n');
fprintf(fid, '\n');

% Write time values and mkrMatrix.
for i = 1:nRows
    fprintf(fid, '\t%d', i);
    fprintf(fid, '\t%.5f', mStatic.time(i));
    for j = 1:nCols
        fprintf(fid, '\t%.3f', 1000.*mkrMatrix(i, j));
    end
    fprintf(fid, '\n');
end
fclose(fid);
return;



% NOTE: I deleted these markers from the list on 11-18-05 because
%       many of the static trials from Gillette did not have these 
%       markers labelled.
% label{15} = {'epicondMed_r',    'RMFC'};
% label{16} = {'epicondMed_l',    'LMFC'};
% label{19} = {'malleolusMedv_r', 'RMMv'};
% label{20} = {'malleolusMedv_l', 'LMMv'};
% label{29} = {'upperleg1_r',     'RUL1'};
% label{30} = {'upperleg1_l',     'LUL1'};
% label{31} = {'upperleg2_r',     'RUL2'};
% label{32} = {'upperleg2_l',     'LUL2'};
% label{35} = {'lowerleg1_r',     'RLL1'}; 
% label{36} = {'lowerleg1_l',     'LLL1'};

