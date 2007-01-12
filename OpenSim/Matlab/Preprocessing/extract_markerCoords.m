function markerCoords = extract_markerCoords(c, abbr)
% Purpose:  Extracts X, Y, Z coordinate trajectories from c, a structure  
%           obtained from read_c3dFile() or from extract_c3dSimData(),  
%           for the marker specified by abbr.
%
% Input:    c is a structure that contains the following data, 
%             in addition to other information:
%                   *.video       .rate,  .nframes
%                   *.markers()   .label, .data
%           abbr is the marker label, as specified in the C3D file.
%
% Output:   markerCoords returns a matrix(nVideoFrames, 3)
%               of X, Y, Z coordinates.
%           
% ASA, 10-05


% Get lists of labels corresponding to marker data in c.
nMarkers = length(c.markers);
for mkrIndex = 1:nMarkers
    mkrLabels{mkrIndex} = c.markers(mkrIndex).label;
end

% Extract X, Y, Z trajectories of the specified marker.
% NOTE:  The C3D Server returns cell arrays;
%        these arrays are converted to numeric arrays.
mkrIndex = strmatch(abbr, mkrLabels, 'exact');
if isempty(mkrIndex)
    markerCoords = [];
    warning('marker unable to be extracted from input structure.');
else
    for vframeNum = 1:c.video.nframes
        x(vframeNum) = c.markers(mkrIndex).data{vframeNum, 1};
        y(vframeNum) = c.markers(mkrIndex).data{vframeNum, 2};
        z(vframeNum) = c.markers(mkrIndex).data{vframeNum, 3};
    end
    markerCoords = [x' y' z'];
end
return;