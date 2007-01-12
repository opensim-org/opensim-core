function markers = transform_markersToModelCS(c, tInfo)
% Purpose:  Performs the following computations:
%           (1) transforms marker data in c, read from the C3D file of
%               a Gillette subject, to a coordinate system and units 
%               consistent with Darryl's model.  
%           (2) computes and returns the translations of the pelvis.
%               (i.e., midpoint between ASISs).
%
% Input:    c is a structure from read_c3dFile() or extract_c3dSimData()
%             that contains the following data, in addition to other 
%             information:
%                   *.trial       .num,   .type 
%                   *.video       .rate,  .nframes, .units
%                   *.markers()   .label, .data
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.FP     - FP #s in the order hit (gait trial only) 
%
% Output:   markers returns a structure containing the transformed 
%             marker trajectories, with the following format:
%               *.dataName(nVideoFrames), where dataName is derived from
%                           c.markers.label, for all markers stored in c
%               *.pelvisTranslations(nVideoFrames)
%
% Notes:    Gillette Lab CS (if walking in +X) has X anterior, Y left, Z up
%           Darryl's Model CS has X anterior, Y up, Z right
%               
% ASA, 10-05


% Get walking/standing direction.
if strcmpi(c.trial.type, 'static')
    walkDir = 'posX';                   % stood in +x direction (static)
    % NOTE: I'm assuming that subjects faced the +x direction during the
    %       static trial.  This could be confirmed by checking the 
    %       relative locations of markers.
else
    if tInfo.FP{1} < tInfo.FP{2}        % walked in +x direction
         walkDir = 'posX';
    else
         walkDir = 'negX';              % walked in -x direction
    end
end

% For each marker stored in c.markers...
for markerNum = 1:length(c.markers)
    
    % Extract X, Y, Z trajectories of marker (in lab CS).
    % NOTE:  The C3D Server returns cell arrays;
    %        these arrays are converted to numeric arrays.
    for vframeNum = 1:c.video.nframes
        x(vframeNum) = c.markers(markerNum).data{vframeNum, 1};
        y(vframeNum) = c.markers(markerNum).data{vframeNum, 2};
        z(vframeNum) = c.markers(markerNum).data{vframeNum, 3};
    end
        
    % Convert coordinates to be consistent with model CS.
    if strcmpi(walkDir, 'posX')
        data_mCS = [x'  z'  -1*y'];
    elseif strcmpi(walkDir, 'negX')
        data_mCS = [-1*x'  z'  y'];
    end
    
    % Convert units to be consistent with model (meters).
    if strcmpi(c.video.units, 'mm')
        data_mCS = data_mCS/1000.0;
    end
    
    % Store data in output structure.
    dataName = c.markers(markerNum).label;
    eval(['markers.', dataName, '= data_mCS;']);    
end

% Compute pelvis translations from midpoint of R and L ASIS markers.
markers.pelvisTranslations = (markers.RASI + markers.LASI)/2.0;
return;
