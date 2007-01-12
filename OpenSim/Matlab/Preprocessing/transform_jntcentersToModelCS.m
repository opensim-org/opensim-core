function jntcenters = transform_jntcentersToModelCS(c, tInfo)
% Purpose:  Performs the following computations:
%           (1) transforms joint centers in c, read from the C3D file of
%               a Gillette subject, to a coordinate system and units 
%               consistent with Darryl's model.  
%           (2) computes and returns a mid-hip marker.
%
% Input:    c is a structure from read_c3dFile() or extract_c3dSimData()
%             that contains the following data, in addition to other 
%             information:
%                   *.trial       .num,   .type 
%                   *.video       .rate,  .nframes, .units
%                   .jntcenters    .L(),   .label, .data
%                                  .R(),   .label, .data
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.FP     - FP #s in the order hit (gait trial only) 
%
% Output:   jntcenters returns a structure containing the transformed 
%             jntcenter trajectories, with the following format:
%               *.dataName(nVideoFrames), where dataName is derived from
%                       c.jntcenters.*.label, for all jntcenters in cSim
%               *.midHip(nVideoFrames)
%
% Notes:    Gillette Lab CS (if walking in +X) has X anterior, Y left, Z up
%           Darryl's Model CS has X anterior, Y up, Z right
%               
% ASA, 10-05


% Get walking/standing direction.
if strcmpi(c.trial.type, 'static')
    walkDir = 'posX';                   % stood in +x direction (static)
else
    if tInfo.FP{1} < tInfo.FP{2}        % walked in +x direction
         walkDir = 'posX';
    else
         walkDir = 'negX';              % walked in -x direction
    end
end

% For each joint center stored in c.jntcenters.R...
for jntcenterNum = 1:length(c.jntcenters.R)
    
    % Extract X, Y, Z trajectories of joint center (in lab CS).
    % NOTE:  The C3D Server returns cell arrays;
    %        these arrays are converted to numeric arrays.
    for vframeNum = 1:c.video.nframes
        x(vframeNum) = c.jntcenters.R(jntcenterNum).data{vframeNum, 1};
        y(vframeNum) = c.jntcenters.R(jntcenterNum).data{vframeNum, 2};
        z(vframeNum) = c.jntcenters.R(jntcenterNum).data{vframeNum, 3};
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
    dataName = c.jntcenters.R(jntcenterNum).label;
    eval(['jntcenters.', dataName, '= data_mCS;']);    
end

% For each joint center stored in c.jntcenters.L...
for jntcenterNum = 1:length(c.jntcenters.L)
    
    % Extract X, Y, Z trajectories of joint center (in lab CS).
    % NOTE:  The C3D Server returns cell arrays;
    %        these arrays are converted to numeric arrays.
    for vframeNum = 1:c.video.nframes
        x(vframeNum) = c.jntcenters.L(jntcenterNum).data{vframeNum, 1};
        y(vframeNum) = c.jntcenters.L(jntcenterNum).data{vframeNum, 2};
        z(vframeNum) = c.jntcenters.L(jntcenterNum).data{vframeNum, 3};
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
    dataName = c.jntcenters.L(jntcenterNum).label;
    eval(['jntcenters.', dataName, '= data_mCS;']);    
end

% Compute mid hip marker from midpoint of R and L hip markers;
% If dynamic hip centers are available, use them.
for jntcenterIndex = 1:length(c.jntcenters.R)
    RJCLabels{jntcenterIndex} = c.jntcenters.R(jntcenterIndex).label;
    LJCLabels{jntcenterIndex} = c.jntcenters.L(jntcenterIndex).label;
end
rHipIndex = strmatch('HIPR', RJCLabels);
lHipIndex = strmatch('HIPL', LJCLabels);
if isempty(rHipIndex) | isempty(lHipIndex)   
    jntcenters.midHip = (jntcenters.RFEP + jntcenters.LFEP)/2.0;
        % dynamic hip centers unavailable; use data from Vicon PIG model.
else
    jntcenters.midHip = (jntcenters.HIPR + jntcenters.HIPL)/2.0;
        % dynamic hip centers available; use these.
end
return;
