function cStatic = extract_c3dStatic(c, startFrame)
% Purpose:  Extracts a segment of data from structure c, read from the
%           static C3D file of a Gillette subject, and returns a new
%           structure, similar in format to c, but starting at startFrame.
%           This function was written to delete bad frames of marker data
%           that frequently appear at the beginning of static trials.
%
% Input:    c is a structure returned from read_c3DFile()
%           startFrame is the video frame to start extracting data
%        
% Output:   cStatic is a structure with the following format:  
%               cStatic.subject
%               cStatic.trial                 .num, .type 
%                 
%               POINT/VIDEO Data:
%               cStatic.video         .rate,  .nframes, .units
%               cStatic.markers()             .label, .data
%               cStatic.jntcenters    .L(),   .label, .data
%                                     .R(),   .label, .data
%               cStatic.jntangles     .L(),   .label, .data 
%                                     .R(),   .label, .data
%               cStatic.jntmoments    .L(),   .label, .data
%                                     .R(),   .label, .data
%               cStatic.jntpowers     .L(),   .label, .data
%                                     .R(),   .label, .data
%
% ASA, 11-05


% Specify the video frame numbers to start and stop extracting data.
start.vframe = startFrame;
stop.vframe = c.video.nframes;

% Extract data from c and store in new structure ...
cStatic.subject = c.subject;
cStatic.trial = c.trial;

% Update number of video frames and Extract POINT/VIDEO data.
cStatic.video.rate = c.video.rate;
cStatic.video.nframes = stop.vframe - start.vframe + 1;
cStatic.video.units = c.video.units;
for mkrNum = 1:length(c.markers)
    cStatic.markers(mkrNum).label = c.markers(mkrNum).label;
    cStatic.markers(mkrNum).data = ...
        c.markers(mkrNum).data(start.vframe:stop.vframe, :);
end
for jcNum = 1:length(c.jntcenters.R)
    cStatic.jntcenters.L(jcNum).label = c.jntcenters.L(jcNum).label;
    cStatic.jntcenters.L(jcNum).data = ...
        c.jntcenters.L(jcNum).data(start.vframe:stop.vframe, :);
    cStatic.jntcenters.R(jcNum).label = c.jntcenters.R(jcNum).label;
    cStatic.jntcenters.R(jcNum).data = ...
        c.jntcenters.R(jcNum).data(start.vframe:stop.vframe, :);
end
for jaNum = 1:length(c.jntangles.R)
    cStatic.jntangles.L(jaNum).label = c.jntangles.L(jaNum).label;
    cStatic.jntangles.L(jaNum).data = ...
        c.jntangles.L(jaNum).data(start.vframe:stop.vframe, :);
    cStatic.jntangles.R(jaNum).label = c.jntangles.R(jaNum).label;
    cStatic.jntangles.R(jaNum).data = ...
        c.jntangles.R(jaNum).data(start.vframe:stop.vframe, :);
end
for jmNum = 1:length(c.jntmoments.R)
    cStatic.jntmoments.L(jmNum).label = c.jntmoments.L(jmNum).label;
    cStatic.jntmoments.L(jmNum).data = ...
        c.jntmoments.L(jmNum).data(start.vframe:stop.vframe, :);
    cStatic.jntmoments.R(jmNum).label = c.jntmoments.R(jmNum).label;
    cStatic.jntmoments.R(jmNum).data = ...
        c.jntmoments.R(jmNum).data(start.vframe:stop.vframe, :);
end
for jpNum = 1:length(c.jntpowers.R)
    cStatic.jntpowers.L(jpNum).label = c.jntmoments.L(jpNum).label;
    cStatic.jntpowers.L(jpNum).data = ...
        c.jntpowers.L(jpNum).data(start.vframe:stop.vframe, :);
    cStatic.jntpowers.R(jpNum).label = c.jntpowers.R(jpNum).label;
    cStatic.jntpowers.R(jpNum).data = ...
        c.jntpowers.R(jpNum).data(start.vframe:stop.vframe, :);
end
return;