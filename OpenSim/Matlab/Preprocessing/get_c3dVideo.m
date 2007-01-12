function v = get_c3dVideo(itf, ref_dataFormat);
% Purpose:  This function uses the C3D Server from Motion Lab Systems to 
%           retrieve POINT/VIDEO data from a C3D file relevant for 
%           generating a subject-specific simulation. 
%
%           The C3D Server must be activated as a COM object using 
%           Matlab's ActiveX interface PRIOR to calling this function.  
%
% Input:    itf is the name given to the instance of the C3D Server 
%           activated by the calling function. 
%
% Output:   v returns a structure with the following format: 
%               v.video          .rate,  .nframes, .units
%               v.markers()             .label, .data
%               v.jntcenters    .L(),   .label, .data
%                               .R(),   .label, .data
%               v.jntangles     .L(),   .label, .data 
%                               .R(),   .label, .data
%               v.jntmoments    .L(),   .label, .data
%                               .R(),   .label, .data
%               v.jntpowers     .L(),   .label, .data
%                               .R(),   .label, .data
%
%           If the relevant data are not found in the file,
%           the corresponding arrays are returned empty.
%
% ASA 6-05
% Retrieve video frame rate from POINT:RATE.
parameter_index = itf.GetParameterIndex('POINT', 'RATE');
item_index = 0;         % index corresponding to 1st item
video.rate = itf.GetParameterValue(parameter_index, item_index);
% Get video frame range from TRIAL:ACTUAL_START_FIELD, ACTUAL_END_FIELD;
% store number of video frames.
item_index = 0;         % index corresponding to first frame number
nStartFrame = itf.GetVideoFrame(item_index);
item_index = 1;         % index corresponding to last frame number
nEndFrame = itf.GetVideoFrame(item_index);
video.nframes = nEndFrame - nStartFrame + 1;
% Retrieve units of marker data from POINT:UNITS.
parameter_index = itf.GetParameterIndex('POINT', 'UNITS');
item_index = 0;         % index corresponding to 1st item
video.units = itf.GetParameterValue(parameter_index, item_index);
% Retrieve number of point records from POINT:USED.
parameter_index = itf.GetParameterIndex('POINT', 'USED');
item_index = 0;         % index corresponding to 1st item
nPtUsed = itf.GetParameterValue(parameter_index, item_index);
% Retrieve list of labels from POINT:LABELS
parameter_index = itf.GetParameterIndex('POINT', 'LABELS');
for item_index = 0:(nPtUsed-1) 
    PtLabels{item_index+1} = ...
                itf.GetParameterValue(parameter_index, item_index);
end
% Initialize arrays used to retrieve video data.
dataX = zeros(video.nframes, 1);
dataY = zeros(video.nframes, 1);
dataZ = zeros(video.nframes, 1);
% Initialize counters used to index arrays.
ctr_markers = 0;
ctr_jntcenters.L = 0;
ctr_jntcenters.R = 0;
ctr_jntangles.L = 0;
ctr_jntangles.R = 0;
ctr_jntmoments.L = 0;
ctr_jntmoments.R = 0;
ctr_jntpowers.L = 0;
ctr_jntpowers.R = 0;
% Get reference list of labels describing data to be retrieved.
ref_labels = ref_dataFormat.c3dVideoLabels;
for i = 1:nPtUsed               % evaluate each POINT/VIDEO label
    channel_index = i-1;        % C3D Server uses 0-based index 
    byScaled_index = char(48);  % ASCII '0' returns unscaled data 
                                      
    switch(PtLabels{i})
        
       % MARKER DATA 
       % IF PtLabels{i} is a label corresponding to marker data,
       % update counter, store label, and retrieve data.
        case ref_labels.markers
            ctr_markers = ctr_markers + 1;
            markers(ctr_markers).label = PtLabels{i};
            dataX = itf.GetPointDataEx(channel_index, 0, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataY = itf.GetPointDataEx(channel_index, 1, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataZ = itf.GetPointDataEx(channel_index, 2, ...
                            nStartFrame, nEndFrame, byScaled_index);
            markers(ctr_markers).data = [dataX dataY dataZ];
       % JOINT CENTER DATA, LEFT LIMB
       % IF PtLabels{i} is a label corresponding to L joint center data,
       % update counter, store label, and retrieve data.
        case ref_labels.jntcenters.L
            ctr_jntcenters.L = ctr_jntcenters.L + 1;
            jntcenters.L(ctr_jntcenters.L).label = PtLabels{i};
            dataX = itf.GetPointDataEx(channel_index, 0, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataY = itf.GetPointDataEx(channel_index, 1, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataZ = itf.GetPointDataEx(channel_index, 2, ...
                            nStartFrame, nEndFrame, byScaled_index);
            jntcenters.L(ctr_jntcenters.L).data = [dataX dataY dataZ];
    
      % JOINT CENTER DATA, RIGHT LIMB
       % IF PtLabels{i} is a label corresponding to R joint center data,
       % update counter, store label, and retrieve data.            
        case ref_labels.jntcenters.R
            ctr_jntcenters.R = ctr_jntcenters.R + 1;
            jntcenters.R(ctr_jntcenters.R).label = PtLabels{i};
            dataX = itf.GetPointDataEx(channel_index, 0, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataY = itf.GetPointDataEx(channel_index, 1, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataZ = itf.GetPointDataEx(channel_index, 2, ...
                            nStartFrame, nEndFrame, byScaled_index);
            jntcenters.R(ctr_jntcenters.R).data = [dataX dataY dataZ];
            
       % JOINT ANGLE DATA, LEFT LIMB
       % IF PtLabels{i} is a label corresponding to L joint angle data,
       % update counter, store label, and retrieve data.
        case ref_labels.jntangles.L
            ctr_jntangles.L = ctr_jntangles.L + 1;
            jntangles.L(ctr_jntangles.L).label = PtLabels{i};
            dataX = itf.GetPointDataEx(channel_index, 0, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataY = itf.GetPointDataEx(channel_index, 1, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataZ = itf.GetPointDataEx(channel_index, 2, ...
                            nStartFrame, nEndFrame, byScaled_index);
            jntangles.L(ctr_jntangles.L).data = [dataX dataY dataZ];
            
       % JOINT ANGLE DATA, RIGHT LIMB
       % IF PtLabels{i} is a label corresponding to R joint angle data,
       % update counter, store label, and retrieve data.
        case ref_labels.jntangles.R
            ctr_jntangles.R = ctr_jntangles.R + 1;
            jntangles.R(ctr_jntangles.R).label = PtLabels{i};
            dataX = itf.GetPointDataEx(channel_index, 0, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataY = itf.GetPointDataEx(channel_index, 1, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataZ = itf.GetPointDataEx(channel_index, 2, ...
                            nStartFrame, nEndFrame, byScaled_index);
            jntangles.R(ctr_jntangles.R).data = [dataX dataY dataZ];  
            
       % JOINT MOMENT DATA, LEFT LIMB
       % IF PtLabels{i} is a label corresponding to L joint moment data,
       % update counter, store label, and retrieve data.
        case ref_labels.jntmoments.L
            ctr_jntmoments.L = ctr_jntmoments.L + 1;
            jntmoments.L(ctr_jntmoments.L).label = PtLabels{i};
            dataX = itf.GetPointDataEx(channel_index, 0, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataY = itf.GetPointDataEx(channel_index, 1, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataZ = itf.GetPointDataEx(channel_index, 2, ...
                            nStartFrame, nEndFrame, byScaled_index);
            jntmoments.L(ctr_jntmoments.L).data = [dataX dataY dataZ];
            
       % JOINT MOMENT DATA, RIGHT LIMB
       % IF PtLabels{i} is a label corresponding to R joint moment data,
       % update counter, store label, and retrieve data.
        case ref_labels.jntmoments.R
            ctr_jntmoments.R = ctr_jntmoments.R + 1;
            jntmoments.R(ctr_jntmoments.R).label = PtLabels{i};
            dataX = itf.GetPointDataEx(channel_index, 0, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataY = itf.GetPointDataEx(channel_index, 1, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataZ = itf.GetPointDataEx(channel_index, 2, ...
                            nStartFrame, nEndFrame, byScaled_index);
            jntmoments.R(ctr_jntmoments.R).data = [dataX dataY dataZ];
            
       % JOINT POWER DATA, LEFT LIMB
       % IF PtLabels{i} is a label corresponding to L joint power data,
       % update counter, store label, and retrieve data.
        case ref_labels.jntpowers.L
            ctr_jntpowers.L = ctr_jntpowers.L + 1;
            jntpowers.L(ctr_jntpowers.L).label = PtLabels{i};
            dataX = itf.GetPointDataEx(channel_index, 0, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataY = itf.GetPointDataEx(channel_index, 1, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataZ = itf.GetPointDataEx(channel_index, 2, ...
                            nStartFrame, nEndFrame, byScaled_index);
            jntpowers.L(ctr_jntpowers.L).data = [dataX dataY dataZ];
            
       % JOINT POWER DATA, RIGHT LIMB
       % IF PtLabels{i} is a label corresponding to R joint power data,
       % update counter, store label, and retrieve data.
        case ref_labels.jntpowers.R
            ctr_jntpowers.R = ctr_jntpowers.R + 1;
            jntpowers.R(ctr_jntpowers.R).label = PtLabels{i};
            dataX = itf.GetPointDataEx(channel_index, 0, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataY = itf.GetPointDataEx(channel_index, 1, ...
                            nStartFrame, nEndFrame, byScaled_index);
            dataZ = itf.GetPointDataEx(channel_index, 2, ...
                            nStartFrame, nEndFrame, byScaled_index);
            jntpowers.R(ctr_jntpowers.R).data = [dataX dataY dataZ];
    end
end
            
% Store data in structure and return.
% If data are unavailable (i.e., counter == 0), return an empty array.
v.video = video;
if ctr_markers > 0 
    v.markers = markers;
else
    v.markers = [];
end
if ctr_jntcenters.L > 0 & ctr_jntcenters.R > 0
    v.jntcenters = jntcenters;
else
    v.jntcenters.R = [];
    v.jntcenters.L = [];
end    
if ctr_jntangles.L > 0 & ctr_jntangles.R > 0
    v.jntangles = jntangles;
else
    v.jntangles.R = [];
    v.jntangles.L = [];
end 
if ctr_jntmoments.L > 0 & ctr_jntmoments.R > 0
    v.jntmoments = jntmoments;
else
    v.jntmoments.R = [];
    v.jntmoments.L = [];
end 
if ctr_jntpowers.L > 0 & ctr_jntpowers.R > 0
    v.jntpowers = jntpowers;
else
    v.jntpowers.R = [];
    v.jntpowers.L = [];
end 
return;
