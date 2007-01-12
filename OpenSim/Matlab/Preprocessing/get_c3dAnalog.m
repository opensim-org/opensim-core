function a = get_c3dAnalog(itf, ref_dataFormat);
% Purpose:  This function uses the C3D Server from Motion Lab Systems to 
%           retrieve ANALOG data from a C3D file relevant for 
%           generating a subject-specific simulation. 
%
%           The C3D Server must be activated as a COM object using 
%           Matlab's ActiveX interface PRIOR to calling this function.  
%
% Input:    itf is the name given to the instance of the C3D Server 
%           activated by the calling function. 
%
% Output:   a returns a structure with the following format: 
%               a.analog            .rate,  .ratio
%               a.grf()             .label, .data
%               a.emg()             .label, .data
%           
%           The GRF data are scaled and returned in FP coordinates.
%           If the relevant data are not found in the file,
%           the corresponding arrays are returned empty.
%
% ASA 6-05

% Retrieve analog frame rate from ANALOG:RATE.
parameter_index = itf.GetParameterIndex('ANALOG', 'RATE');
item_index = 0;         % index corresponding to 1st item
analog.rate = itf.GetParameterValue(parameter_index, item_index);
% Retrieve # analog frames per video frame from the C3D file header
analog.ratio = itf.GetAnalogVideoRatio;
% Get video frame range from TRIAL:ACTUAL_START_FIELD, ACTUAL_END_FIELD;
% store number of video and analog frames.
item_index = 0;         % index corresponding to first frame number
nStartFrame = itf.GetVideoFrame(item_index);
item_index = 1;         % index corresponding to last frame number
nEndFrame = itf.GetVideoFrame(item_index);
nVideoFrames = nEndFrame - nStartFrame + 1;
nAnalogFrames = nVideoFrames * analog.ratio;
% Retrieve number of analog channels from ANALOG:USED
parameter_index = itf.GetParameterIndex('ANALOG', 'USED');
item_index = 0;         % index corresponding to 1st item
nAChanUsed = itf.GetParameterValue(parameter_index, item_index);
% Retrieve list of labels from ANALOG:LABELS
parameter_index = itf.GetParameterIndex('ANALOG', 'LABELS');
for item_index = 0:(nAChanUsed-1)   
    AChanLabels{item_index+1} = ...
                    itf.GetParameterValue(parameter_index, item_index);
end
% Initialize array used to retrieve analog data.
data = zeros(nAnalogFrames, 1);
% Initialize counters used to index arrays.
ctr_grf = 0;
ctr_emg = 0;
% Get reference list of labels describing data to be retrieved.
ref_labels = ref_dataFormat.c3dAnalogLabels;
for i = 1:nAChanUsed              % evaluate each ANALOG label
    channel_index = i-1;          % C3D Server uses 0-based index
    byScaled_index = char(49);    % ASCII '1' returns scaled data 
    byUseScale_index = char(48);  % ASCII '0' gets offset, scale from file
    dummy_offset = 0.0;           % dummy offset value
    dummy_scale = 1.0;            % dummy scale value
    switch(AChanLabels{i})
        
       % GRF DATA 
       % IF AChanLabels{i} is a label corresponding to GRF data,
       % update counter, store label, and retrieve data.
        case ref_labels.grf
            ctr_grf = ctr_grf + 1;
            grf(ctr_grf).label = AChanLabels{i};
            data = itf.GetAnalogDataEx(channel_index, ...
                        nStartFrame, nEndFrame, byScaled_index, ...
                        dummy_offset, dummy_scale, byUseScale_index);
            grf(ctr_grf).data = data;
       % EMG DATA
       % IF AChanLabels{i} is a label corresponding to EMG data,
       % update counter, store label, and retrieve data.
        case ref_labels.emg
            ctr_emg = ctr_emg + 1;
            emg(ctr_emg).label = AChanLabels{i};
            data = itf.GetAnalogDataEx(channel_index, ...
                        nStartFrame, nEndFrame, byScaled_index, ...
                        dummy_offset, dummy_scale, byUseScale_index);
            emg(ctr_emg).data = data;
    end
end
            
% Store data in structure and return.
% If data are unavailable (i.e., counter == 0), return an empty array.
a.analog = analog;
if ctr_grf > 0 
    a.grf = grf;
else
    a.grf = [];
end
if ctr_emg > 0 
    a.emg = emg;
else
    a.emg = [];
end
return;
