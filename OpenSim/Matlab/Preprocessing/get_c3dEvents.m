function e = get_c3dEvents(itf);
% Purpose:  This function uses the C3D Server from Motion Lab Systems 
%           to retrieve EVENT data from a C3D file relevant for 
%           generating a subject-specific simulation. 
%
%           The C3D Server must be activated as a COM object using 
%           Matlab's ActiveX interface PRIOR to calling this function.  
%
% Input:    itf is the name given to the instance of the C3D Server 
%           activated by the calling function. 
%
% Output:   e returns a structure with the following format: 
%               e().label - concatenates the C3D EVENT parameters 
%                           'CONTEXTS' and 'LABELS'
%               e().time  - stores times corresponding to events
%               The times are sorted and returned in ascending order.
%               
% ASA 7-05
% Retrieve number of events from EVENT:USED.
parameter_index = itf.GetParameterIndex('EVENT', 'USED');
if parameter_index ~= -1
	item_index = 0;                     % index corresponding to 1st item
	nEvents = itf.GetParameterValue(parameter_index, item_index);
else
	disp('get_c3dEvents: No EVENT:USED parameter, assuming 0 events');
	nEvents = 0;
end
% Retrieve limb corresponding to each event from EVENT:CONTEXTS.
parameter_index = itf.GetParameterIndex('EVENT', 'CONTEXTS');
for eventIndex = 1:nEvents
    item_index = eventIndex - 1;    % C3D Server uses 0-based index 
    limb{eventIndex} = itf.GetParameterValue(parameter_index, item_index);
end
% Retrieve label corresponding to each event from EVENT:LABELS.
parameter_index = itf.GetParameterIndex('EVENT', 'LABELS');
for eventIndex = 1:nEvents
    item_index = eventIndex - 1;    % C3D Server uses 0-based index 
    label{eventIndex} = itf.GetParameterValue(parameter_index, item_index);
end
% Retrieve time corresponding to each event from EVENT:TIMES.
parameter_index = itf.GetParameterIndex('EVENT', 'TIMES');
for eventIndex = 1:nEvents
    item_index = 2*eventIndex - 1;     
        % skip the extra 0.0000s in C3D file by reading every other value
    time{eventIndex} = itf.GetParameterValue(parameter_index, item_index);
end
% Store data in structure and return.
% If data are unavailable, return an empty array.
if nEvents > 0
    [sortedTime, sortIndex] = sort([time{1:length(time)}]);
                                        % sort times in ascending order
    for eventIndex = 1:nEvents
        e(eventIndex).label = ...
          [limb{sortIndex(eventIndex)}, ' ', label{sortIndex(eventIndex)}];
        e(eventIndex).time = sortedTime(eventIndex);
    end
else
    e = [];    
end
return;
