function [limb, muscle] = get_emgLabels(emgChannel, ref_dataFormat)
% Purpose:  Returns the limb and muscle corresponding to the specified
%           EMG channel, assuming Gillette's "new" (i.e., post 11-19-01) 
%           10-channel protocol for EMG data collection.
%
% Input:    emgChannel is the number of the EMG channel of interest
%           
% Output:   limb returns the corresponding limb ('R' or 'L')
%           muscle returns the name of the corresponding muscle ('string')
%
% ASA, 9-05

if emgChannel > length(ref_dataFormat.emgChannels)
	error(sprintf('get_emgLabels: EMG channel %d has no label', emgChannel));
end

limb = ref_dataFormat.emgChannels{emgChannel}{1};
muscle = ref_dataFormat.emgChannels{emgChannel}{2};

return;
