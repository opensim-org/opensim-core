function emgChannel = get_emgChannel(limb, muscle, ref_dataFormat)
% Purpose:  Returns the EMG channel corresponding to the specified limb
%           and muscle, assuming Gillette's "new" (i.e., post 11-19-01) 
%           10-channel protocol for EMG data collection.  
%
% Input:    limb is the limb of interest ('R' or 'L')
%           muscle is the muscle name of interest('string'); 
%               the name must correspond to a name in get_emgLabels()
%
% Output:   emgChannel is the channel number in the C3D file corresponding
%               to the specified limb and muscle.
%
% ASA, 9-05

for i=1:length(ref_dataFormat.emgChannels)
	if strcmpi(ref_dataFormat.emgChannels{i}{1}, limb) & strcmpi(ref_dataFormat.emgChannels{i}{2}, muscle)
		emgChannel = i;
		return 
	end
end

return;
