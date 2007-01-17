function emgEnsembleAve = get_emgEnsembleAve(emgEnvResamp, nChannels, ref_dataFormat)
% Purpose:  Computes ensemble average of the EMG envelopes for each channel.
%
% Input:    emgEnvResamp is a structure with the following format:
%               *(fileNum).R{cycleNum}{emgChannel}.nlow
%               *(fileNum).L{cycleNum}{emgChannel}.nlow
%               *(fileNum).subject   - subject number 
%               *(fileNum).trial     - trial number
%           nChannels specifies the number of EMG channels
%     
% Ouptput:  emgEnsembleAve is a structure with the following format:
%           *(emgChannel).limb 
%                        .muscle 
%                        .envAve
%                        .envSD
%
% Note:     This code calls a subfunction that assumes subjects were 
%           tested using Gillette's 'new' 10-channel protocol for EMG
%           data collection (i.e., tested after 11-19-01).
%
% ASA, 12-05


% Get number of files to be averaged.
nFiles = length(emgEnvResamp);

% For each analog EMG channel ...
for emgChannel = 1:nChannels        

    % Get limb and muscle corresponding to the current EMG channel.
    [limb, muscle] = get_emgLabels(emgChannel, ref_dataFormat);
   
	if ~strcmp(limb,'R') & ~strcmp(limb,'L')
		error('Expected limb R or L');
	end

    % Initialize counter for counting trials/cycles.    
    count = 0;
    
	for fileNum = 1:nFiles                             % for each file             
		nCycles = length(emgEnvResamp(fileNum).(limb));     % for each cycle 
		for cycleNum = 1:nCycles
			count = count + 1;
			data(:, count) = ...
				emgEnvResamp(fileNum).(limb){cycleNum}{emgChannel}.nlow;
		end
	end
  
    % Compute ensemble average and standard deviation.
    % Need transpose since mean, SD is computed for each column.
    envAve = mean(data');
    envSD = std(data');
  
    % Store the normalized rectified EMG data and EMG envelopes,
    % sampled at analog frame rate.
    emgEnsembleAve(emgChannel).limb = limb;
    emgEnsembleAve(emgChannel).muscle = muscle;    
    emgEnsembleAve(emgChannel).envAve = envAve;
    emgEnsembleAve(emgChannel).envSD = envSD;
end
return;
    
