function proEMG = process_emgChannel(c, emgChannel)
% Purpose:  Retrieves EMG data for the specified channel and performs the
%           filtering, rectifying, and normalizing operations needed to
%           obtain an EMG envelope.
%
% Input:    c is a structure from read_c3DFile()
%           emgChannel specifies the EMG channel number 
%
% Output:  proEMG returns a structure with the following format,
%          sampled at the analog frame rate:
%          *.raw   - raw EMG signal, as read from the C3D file
%          *.band  - result of applying band-pass filter to raw data
%          *.rect  - result of full-wave rectifying the filtered data
%          *.low   - result of applying low-pass filter to rectified data
%          *.nrect - result of normalizing rectified data by max/min values
%          *.nlow  - result of normalizing filtered data by max/min values
%
% NOTE:    This function should be used only on full trials of data in c,
%          and not on extracted segments of data (e.g. 'simulateable' or
%          individual 'cycles' of data).  This is because a large # of
%          data points are needed to apply the 4th order filters 
%          prescribed here.
%
% ASA, 9-05


% Get number of analog frames.
nAnalogFrames = c.video.nframes * c.analog.ratio;

% Extract raw EMG signal.
% NOTE:  The C3D Server returns a cell array;
%        this array is converted to a numeric array.
rawEMG = cell2mat(c.emg(emgChannel).data(1:nAnalogFrames));

% Apply 4th order, 0-lag, Butterworth band-pass filter to raw EMG signal.
order = 4;
fs = c.analog.rate;
if fs == 1080
	cutoff = [20 400];                % default
	% cutoff = [80 400];              % use when there is 60 Hz noise
elseif fs == 600	% For Delaware EMG data
	cutoff = [11 222];                % default
	% cutoff = [44 222];              % use when there is 60 Hz noise
end
[b, a] = butter(order/2, cutoff/(0.5*fs));
bandEMG = filtfilt(double(b), double(a), double(rawEMG));  

% Rectify the filtered EMG signal.
rectEMG = abs(bandEMG);

% Apply 4th order, 0-lag, Butterworth low-pass filter to rectified signal.
order = 4;
cutoff = [10];
fs = c.analog.rate;
[b, a] = butter(order, cutoff/(0.5*fs));
lowEMG = filtfilt(double(b), double(a), double(rectEMG));  

% Normalize rectified and low-pass filtered EMG signals.
nMinSamples = round(0.01 * nAnalogFrames);  
                                % average 1% of samples to get "min".
sortRect = sort(rectEMG);           
minRect = mean(sortRect(1:nMinSamples));
normRect = (rectEMG - minRect)/ ...
            (max(rectEMG) - minRect);
sortLow = sort(lowEMG);
minLow = mean(sortLow(1:nMinSamples));
normLow = (lowEMG - minLow)/ ...
            (max(lowEMG) - minLow);
        
% Return raw and processed EMG data in structure.        
proEMG.raw = rawEMG;
proEMG.band = bandEMG;
proEMG.rect = rectEMG;
proEMG.low = lowEMG;
proEMG.nrect = normRect;
proEMG.nlow = normLow;                
return;

