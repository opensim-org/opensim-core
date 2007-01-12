function c = store_c3dData(fname, ttype, e, f, v, a)
% Purpose:  This function packages the data read from a C3D file
%           into a structure.
%
% Input:    fname is the name of the C3D file that was read 
%           ttype is the trial type read from SUBJECTS:IS_STATIC
%           e is the structure returned by get_c3dEvents
%           f is the structure returned by get_c3dForcePlatform
%           v is the structure returned by get_c3dVideo
%           a is the structure returned by get_c3dAnalog
%
% Output:  c is a structure, formatted as described in read_c3dFile
% ASA 6-05, rev 7-05, rev 9-05


% Store 6-digit subject ID and 1- or 2-digit trial number from 'fname'.
dot_index = strfind(fname, '.');
[subject, trial.num] = strread(fname(1:dot_index), '%6s %s', 1);

% Identify trial type from the value read from SUBJECTS:IS_STATIC.
if ttype == 1            
    trial.type = 'static';
else
    trial.type = 'gait';
end

% Store data in structure and return.
c.subject = subject;                % 6-digit subject ID
c.trial = trial;                    % trial number and type
c.events = e;                       % event data
c.fpInfo = f;                       % force plate information
c.video = v.video;                  % video rate and nframes
c.markers = v.markers;              % marker data
c.jntcenters = v.jntcenters;        % R/L joint center data
c.jntangles = v.jntangles;          % R/L joint angle data
c.jntmoments = v.jntmoments;        % R/L joint moment data
c.jntpowers = v.jntpowers;          % R/L joint power data
c.analog = a.analog;                % analog rate and analog/video ratio
c.grf = a.grf;                      % GRF data
c.emg = a.emg;                      % EMG data
return;