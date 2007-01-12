% c3dPreProScript_sample.m
% Purpose:  This script pre-processes experimental data for a 
%           'simulateable' segment of data read from the C3D file 
%           of a Gillette control subject. 
%
% Format for Subject/Trial Information:
%       subject is the 6-digit subject ID ('character array')
%       tInfo is a structure containing the following 'trial info':
%           *.trial  - trial number to analyze ('character array')
%           *.mass   - mass of subject in kg
%           *.speed  - average walking speed during trial in m/s
%           *.FP     - FP #s in the order hit; 
%                       my best guess from visual inspection (cell array)
%           *.limb   - limbs corresponding to FP strikes (cell array)
%           *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%           *.static - trial number of corresponding static trial;
%                      cell array:  1st cell = 'static trial #'
%                                   2nd cell = 'all' or 'min',
%                                   corresponding to available marker set
%
%           NOTES:
%           (1) tInfo can have any name, such as 'ss', 'fast', 'slow' ...
%           (2) set ictoMatrix = [] to call get_ictoEvents();
%               this function retrieves the necessary 'IC' and 'TO' events
%               if this information has not been stored previously.
%           (3) set static = {} to write only anatomical markers,
%               and not tracking markers, to file.
%
% ASA, 11-05; revised 12-05


%%% Case 1:  
%   Representative trial at self-selected speed,
%   matrix of event frames unknown,
%   no static trial.
subject = '900060';
tInfo.trial = '10';
tInfo.mass = 31.7;             % in kg
tInfo.speed = 1.32;            % in m/s
tInfo.FP = {1 2 3};            % FP#s in order hit, as numbered in C3D file    
tInfo.limb = {'L', 'R', 'L'};  % limbs corresponding to FP hits
tInfo.ictoMatrix = [];
tInfo.static = {};
prepro_GilletteControls(subject, tInfo);
clear tInfo refTrials;


%%% Case 2:  
%   Representative trial at self-selected speed,
%   with matrix of event frames known (saved from previous processing),
%   no static trial.
subject = '900060';
tInfo.trial = '10';
tInfo.mass = 31.7;
tInfo.speed = 1.32;
tInfo.FP = {1 2 3};
tInfo.limb = {'L', 'R', 'L'};
tInfo.ictoMatrix = [1000 1071 1459 1569 1950; ...
                    1459 1569 1950 2043 2421; ...
                    1950 2043 2421 2524 2907];
tInfo.static = {};
prepro_GilletteControls(subject, tInfo);
clear tInfo refTrials;


%%% Case 3:  
%   Representative trial at self-selected speed,
%   with matrix of event frames known (saved from previous processing),
%   with static trial.
subject = '900060';
tInfo.trial = '10';
tInfo.mass = 31.7;
tInfo.speed = 1.32;
tInfo.FP = {1 2 3};
tInfo.limb = {'L', 'R', 'L'};
tInfo.ictoMatrix = [1000 1071 1459 1569 1950; ...
                    1459 1569 1950 2043 2421; ...
                    1950 2043 2421 2524 2907];
tInfo.static = {'2', 'all'};
prepro_GilletteControls(subject, tInfo);
clear tInfo refTrials;


%%% Case 4:
%   Representative trial at self-selected speed,
%   with matrix of event times known (saved from previous processing),
%   with a knee axis malalignment.
subject = '900019';
tInfo.trial = '12';
tInfo.mass = 44.2;
tInfo.speed = 1.30;
tInfo.FP = {4 3 2 1};
tInfo.limb = {'R', 'L', 'R', 'L'};
tInfo.ictoMatrix = [1132 1251 1652 1756 2168; ...
                    1652 1756 2168 2278 2681; ...
                    2168 2278 2681 2790 3204; ...
                    2681 2790 3204 3314 3726];
tInfo.static = {};
prepro_GilletteControls(subject, tInfo);
clear tInfo refTrials;




% NOTES TO SARYN:
% 1.  The sample file is named '900060 10.c3d'
%     My code creates the file name from the subject and trial information.
%     You will need to modify this part of the code to work with your
%     file naming convention.
%
% 2.  If you don't know the analog frames corresponding to IC and TO
%     events, set tInfo.ictoMatrix = [].  The code will call a function
%     that allows the user to interactively estimate these events.  This
%     function writes the corresponding frames to the screen.  If you then 
%     cut and paste these event values into ictoMatrix in the script, 
%     the code will skip this step the next time.  This is especially 
%     useful if you want to run other pre-processing functions for the
%     same subject (for example, I have other code that writes
%     EMG envelopes to a motion file for comparison with CMC-generated 
%     muscle excitations.
%
% 3.  tInfo.FP and tInfo.limb are critical.
%     tInfo.mass may be critical -- I can't remember where this gets used.
%     tInfo.speed is not critical -- I used this when labelling plots
%           basically for my own reference.  It would be easy to get rid 
%           of it in the code.
%     
% 4.  prepro_GilletteControls.m is the main pre-processing function.
%     It could be simplified.   Some of my subjects had static trials, 
%     some didn't.   Some of the C3D files had all of the marker data,
%     some didn't.   I had to hack a few options...    
%
% 5.  Several of the sub-functions are interactive.  I suspect it will be
%     easier to explain these over the phone.   Call when you are ready.
%
% 6.  If you quit the function (ctrx-c) when one of the menu boxes is 
%     waiting for input, Matlab is likely to crash.  While this is somewhat
%     annoying, the menu boxes were so helpful that I used them anyway.
%
% 7.  Please delete the sample Gillette data files from your machine
%     when you are done with them.  They should not be shared.
