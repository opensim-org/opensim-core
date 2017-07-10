function [handles] = InteractiveHopperSettings(handles,saveOrLoad,varargin)
% INTERACTIVEHOPPERSETTINGS
%   Save or load GUI settings from the InteractiveHopper example

%-----------------------------------------------------------------------%
% The OpenSim API is a toolkit for musculoskeletal modeling and         %
% simulation. See http://opensim.stanford.edu and the NOTICE file       %
% for more information. OpenSim is developed at Stanford University     %
% and supported by the US National Institutes of Health (U54 GM072970,  %
% R24 HD065690) and by DARPA through the Warrior Web program.           %
%                                                                       %
% Copyright (c) 2017 Stanford University and the Authors                %
% Author(s): Nick Bianco                                                %
%                                                                       %
% Licensed under the Apache License, Version 2.0 (the "License");       %
% you may not use this file except in compliance with the License.      %
% You may obtain a copy of the License at                               %
% http://www.apache.org/licenses/LICENSE-2.0.                           %
%                                                                       %
% Unless required by applicable law or agreed to in writing, software   %
% distributed under the License is distributed on an "AS IS" BASIS,     %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or       %
% implied. See the License for the specific language governing          %
% permissions and limitations under the License.                        %
%-----------------------------------------------------------------------%

% Parse inputs
p = inputParser();

defaultFilename = 'file';
defaultSetDefaults = false;

addOptional(p,'filename',defaultFilename)
addOptional(p,'setDefaults',defaultSetDefaults)

parse(p,varargin{:})

filename = p.Results.filename;
setDefaults = p.Results.setDefaults;

% Prescibe relevant subfields from GUI handles
subfields = {'Value','Enable','String','Min','Max'};

% Save or load current GUI settings
switch saveOrLoad
    case 'save'
        saveHandles(handles,filename,subfields);
    case 'load'
        if setDefaults
            settings = getDefaults(subfields);
        else
            settings = load(filename);
        end
        
        handles = loadHandles(handles,settings,subfields);      
end

% LOADHANDLES
%   Given a MAT file containing settings for the InteractiveHopper GUI,
%   update the GUI handles to reflect the settings in the relevant setting
%   subfields.
function [handles] = loadHandles(handles,settings,subfields)

handleNames = fieldnames(settings);

for i = 1:length(handleNames)
    for j = 1:length(subfields)
        if isfield(settings.(handleNames{i}),subfields{j})
            handles.(handleNames{i}).(subfields{j}) = settings.(handleNames{i}).(subfields{j});           
        end
    end
end

% SAVEHANDLES
%   Given the current set of handles from the InteractiveHopper GUI, save 
%   the settings from each handle's relevant subfields. Settings are saved
%   to a MAT file in the current working directory.
function saveHandles(handles,filename,subfields)

settings = struct();
handleNames = fieldnames(handles);

for i = 1:length(handleNames)
    for j = 1:length(subfields)
        if isstruct(handles.(handleNames{i}))
            if isfield(handles.(handleNames{i}),subfields{j})
                settings.(handleNames{i}).(subfields{j}) = handles.(handleNames{i}).(subfields{j});
            end
        else
            if isprop(handles.(handleNames{i}),subfields{j})
                settings.(handleNames{i}).(subfields{j}) = handles.(handleNames{i}).(subfields{j});
            end
        end            
    end
end

save(filename,'-struct','settings')

% SETSUBFIELDS
%   Set values of settings subfields based on optional inputs.
function S = setSubfields(S,subfields,field,varargin)

p = inputParser;

for i = 1:length(subfields)
   addOptional(p,subfields{i},NaN) 
end

parse(p,varargin{:});

for i = 1:length(subfields)
    if ~isnan(p.Results.(subfields{i}))
        fieldnames = {field subfields{i}};
        S = setfield(S,fieldnames{:},p.Results.(subfields{i}));
    end  
end

% GETDEFAULTS
%   These settings reflect the default settings that are called by the
%   InteractiveHopper example upon start up. 
function S = getDefaults(subfields)

muscle = InteractiveHopperParameters('defaultMuscle');

muscleFunc = InteractiveHopperParameters(muscle);
[max_isometric_force,optimal_fiber_length,tendon_slack_length,muscle_mass] = muscleFunc();

controls = InteractiveHopperParameters('controls');
[muscleExcitation,muscleExcitationColor,deviceControl,deviceControlColor] = controls();

passiveSlider = InteractiveHopperParameters('passiveSlider');
[passive_slider,passive_min,passive_max] = passiveSlider();

activeSlider = InteractiveHopperParameters('activeSlider');
[active_slider,active_min,active_max] = activeSlider();

S = struct();

S = setSubfields(S,subfields,'active_as_prop_myo'        ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'active_mass'               ,'Value',0,'Enable','off','String',' ');
S = setSubfields(S,subfields,'active_mass_text'          ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'active_mass_units'         ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'active_patella_wrap'       ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'active_slider'             ,'Value',active_slider,'Enable','off' ...
                                                         ,'Min',active_min,'Max',active_max);
S = setSubfields(S,subfields,'arnold'                    ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'average_joe'               ,'Value',1,'Enable','on');
S = setSubfields(S,subfields,'clear'                     ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'deviceControl'             ,'Value',deviceControl);
S = setSubfields(S,subfields,'deviceControlColor'        ,'Value',deviceControlColor);
S = setSubfields(S,subfields,'enable_active'             ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'enable_passive'            ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'katie_ledecky'             ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'load_setup'                ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'max_isometric_force'       ,'Value',max_isometric_force,'Enable','off' ...
                                                         ,'String',num2str(max_isometric_force));
S = setSubfields(S,subfields,'max_isometric_force_text'  ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'max_isometric_force_units' ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'max_jump_best'             ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'max_jump_best_text'        ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'max_jump_recent'           ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'max_jump_recent_text'      ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'muscle'                    ,'Value',muscle);
S = setSubfields(S,subfields,'muscleExcitation'          ,'Value',muscleExcitation);
S = setSubfields(S,subfields,'muscleExcitationColor'     ,'Value',muscleExcitationColor);
S = setSubfields(S,subfields,'muscle_mass'               ,'Value',muscle_mass,'Enable','off' ...
                                                         ,'String',num2str(muscle_mass));
S = setSubfields(S,subfields,'muscle_mass_text'          ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'muscle_mass_units'         ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'new_device_control'        ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'new_musc_excitation'       ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'passive_mass'              ,'Enable','off','String',' ');
S = setSubfields(S,subfields,'passive_mass_text'         ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'passive_mass_units'        ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'passive_patella_wrap'      ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'passive_slider'            ,'Value',passive_slider,'Enable','off' ...
                                                         ,'Min',passive_min,'Max',passive_max);
S = setSubfields(S,subfields,'reset'                     ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'save_setup'                ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'setup'                     ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'simulate'                  ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'stiffness'                 ,'Enable','off','String',' ');
S = setSubfields(S,subfields,'stiffness_text'            ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'stiffness_units'           ,'Value',0,'Enable','off');
S = setSubfields(S,subfields,'tendon_slack_length'       ,'Value',tendon_slack_length,'Enable','off' ...
                                                         ,'String',num2str(tendon_slack_length));
S = setSubfields(S,subfields,'tendon_slack_length_text'  ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'tendon_slack_length_units' ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'optimal_fiber_length'      ,'Value',optimal_fiber_length,'Enable','off' ...
                                                         ,'String',num2str(optimal_fiber_length));
S = setSubfields(S,subfields,'optimal_fiber_length_text' ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'optimal_fiber_length_units','Value',0,'Enable','on');
S = setSubfields(S,subfields,'tension_or_gain'           ,'Enable','off','String',' ');
S = setSubfields(S,subfields,'tension_or_gain_text'      ,'Value',0,'Enable','off','String','Max Tension');
S = setSubfields(S,subfields,'tension_or_gain_units'     ,'Value',0,'Enable','off','String','N');
S = setSubfields(S,subfields,'visualize'                 ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'with_device'               ,'Value',0,'Enable','on');
S = setSubfields(S,subfields,'without_device'            ,'Value',1,'Enable','on');
