function varargout = InteractiveHopper(varargin)
% INTERACTIVEHOPPER MATLAB code for InteractiveHopper.fig
%      INTERACTIVEHOPPER, by itself, creates a new INTERACTIVEHOPPER or raises the existing
%      singleton*.
%
%      H = INTERACTIVEHOPPER returns the handle to a new INTERACTIVEHOPPER or the handle to
%      the existing singleton*.
%
%      INTERACTIVEHOPPER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERACTIVEHOPPER.M with the given input arguments.
%
%      INTERACTIVEHOPPER('Property','Value',...) creates a new INTERACTIVEHOPPER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before InteractiveHopper_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to InteractiveHopper_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help InteractiveHopper

% Last Modified by GUIDE v2.5 29-Mar-2017 15:20:31

% Begin initialization code - DO NOT EDIT

% Function info:
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @InteractiveHopper_OpeningFcn, ...
                   'gui_OutputFcn',  @InteractiveHopper_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

function InteractiveHopper_OpeningFcn(hObject, eventdata, handles, varargin)

%%% SET DEFAULTS %%%

% Max height
setField(handles.max_jump_best,0)

% Passive slider value
set(handles.passive_slider,'min',1);
set(handles.passive_slider,'max',100);
set(handles.passive_slider,'Value',50);

% Active slider value
set(handles.active_slider,'min',1);
set(handles.active_slider,'max',100);
set(handles.active_slider,'Value',50);

% Device options disabled
set(findall(handles.choose_assistive_strategy, '-property', 'enable'), 'enable', 'off')

% Label axes
axes(handles.control_axes)
axis([0 5 0 1])
xlabel('Jump Time (s)')
ylabel('Excitation')

% Default muscle: "The Average Joe"
maxIsometricForce = 4000.0;
tendonStiffness = 28.1;
tendonSlackLength = 0.25;
metabolicCostLimit = 5;
setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,metabolicCostLimit)

% Choose default command line output for InteractiveHopper
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

function varargout = InteractiveHopper_OutputFcn(hObject, eventdata, handles) 

% Get default command line output from handles structure
varargout{1} = handles.output;

% SIMULATE - Simulate hopper.
function simulate_Callback(hObject, eventdata, handles)

%%% Muscle %%%% 
if isfield(handles,'muscleExcitation')
    muscleExcitation = handles.muscleExcitation;
else
    muscleExcitation = [0.0 1.99 2.0 3.89 3.9 4.0;
                        0.3 0.3  1.0 1.0  0.1 0.1];
    len = size(muscleExcitation,2)+1;
    muscleExcitation(1,len) = 5;
    muscleExcitation(2,len) = muscleExcitation(2,len-1);
    
    axes(handles.control_axes)
    color = [0.64 0.08 0.18];
    pts = line('Xdata',NaN,'Ydata',NaN,'marker','o','LineWidth',1.5,'Color',color);
    set(pts,'Xdata',muscleExcitation(1,:),'Ydata',muscleExcitation(2,:))
end
maxIsometricForce = handles.max_isometric_force.Value;
tendonStiffness = handles.tendon_stiffness.Value;
tendonSlackLength = handles.tendon_slack_length.Value;
muscleMass = handles.muscle_mass.Value;
MillardTendonParams = [0.049 tendonStiffness 0.67 0.5 tendonSlackLength];

%%% Passive device %%%
addPassiveDevice = handles.enable_passive.Value;
passivePatellaWrap = handles.passive_patella_wrap.Value;
springStiffness = handles.stiffness.Value;
passiveMass = handles.passive_mass.Value;

%%% Active device %%%
addActiveDevice = handles.enable_active.Value;
isActivePropMyo = handles.active_as_prop_myo.Value;
if addActiveDevice
    if isActivePropMyo
        deviceControl = muscleExcitation;
        maxTension = 1000;
        gain = handles.tension_or_gain.Value;
        
    else 
        maxTension = handles.tension_or_gain.Value;
        gain = 1;
        if isfield(handles,'deviceControl')
            deviceControl = handles.deviceControl;
        else
            deviceControl = [0.0 2.5 5.0;
                0.0 0.75 0.0];
            handles.deviceControl = deviceControl;
            guidata(hObject, handles)
            axes(handles.control_axes)
            color = [0 0.45 0.74];
            pts = line('Xdata',NaN,'Ydata',NaN,'marker','o','LineWidth',1.5,'Color',color);
            set(pts,'Xdata',deviceControl(1,:),'Ydata',deviceControl(2,:))
        end
    end
end

activePatellaWrap = handles.active_patella_wrap.Value;
activeMass = handles.active_mass.Value;

% Other GUI, visualization features
visualize = handles.visualize.Value;

axes(handles.results_axes)
results = RunInteractiveHopperSolution('visualize', visualize, ...
             'muscleExcitation', muscleExcitation, ...
             'muscleMass', muscleMass, ...
             'addPassiveDevice', addPassiveDevice, ...
             'passivePatellaWrap', passivePatellaWrap, ...
             'springStiffness', springStiffness, ...
             'passiveMass', passiveMass, ...
             'addActiveDevice', addActiveDevice, ...
             'activePatellaWrap', activePatellaWrap, ...
             'isActivePropMyo', isActivePropMyo, ...
             'maxTension', maxTension, ...
             'gain', gain, ...
             'deviceControl',deviceControl, ...
             'activeMass', activeMass, ...
             'MillardTendonParams', MillardTendonParams, ...
             'maxIsometricForce', maxIsometricForce);
         
maxHeight = max(results.height);
maxHeightBest = get(handles.max_jump_best,'Value');

setField(handles.max_jump_recent,maxHeight(1))

if maxHeight(1) > maxHeightBest
   setField(handles.max_jump_best,maxHeight(1))
end
         
% WITHOUT_DEVICE - If no device, disable "Choose Assistive Strategy" panel
function without_device_Callback(hObject, eventdata, handles)
set(findall(handles.choose_assistive_strategy, '-property', 'enable'), 'enable', 'off')

% WITH_DEVICE - If adding device, enable option to add device
function with_device_Callback(hObject, eventdata, handles)
set(handles.enable_passive,'enable','on')
set(handles.enable_passive,'value',0)
set(handles.enable_active,'enable','on')
set(handles.enable_active,'value',0)

% AVERAGE_JOE - Sets "The Average Joe" muscle.
function average_joe_Callback(hObject, eventdata, handles)
maxIsometricForce = 4000.0;
tendonStiffness = 28.1;
tendonSlackLength = 0.25;
mass = 5;

setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,mass)

% ARNOLD - Sets "The Arnold" muscle.
function arnold_Callback(hObject, eventdata, handles)
maxIsometricForce = 5000.0;
tendonStiffness = 30;
tendonSlackLength = 0.15;
mass = 3.5;

setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,mass)
    
% KATIE_LEDECKY - Sets "The Katie Ledecky" muscle.
function katie_ledecky_Callback(hObject, eventdata, handles)
maxIsometricForce = 3500.0;
tendonStiffness = 25;
tendonSlackLength = 0.30;
mass = 7.5;
    
setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,mass)

% SETMUSCLE - Set a muscle given specific parameters.
function setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,mass)

setField(handles.max_isometric_force,maxIsometricForce)
setField(handles.tendon_stiffness,tendonStiffness)
setField(handles.tendon_slack_length,tendonSlackLength)
setField(handles.muscle_mass,mass)

% NEW_MUSC_EXCITATION - Get new muscle excitation.
function new_musc_excitation_Callback(hObject, eventdata, handles)

handle_names = fieldnames(handles);
reenable = cell(0);
for i = 1:length(handle_names)
    handle = getfield(handles,handle_names{i});
    if isprop(handle,'Callback')
        if strcmp(get(handle,'Enable'),'on')
            reenable{1,length(reenable)+1} = handle;
        end
    end
end

set(handles.new_musc_excitation,'Enable','off')
set(handles.clear,'Enable','off')
set(handles.visualize,'Enable','off')
set(findall(handles.choose_config, '-property', 'enable'), 'enable', 'off')
set(findall(handles.choose_assistive_strategy, '-property', 'enable'), 'enable', 'off')
set(findall(handles.simulate_hopper_model, '-property', 'enable'), 'enable', 'off')

axes(handles.control_axes)
muscleExcitation = getUserActivation([0.64 0.08 0.18]);
handles.muscleExcitation = muscleExcitation;
guidata(hObject, handles)

for i = 1:length(reenable)
    set(reenable{i},'Enable','on')
end

% NEW_DEVICE_CONTROL - Get new device control.
function new_device_control_Callback(hObject, eventdata, handles)
handle_names = fieldnames(handles);
reenable = cell(0);
for i = 1:length(handle_names)
    handle = getfield(handles,handle_names{i});
    if isprop(handle,'Callback')
        if strcmp(get(handle,'Enable'),'on')
            reenable{1,length(reenable)+1} = handle;
        end
    end
end

set(handles.new_musc_excitation,'Enable','off')
set(handles.clear,'Enable','off')
set(handles.visualize,'Enable','off')
set(findall(handles.choose_config, '-property', 'enable'), 'enable', 'off')
set(findall(handles.choose_assistive_strategy, '-property', 'enable'), 'enable', 'off')
set(findall(handles.simulate_hopper_model, '-property', 'enable'), 'enable', 'off')

axes(handles.control_axes)
deviceControl = getUserActivation([0 0.45 0.74]);
handles.deviceControl = deviceControl;
guidata(hObject, handles)

for i = 1:length(reenable)
    set(reenable{i},'Enable','on')
end

% CLEAR - Clear results and control axes.
function clear_Callback(hObject, eventdata, handles)
axes(handles.results_axes)
ax = gca;
cla(ax,'reset');

axes(handles.control_axes)
ax = gca;
cla(ax,'reset');
axis([0 5 0 1])
xlabel('Jump Time (s)')
ylabel('Activation')

% ENABLE_PASSIVE - Add a passive device to the hopper.
function enable_passive_Callback(hObject, eventdata, handles)

handle_names = {'passive_patella_wrap','passive_slider','stiffness_text', ...
                'passive_mass_text','stiffness_units','passive_mass_units'};

if get(hObject,'Value')
    enable(handles,handle_names,true)
    set(handles.passive_slider,'Value',50)
    setPassive(handles,50)
else
    enable(handles,handle_names,false)
    setPassive(handles,[])
end

function active_patella_wrap_Callback(hObject, eventdata, handles)

% PASSIVE_SLIDER - Set spring stiffness with slider.
function passive_slider_Callback(hObject, eventdata, handles)
sliderVal = get(hObject,'Value');
setPassive(handles,sliderVal)

% SETPASSIVE - Given a PASSIVE_SLIDER value, set the spring stiffness
%             and mass
function setPassive(handles,sliderVal)
stiffness = sliderVal*100;
passive_mass = sliderVal*0.1;

setField(handles.stiffness,stiffness)
setField(handles.passive_mass,passive_mass)

% SETFIELD - Set the value and string of a field given a value
function setField(handle,value)
if isempty(value)
    set(handle,'String',' ')
    set(handle,'Value', 0)
else
    set(handle,'String',num2str(value))
    set(handle,'Value', value)
end

% ENABLE - Given a list of handle names, enable or disable figure elements
%          based on true or false flag.
function enable(handles,handle_names,trueOrFalse)

for i = 1:length(handle_names)
    handle = getfield(handles,handle_names{i});
    
    if trueOrFalse
        set(handle,'Enable','on')
    else
        set(handle,'Enable','off')
    end
end

% ENABLE_ACTIVE - Add an active device to the hopper.
function enable_active_Callback(hObject, eventdata, handles)

handle_names = {'active_patella_wrap','active_as_prop_myo','new_device_control', ...
                'active_mass_text','active_mass_units','tension_or_gain_text', ...
                'tension_or_gain_units','active_slider'};

if get(hObject,'Value')
    enable(handles,handle_names,true)
    set(handles.active_slider,'Value',50)
    setActive(handles,50)
else
    enable(handles,handle_names,false)
    setActive(handles,[])
end

% ACTIVE_SLIDER
function active_slider_Callback(hObject, eventdata, handles)
sliderVal = get(hObject,'Value');
setActive(handles,sliderVal)

% SETACTIVE - Set Max Tension or Gain and Mass for active device based on
%             slider input from user.
function setActive(handles,sliderVal)

switch get(handles.tension_or_gain_text,'String')
    case 'Max Tension'
        tension_or_gain = sliderVal*10;
    case 'Gain'
        tension_or_gain = (sliderVal/100)+0.5;
end
active_mass = sliderVal*0.1;

setField(handles.tension_or_gain,tension_or_gain)
setField(handles.active_mass,active_mass)

% ACTIVE_AS_PROP_MYO - Specify a proportional myoelectric controller for
%                      the active device
function active_as_prop_myo_Callback(hObject, eventdata, handles)

handle_names = {'new_device_control'};
sliderVal = handles.active_slider.Value;

if get(hObject,'Value')
    enable(handles,handle_names,false)
    setField(handles.tension_or_gain,1)
    set(handles.tension_or_gain_text,'String','Gain')
    setField(handles.tension_or_gain_units,[])
    setActive(handles,sliderVal)
else
    enable(handles,handle_names,true)
    setField(handles.tension_or_gain,100)
    set(handles.tension_or_gain_text,'String','Max Tension')
    set(handles.tension_or_gain_units,'String','N-m')
    setActive(handles,sliderVal)
end

%% Currently unused callback and create functions
function max_jump_recent_Callback(hObject, eventdata, handles)

function max_jump_recent_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function max_jump_best_Callback(hObject, eventdata, handles)

function max_jump_best_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function control_axes_CreateFcn(hObject, eventdata, handles)

function passive_patella_wrap_Callback(hObject, eventdata, handles)

function passive_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function visualize_Callback(hObject, eventdata, handles)

function results_axes_CreateFcn(hObject, eventdata, handles)

function max_isometric_force_Callback(hObject, eventdata, handles)
 
function active_slider_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function max_isometric_force_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tendon_stiffness_Callback(hObject, eventdata, handles)

function tendon_stiffness_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function stiffness_Callback(hObject, eventdata, handles)

function stiffness_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function passive_mass_Callback(hObject, eventdata, handles)

function passive_mass_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tendon_slack_length_Callback(hObject, eventdata, handles)

function tendon_slack_length_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function muscle_mass_Callback(hObject, eventdata, handles)

function muscle_mass_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tension_or_gain_Callback(hObject, eventdata, handles)

function tension_or_gain_text_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function device_cost_limit_Callback(hObject, eventdata, handles)

function device_cost_limit_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function active_mass_Callback(hObject, eventdata, handles)

function active_mass_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function reset_Callback(hObject, eventdata, handles)

function setup_Callback(hObject, eventdata, handles)

function setup_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function save_setup_Callback(hObject, eventdata, handles)

function load_setup_Callback(hObject, eventdata, handles)

function tension_or_gain_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
