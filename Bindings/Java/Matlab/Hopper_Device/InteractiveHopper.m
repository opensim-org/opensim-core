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

% INTERACTIVEHOPPER_OPENINGFCN
function InteractiveHopper_OpeningFcn(hObject, eventdata, handles, varargin)

% Group all handles for enabling/disabling
handles.all = [handles.new_musc_excitation handles.muscles handles.choose_config ...
               handles.choose_assistive_strategy handles.simulate_hopper_model ...
               handles.setup_file handles.clear handles.reset];

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
averageJoe = InteractiveHopperSettings('averageJoe');
[maxIsometricForce,tendonStiffness,tendonSlackLength,mass] = averageJoe();
setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,mass)

% Choose default command line output for InteractiveHopper
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% INTERACTIVEHOPPER_OUTPUTFCN
function varargout = InteractiveHopper_OutputFcn(hObject, eventdata, handles) 

% Get default command line output from handles structure
varargout{1} = handles.output;

% SIMULATE - Simulate hopper.
function simulate_Callback(hObject, eventdata, handles)

% Muscle
muscle = handles.muscle;
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


% Passive device
addPassiveDevice = handles.enable_passive.Value;
passivePatellaWrap = handles.passive_patella_wrap.Value;
passiveParameter = handles.passive_slider.Value;

% Active device 
addActiveDevice = handles.enable_active.Value;
activePatellaWrap = handles.active_patella_wrap.Value;
activeParameter = handles.active_slider.Value;
isActivePropMyo = handles.active_as_prop_myo.Value;
if addActiveDevice
    if isActivePropMyo
        deviceControl = muscleExcitation;
    else 
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
else
    deviceControl = [0.0 2.5 5.0;
                     0.0 0.75 0.0];
end

% Other GUI, visualization features
visualize = handles.visualize.Value;

% Build or modify hopper solution
%if ~exist('hopper','var')
    hopper = BuildInteractiveHopperSolution('visualize', visualize, ...
             'muscle', muscle, ...
             'muscleExcitation', muscleExcitation, ...
             'addPassiveDevice', addPassiveDevice, ...
             'passivePatellaWrap', passivePatellaWrap, ...
             'passiveParameter', passiveParameter, ...
             'addActiveDevice', addActiveDevice, ...
             'activePatellaWrap', activePatellaWrap, ...
             'isActivePropMyo', isActivePropMyo, ...
             'activeParameter', activeParameter, ...
             'deviceControl',deviceControl);
% else
%    hopper = ModifyInteractiveHopperSolution('visualize', visualize, ...
%              'muscle', muscle, ...
%              'muscleExcitation', muscleExcitation, ...
%              'addPassiveDevice', addPassiveDevice, ...
%              'passivePatellaWrap', passivePatellaWrap, ...
%              'passiveParameter', passiveParameter, ...
%              'addActiveDevice', addActiveDevice, ...
%              'activePatellaWrap', activePatellaWrap, ...
%              'isActivePropMyo', isActivePropMyo, ...
%              'activeParameter', activeParameter, ...
%              'deviceControl',deviceControl);    
% end
         

% Create table reporter and add hop height and vastus activation to report
reporter = TableReporter();
reporter.setName('hopper_device_results');
reporter.set_report_time_interval(0.2); % seconds.
reporter.addToReport(...
    hopper.getComponent('slider/yCoord').getOutput('value'), 'height');
reporter.addToReport(...
    hopper.getComponent('vastus').getOutput('activation'))
hopper.addComponent(reporter);

% Initialize system
sHD = hopper.initSystem();

% Simulate hopper
Simulate(hopper, sHD, visualize);

% Get table reporter
if exist('reporter') == 1
    table = reporter.getTable();
    results = opensimTimeSeriesTableToMatlab(table);
end

% Update max jump height value
maxHeight = max(results.height);
maxHeightBest = get(handles.max_jump_best,'Value');

setField(handles.max_jump_recent,maxHeight(1))

if maxHeight(1) > maxHeightBest
   setField(handles.max_jump_best,maxHeight(1))
end

% Plot results
axes(handles.results_axes)
if isfield(results, 'height')
    plot(results.time, results.height);
    xlabel('time');
    ylabel('height');
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

muscle = 'averageJoe';
handles.muscle = muscle;
guidata(hObject, handles)
averageJoe = InteractiveHopperSettings(muscle);
[maxIsometricForce,tendonStiffness,tendonSlackLength,mass] = averageJoe();
setMuscle(handles, maxIsometricForce, tendonStiffness, tendonSlackLength, mass)

% ARNOLD - Sets "The Arnold" muscle.
function arnold_Callback(hObject, eventdata, handles)

muscle = 'arnold';
handles.muscle = muscle;
guidata(hObject, handles)
arnold = InteractiveHopperSettings(muscle);
[maxIsometricForce,tendonStiffness,tendonSlackLength,mass] = arnold();
setMuscle(handles, maxIsometricForce, tendonStiffness, tendonSlackLength, mass)
    
% KATIE_LEDECKY - Sets "The Katie Ledecky" muscle.
function katie_ledecky_Callback(hObject, eventdata, handles)

muscle = 'katieLedecky';
handles.muscle = muscle;
guidata(hObject, handles)
katieLedecky = InteractiveHopperSettings(muscle);
[maxIsometricForce,tendonStiffness,tendonSlackLength,mass] = katieLedecky();
setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,mass)

% SETMUSCLE - Set a muscle given specific parameters.
function setMuscle(handles, maxIsometricForce, tendonStiffness, tendonSlackLength, mass)

setField(handles.max_isometric_force, maxIsometricForce)
setField(handles.tendon_stiffness, tendonStiffness)
setField(handles.tendon_slack_length, tendonSlackLength)
setField(handles.muscle_mass, mass)

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

set(findall(handles.all, '-property', 'enable'), 'enable', 'off')

axes(handles.control_axes)
muscleExcitation = getUserControl([0.64 0.08 0.18]);
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

set(findall(handles.all, '-property', 'enable'), 'enable', 'off')

axes(handles.control_axes)
deviceControl = getUserControl([0 0.45 0.74]);
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

passiveParameter = get(hObject,'Value');
setPassive(handles,passiveParameter)

% SETPASSIVE - Given a PASSIVE_SLIDER value, set the spring stiffness
%             and mass
function setPassive(handles,passiveParameter)

passive = InteractiveHopperSettings('passive');
[passive_mass,stiffness] = passive(passiveParameter);

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

activeParameter = get(hObject,'Value');
setActive(handles,activeParameter)

% SETACTIVE - Set Max Tension or Gain and Mass for active device based on
%             slider input from user.
function setActive(handles,activeParameter)

if handles.active_as_prop_myo.Value
    activePropMyo = InteractiveHopperSettings('activePropMyo');
    [active_mass,tension_or_gain] = activePropMyo(activeParameter);
else
    activeControl = InteractiveHopperSettings('activeControl');
    [active_mass,tension_or_gain] = activeControl(activeParameter);
end

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
    set(handles.tension_or_gain_units,'String','N')
    setActive(handles,sliderVal)
end

% GETUSERCONTROL
function [xy] = getUserControl(color)

w = [0 5 0 1];
axis(w), hold on, grid on

title(getString(message('SPLINES:resources:axesLabel_WhenYouAreDone')))
pts = line('Xdata',NaN,'Ydata',NaN,'marker','o','LineWidth',1.5,'Color',color);
pts2 = line('Xdata',NaN,'Ydata',NaN,'LineStyle','--','LineWidth',1.5,'Color',color);

maxpnts = 100; xy = zeros(2,maxpnts);
while 1
    for j=2:maxpnts
        try
            [x,y] = ginput(1);
        catch ME
            display('ERROR: ginput failed')
        end
        if isempty(x)||x<w(1)||x>w(2)||y<w(3)||y>w(4)
            xy(1,j) = 5;
            xy(2,j) = xy(2,j-1);
            
            set(pts,'Xdata',xy(1,1:j),'Ydata',xy(2,1:j))
            break;
        end
        xy(:,j) = [x;y];
        if j>1
            set(pts,'Xdata',xy(1,1:j),'Ydata',xy(2,1:j))
            set(pts2,'Xdata',[xy(1,j) 5],'Ydata',[xy(2,j) xy(2,j)])
        else
            set(pts,'Xdata',x,'Ydata',y)
        end
    end
    
    if j>1, break, end
    
end
title(' ')
xy(:,j) = [5;xy(2,j-1)];
xy(:,j+1:maxpnts)=[];

% Currently unused callback and create functions - DO NOT DELETE
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
