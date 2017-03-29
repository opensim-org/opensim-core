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

% Last Modified by GUIDE v2.5 29-Mar-2017 09:21:07

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

% Spring stiffness
set(handles.passive_slider,'min',1);
set(handles.passive_slider,'max',100);
set(handles.passive_slider,'Value',50);

% Device options disabled
set(findall(handles.choose_assistive_strategy, '-property', 'enable'), 'enable', 'off')

% Label axes
axes(handles.activation_axes)
axis([0 5 0 1])
xlabel('Jump Time (s)')
ylabel('Excitation')

% Default muscle: "The Average Joe"
maxIsometricForce = 4000.0;
tendonStiffness = 28.1;
tendonSlackLength = 0.25;
metabolicCostLimit = 5;
setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,metabolicCostLimit)

% Default spring setting
setSpring(handles,50)

% Choose default command line output for InteractiveHopper
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

function varargout = InteractiveHopper_OutputFcn(hObject, eventdata, handles) 

% Get default command line output from handles structure
varargout{1} = handles.output;

% SIMULATE - Simulate hopper.
function simulate_Callback(hObject, eventdata, handles)

% Muscle activation
if isfield(handles,'muscleActivation')
    muscleActivation = handles.muscleActivation;
else
    muscleActivation = [0.0 1.99 2.0 3.89 3.9 4.0;
                        0.3 0.3  1.0 1.0  0.1 0.1];
    len = size(muscleActivation,2)+1;
    muscleActivation(1,len) = 5;
    muscleActivation(2,len) = muscleActivation(2,len-1);
    
    axes(handles.activation_axes)
    color = [0.64 0.08 0.18];
    pts = line('Xdata',NaN,'Ydata',NaN,'marker','o','LineWidth',1.5,'Color',color);
    set(pts,'Xdata',muscleActivation(1,:),'Ydata',muscleActivation(2,:))
end
addPassiveDevice = handles.enable_passive.Value;
passivePatellaWrap = handles.passive_patella_wrap.Value;
springStiffness = handles.stiffness.Value;
addActiveDevice = handles.enable_active.Value;
activePatellaWrap = handles.active_patella_wrap.Value;
isActivePropMyo = handles.active_as_prop_myo.Value;
if isActivePropMyo || ~addActiveDevice
    deviceActivation = muscleActivation;               
else
    deviceActivation = handles.deviceActivation;
end
visualize = handles.visualize.Value;

% Adjust muscle model parameters based on user selection
maxIsometricForce = handles.max_isometric_force.Value;
tendonStiffness = handles.tendon_stiffness.Value;
tendonSlackLength = handles.tendon_slack_length.Value;
metabolicCostLimit = handles.metabolic_cost_limit.Value;

MillardTendonParams = [0.049 tendonStiffness 0.67 0.5 tendonSlackLength];

% Get device masses
passiveMass = handles.passive_mass.Value;
%activeMass = handles.active_mass.Value;

axes(handles.results_axes)
RunInteractiveHopperSolution('visualize', visualize, ...
             'muscleActivation', muscleActivation, ...
             'addPassiveDevice', addPassiveDevice, ...
             'passivePatellaWrap', passivePatellaWrap, ...
             'springStiffness', springStiffness, ...
             'passiveMass', passiveMass, ...
             'addActiveDevice', addActiveDevice, ...
             'activePatellaWrap', activePatellaWrap, ...
             'isActivePropMyo', isActivePropMyo, ...
             'deviceActivation',deviceActivation, ...
             'MillardTendonParams', MillardTendonParams, ...
             'maxIsometricForce', maxIsometricForce);
         
% WITHOUT_DEVICE - If no device, disable "Choose Assistive Strategy" panel
function without_device_Callback(hObject, eventdata, handles)
set(findall(handles.choose_assistive_strategy, '-property', 'enable'), 'enable', 'off')

% WITH_DEVICE - If adding device, enable option to add device
function with_device_Callback(hObject, eventdata, handles)
set(handles.enable_passive,'enable','on')
set(handles.enable_passive,'value',0)
set(handles.enable_active,'enable','on')
set(handles.enable_active,'value',0)

% NEW_MUSC_ACT - Get new muscle activation.
function new_musc_act_Callback(hObject, eventdata, handles)

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

set(handles.new_musc_act,'Enable','off')
set(handles.clear,'Enable','off')
set(handles.visualize,'Enable','off')
set(findall(handles.choose_config, '-property', 'enable'), 'enable', 'off')
set(findall(handles.choose_assistive_strategy, '-property', 'enable'), 'enable', 'off')
set(findall(handles.simulate_hopper_model, '-property', 'enable'), 'enable', 'off')

axes(handles.activation_axes)
muscleActivation = getUserActivation([0.64 0.08 0.18]);
handles.muscleActivation = muscleActivation;
guidata(hObject, handles)

for i = 1:length(reenable)
    set(reenable{i},'Enable','on')
end

% NEW_DEVICE_ACT - Get new device activation.
function new_device_act_Callback(hObject, eventdata, handles)
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

set(handles.new_musc_act,'Enable','off')
set(handles.clear,'Enable','off')
set(handles.visualize,'Enable','off')
set(findall(handles.choose_config, '-property', 'enable'), 'enable', 'off')
set(findall(handles.choose_assistive_strategy, '-property', 'enable'), 'enable', 'off')
set(findall(handles.simulate_hopper_model, '-property', 'enable'), 'enable', 'off')

axes(handles.activation_axes)
deviceActivation = getUserActivation([0 0.45 0.74]);
handles.deviceActivation = deviceActivation;
guidata(hObject, handles)

for i = 1:length(reenable)
    set(reenable{i},'Enable','on')
end

% CLEAR - Reset axes to clear plotted activations.
function clear_Callback(hObject, eventdata, handles)
axes(handles.activation_axes)
ax = gca;
cla(ax,'reset');
axis([0 5 0 1])
xlabel('Jump Time (s)')
ylabel('Activation')

% ENABLE_PASSIVE - Add a passive device to the hopper.
function enable_passive_Callback(hObject, eventdata, handles)

if get(hObject,'Value')
    set(handles.passive_patella_wrap,'Enable','on')
    set(handles.passive_slider,'Enable','on')
    set(handles.stiffness_text,'Enable','on')
    set(handles.passive_mass_text,'Enable','on')
else
    set(handles.passive_patella_wrap,'Enable','off')
    set(handles.passive_slider,'Enable','off')
    set(handles.stiffness_text,'Enable','off')
    set(handles.passive_mass_text,'Enable','off')
end

function active_patella_wrap_Callback(hObject, eventdata, handles)

% PASSIVE_SLIDER - Set spring stiffness with slider.
function passive_slider_Callback(hObject, eventdata, handles)
sliderVal = get(hObject,'Value');
setSpring(handles,sliderVal)

function setSpring(handles,sliderVal)
set(handles.stiffness,'Value', sliderVal * 100)
set(handles.stiffness,'String',num2str(handles.stiffness.Value))
set(handles.passive_mass,'Value', sliderVal * 0.1)
set(handles.passive_mass,'String',num2str(handles.passive_mass.Value))

% ENABLE_ACTIVE - Add an active device to the hopper.
function enable_active_Callback(hObject, eventdata, handles)

if get(hObject,'Value')
    set(handles.active_patella_wrap,'Enable','on')
    set(handles.active_as_prop_myo,'Enable','on')
    set(handles.new_device_act,'Enable','on')
    set(handles.optimal_force_text,'Enable','on')
    set(handles.active_mass_text,'Enable','on')
    set(handles.max_torque,'Enable','on')
else
    set(handles.active_patella_wrap,'Enable','off')
    set(handles.active_as_prop_myo,'Enable','off')
    set(handles.new_device_act,'Enable','off')
    set(handles.optimal_force_text,'Enable','off')
    set(handles.active_mass_text,'Enable','off')
    set(handles.max_torque,'Enable','off')
end

% ACTIVE_AS_PROP_MYO - Specify a proportional myoelectric controller for
%                      the active device
function active_as_prop_myo_Callback(hObject, eventdata, handles)

if get(hObject,'Value')
    set(handles.new_device_act,'Enable','off')
else
    set(handles.new_device_act,'Enable','on')
end

function metabolic_cost_Callback(hObject, eventdata, handles)
% hObject    handle to metabolic_cost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of metabolic_cost as text
%        str2double(get(hObject,'String')) returns contents of metabolic_cost as a double


% --- Executes during object creation, after setting all properties.
function metabolic_cost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to metabolic_cost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function device_cost_Callback(hObject, eventdata, handles)
% hObject    handle to device_cost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of device_cost as text
%        str2double(get(hObject,'String')) returns contents of device_cost as a double


% --- Executes during object creation, after setting all properties.
function device_cost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to device_cost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function max_jump_recent_Callback(hObject, eventdata, handles)
% hObject    handle to max_jump_recent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of max_jump_recent as text
%        str2double(get(hObject,'String')) returns contents of max_jump_recent as a double


% --- Executes during object creation, after setting all properties.
function max_jump_recent_CreateFcn(hObject, eventdata, handles)
% hObject    handle to max_jump_recent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function max_jump_best_Callback(hObject, eventdata, handles)
% hObject    handle to max_jump_best (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of max_jump_best as text
%        str2double(get(hObject,'String')) returns contents of max_jump_best as a double


% --- Executes during object creation, after setting all properties.
function max_jump_best_CreateFcn(hObject, eventdata, handles)
% hObject    handle to max_jump_best (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function activation_axes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to activation_axes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate activation_axes











% --- Executes on button press in passive_patella_wrap.
function passive_patella_wrap_Callback(hObject, eventdata, handles)
% hObject    handle to passive_patella_wrap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of passive_patella_wrap





% --- Executes during object creation, after setting all properties.
function passive_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to passive_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




function visualize_Callback(hObject, eventdata, handles)

% handles.visualize = get(hObject,'Value');
% guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function results_axes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to results_axes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate results_axes



% Hint: get(hObject,'Value') returns toggle state of enable_passive





function max_isometric_force_Callback(hObject, eventdata, handles)
% hObject    handle to max_isometric_force (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of max_isometric_force as text
%        str2double(get(hObject,'String')) returns contents of max_isometric_force as a double


% --- Executes during object creation, after setting all properties.
function max_isometric_force_CreateFcn(hObject, eventdata, handles)
% hObject    handle to max_isometric_force (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tendon_stiffness_Callback(hObject, eventdata, handles)
% hObject    handle to tendon_stiffness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tendon_stiffness as text
%        str2double(get(hObject,'String')) returns contents of tendon_stiffness as a double


% --- Executes during object creation, after setting all properties.
function tendon_stiffness_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tendon_stiffness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTE

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function stiffness_Callback(hObject, eventdata, handles)
% hObject    handle to stiffness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of stiffness as text
%        str2double(get(hObject,'String')) returns contents of stiffness as a double


% --- Executes during object creation, after setting all properties.
function stiffness_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stiffness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function passive_mass_Callback(hObject, eventdata, handles)
% hObject    handle to passive_mass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of passive_mass as text
%        str2double(get(hObject,'String')) returns contents of passive_mass as a double


% --- Executes during object creation, after setting all properties.
function passive_mass_CreateFcn(hObject, eventdata, handles)
% hObject    handle to passive_mass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tendon_slack_length_Callback(hObject, eventdata, handles)
% hObject    handle to tendon_slack_length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tendon_slack_length as text
%        str2double(get(hObject,'String')) returns contents of tendon_slack_length as a double


% --- Executes during object creation, after setting all properties.
function tendon_slack_length_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tendon_slack_length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function metabolic_cost_limit_Callback(hObject, eventdata, handles)
% hObject    handle to metabolic_cost_limit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of metabolic_cost_limit as text
%        str2double(get(hObject,'String')) returns contents of metabolic_cost_limit as a double


% --- Executes during object creation, after setting all properties.
function metabolic_cost_limit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to metabolic_cost_limit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function optimal_force_Callback(hObject, eventdata, handles)
% hObject    handle to optimal_force (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of optimal_force as text
%        str2double(get(hObject,'String')) returns contents of optimal_force as a double


% --- Executes during object creation, after setting all properties.
function optimal_force_CreateFcn(hObject, eventdata, handles)
% hObject    handle to optimal_force (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function device_cost_limit_Callback(hObject, eventdata, handles)
% hObject    handle to device_cost_limit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of device_cost_limit as text
%        str2double(get(hObject,'String')) returns contents of device_cost_limit as a double


% --- Executes during object creation, after setting all properties.
function device_cost_limit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to device_cost_limit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function active_mass_Callback(hObject, eventdata, handles)
% hObject    handle to active_mass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of active_mass as text
%        str2double(get(hObject,'String')) returns contents of active_mass as a double


% --- Executes during object creation, after setting all properties.
function active_mass_CreateFcn(hObject, eventdata, handles)
% hObject    handle to active_mass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function average_joe_Callback(hObject, eventdata, handles)
maxIsometricForce = 4000.0;
tendonStiffness = 28.1;
tendonSlackLength = 0.25;
metabolicCostLimit = 5;

setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,metabolicCostLimit)

% --- Executes on button press in arnold.
function arnold_Callback(hObject, eventdata, handles)
maxIsometricForce = 5000.0;
tendonStiffness = 30;
tendonSlackLength = 0.15;
metabolicCostLimit = 3.5;

setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,metabolicCostLimit)
    
% --- Executes on button press in katie_ledecky.
function katie_ledecky_Callback(hObject, eventdata, handles)
maxIsometricForce = 3500.0;
tendonStiffness = 25;
tendonSlackLength = 0.30;
metabolicCostLimit = 7.5;
    
setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,metabolicCostLimit)

function setMuscle(handles, maxIsometricForce,tendonStiffness,tendonSlackLength,metabolicCostLimit)

set(handles.max_isometric_force,'Value',maxIsometricForce)
set(handles.tendon_stiffness,'Value',tendonStiffness)
set(handles.tendon_slack_length,'Value',tendonSlackLength)
set(handles.metabolic_cost_limit,'Value',metabolicCostLimit)
set(handles.max_isometric_force,'String',num2str(maxIsometricForce))
set(handles.tendon_stiffness,'String',num2str(tendonStiffness))
set(handles.tendon_slack_length,'String',num2str(tendonSlackLength))
set(handles.metabolic_cost_limit,'String',num2str(metabolicCostLimit))


% --- Executes on slider movement.
function active_slider_Callback(hObject, eventdata, handles)
% hObject    handle to active_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function active_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to active_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
