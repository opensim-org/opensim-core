function varargout = HopperExample(varargin)
% HOPPEREXAMPLE MATLAB code for HopperExample.fig
%      HOPPEREXAMPLE, by itself, creates a new HOPPEREXAMPLE or raises the existing
%      singleton*.
%
%      H = HOPPEREXAMPLE returns the handle to a new HOPPEREXAMPLE or the handle to
%      the existing singleton*.
%
%      HOPPEREXAMPLE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HOPPEREXAMPLE.M with the given input arguments.
%
%      HOPPEREXAMPLE('Property','Value',...) creates a new HOPPEREXAMPLE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before HopperExample_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to HopperExample_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help HopperExample

% Last Modified by GUIDE v2.5 20-Mar-2017 10:37:49

% Begin initialization code - DO NOT EDIT

% Function info:
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @HopperExample_OpeningFcn, ...
                   'gui_OutputFcn',  @HopperExample_OutputFcn, ...
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

function HopperExample_OpeningFcn(hObject, eventdata, handles, varargin)

%%% SET DEFAULTS %%%

% Spring stiffness
set(handles.spring_stiffness,'min',1);
set(handles.spring_stiffness,'max',25000);
set(handles.spring_stiffness,'Value',100);

% Device options disabled
set(findall(handles.choose_assistive_strategy, '-property', 'enable'), 'enable', 'off')

% Label axes
axes(handles.activation_axes)
axis([0 5 0 1])
xlabel('Jump Time (s)')
ylabel('Activation')

% Choose default command line output for HopperExample
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

function varargout = HopperExample_OutputFcn(hObject, eventdata, handles) 

% Get default command line output from handles structure
varargout{1} = handles.output;

% SIMULATE - Simulate hopper.
function simulate_Callback(hObject, eventdata, handles)

muscleActivation = handles.muscleActivation;
addPassiveDevice = handles.enable_passive.Value;
passivePatellaWrap = handles.passive_patella_wrap.Value;
springStiffness = handles.spring_stiffness.Value;
addActiveDevice = handles.enable_active.Value;
activePatellaWrap = handles.active_patella_wrap.Value;
isActivePropMyo = handles.active_as_prop_myo.Value;
if isActivePropMyo || ~addActiveDevice
    deviceActivation = [0.0 2.0 3.9;
                        0.3 1.0 0.1];
else
    deviceActivation = handles.deviceActivation;
end
visualize = handles.visualize.Value;

axes(handles.results_axes)
RunHopperGUI('visualize', visualize, ...
             'muscleActivation', muscleActivation, ...
             'addPassiveDevice', addPassiveDevice, ...
             'passivePatellaWrap', passivePatellaWrap, ...
             'springStiffness', springStiffness, ...
             'addActiveDevice', addActiveDevice, ...
             'activePatellaWrap', activePatellaWrap, ...
             'isActivePropMyo', isActivePropMyo, ...
             'deviceActivation',deviceActivation)
         
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
    set(handles.spring_stiffness,'Enable','on')
    set(handles.spring_stiffness_text,'Enable','on')
else
    set(handles.passive_patella_wrap,'Enable','off')
    set(handles.spring_stiffness,'Enable','off')
    set(handles.spring_stiffness_text,'Enable','off')
end

function active_patella_wrap_Callback(hObject, eventdata, handles)

% SPRING_STIFFNESS - Set spring stiffness with slider.
function spring_stiffness_Callback(hObject, eventdata, handles)


% ENABLE_ACTIVE - Add an active device to the hopper.
function enable_active_Callback(hObject, eventdata, handles)

if get(hObject,'Value')
    set(handles.active_patella_wrap,'Enable','on')
    set(handles.active_as_prop_myo,'Enable','on')
    set(handles.new_device_act,'Enable','on')
else
    set(handles.active_patella_wrap,'Enable','off')
    set(handles.active_as_prop_myo,'Enable','off')
    set(handles.new_device_act,'Enable','off')
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
function spring_stiffness_CreateFcn(hObject, eventdata, handles)
% hObject    handle to spring_stiffness (see GCBO)
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
