function varargout = tgcs2017(varargin)
% TGCS2017 MATLAB code for tgcs2017.fig
%      TGCS2017, by itself, creates a new TGCS2017 or raises the existing
%      singleton*.
%
%      H = TGCS2017 returns the handle to a new TGCS2017 or the handle to
%      the existing singleton*.
%
%      TGCS2017('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TGCS2017.M with the given input arguments.
%
%      TGCS2017('Property','Value',...) creates a new TGCS2017 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before tgcs2017_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to tgcs2017_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help tgcs2017

% Last Modified by GUIDE v2.5 28-Feb-2017 11:35:39

% Begin initialization code - DO NOT EDIT

% Function info:
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @tgcs2017_OpeningFcn, ...
                   'gui_OutputFcn',  @tgcs2017_OutputFcn, ...
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

function tgcs2017_OpeningFcn(hObject, eventdata, handles, varargin)

axis([0 5 0 1])
xlabel('Jump Time (s)')
ylabel('Activation')
% Choose default command line output for tgcs2017
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

function varargout = tgcs2017_OutputFcn(hObject, eventdata, handles) 

% Get default command line output from handles structure
varargout{1} = handles.output;

% NEW_MUSC_ACT - Plot new activation.
function new_musc_act_Callback(hObject, eventdata, handles)

names = {'clear','new_musc_act'};

for n = 1:length(names)
    set(handles.(names{n}),'Enable','off')
end

user_act = get_user_act();
assignin('base','user_act',user_act)

for n = 1:length(names)
    set(handles.(names{n}),'Enable','on')
end

% TODO: set toggle for hopper with/without device 
RunHopper_answers

% CLEAR - Reset axes to clear plotted activations.
function clear_Callback(hObject, eventdata, handles)
ax = gca;
cla(ax,'reset');
axis([0 5 0 1])
xlabel('Jump Time (s)')
ylabel('Activation')


% --- Executes on button press in without_device.
function without_device_Callback(hObject, eventdata, handles)
% hObject    handle to without_device (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of without_device


% --- Executes on button press in radio_with_device.
function radio_with_device_Callback(hObject, eventdata, handles)
% hObject    handle to radio_with_device (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radio_with_device


% --- Executes on button press in enable_passive.
function checkbox_passive_enable_Callback(hObject, eventdata, handles)
% hObject    handle to enable_passive (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_passive



function cost_recent_Callback(hObject, eventdata, handles)
% hObject    handle to cost_recent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cost_recent as text
%        str2double(get(hObject,'String')) returns contents of cost_recent as a double


% --- Executes during object creation, after setting all properties.
function cost_recent_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cost_recent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cost_best_Callback(hObject, eventdata, handles)
% hObject    handle to cost_best (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cost_best as text
%        str2double(get(hObject,'String')) returns contents of cost_best as a double


% --- Executes during object creation, after setting all properties.
function cost_best_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cost_best (see GCBO)
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
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on button press in enable_passive.
function enable_passive_Callback(hObject, eventdata, handles)
% hObject    handle to enable_passive (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of enable_passive


% --- Executes on button press in active_patella_wrap.
function active_patella_wrap_Callback(hObject, eventdata, handles)
% hObject    handle to active_patella_wrap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of active_patella_wrap


% --- Executes on button press in active_as_prop_myo.
function active_as_prop_myo_Callback(hObject, eventdata, handles)
% hObject    handle to active_as_prop_myo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of active_as_prop_myo


% --- Executes on button press in passive_patella_wrap.
function passive_patella_wrap_Callback(hObject, eventdata, handles)
% hObject    handle to passive_patella_wrap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of passive_patella_wrap


% --- Executes on slider movement.
function spring_stiffness_Callback(hObject, eventdata, handles)
% hObject    handle to spring_stiffness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function spring_stiffness_CreateFcn(hObject, eventdata, handles)
% hObject    handle to spring_stiffness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in simulate.
function simulate_Callback(hObject, eventdata, handles)
% hObject    handle to simulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in new_device_act.
function new_device_act_Callback(hObject, eventdata, handles)
% hObject    handle to new_device_act (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
