function varargout = muscleAnalysisGUI(varargin)
% MUSCLEANALYSISGUI MATLAB code for muscleAnalysisGUI.fig
%      MUSCLEANALYSISGUI, by itself, creates a new MUSCLEANALYSISGUI or raises the existing
%      singleton*.
%
%      H = MUSCLEANALYSISGUI returns the handle to a new MUSCLEANALYSISGUI or the handle to
%      the existing singleton*.
%
%      MUSCLEANALYSISGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MUSCLEANALYSISGUI.M with the given input arguments.
%
%      MUSCLEANALYSISGUI('Property','Value',...) creates a new MUSCLEANALYSISGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before muscleAnalysisGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to muscleAnalysisGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help muscleAnalysisGUI

% Last Modified by GUIDE v2.5 19-Dec-2016 16:57:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @muscleAnalysisGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @muscleAnalysisGUI_OutputFcn, ...
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


% --- Executes just before muscleAnalysisGUI is made visible.
function muscleAnalysisGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to muscleAnalysisGUI (see VARARGIN)

% Choose default command line output for muscleAnalysisGUI
handles.output = hObject;

guidata(hObject, handles);

% UIWAIT makes muscleAnalysisGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

import org.opensim.modeling.*

% initialize muscle list
model = Model(varargin{1});
musclesSet = model.getMuscles();
numMuscles = musclesSet.getSize();
musclesArray = cell(numMuscles, 1);
for indMuscle = 1:numMuscles
    musclesArray(indMuscle) = musclesSet.get(indMuscle-1).getName();
end
set(handles.listbox1, 'String', musclesArray);

% initialize output list
thisMusc = musclesSet.get(0);
outputs = thisMusc.getOutputNames();
numOutputs = outputs.size();
outputsArray = cell(numOutputs, 1);
for indOutput = 1:numOutputs
   outputsArray(indOutput) = outputs(1).get(indOutput-1);
end
set(handles.listbox2, 'String', outputsArray);

% pass model and states to handles structure
handles.model = model;
statesStorage = Storage(varargin{2});
statesTrajectory = StatesTrajectory.createFromStatesStorage(model, statesStorage);
handles.stateTrajectory = statesTrajectory;

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = muscleAnalysisGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% allow multiple selection
set(hObject,'Max',2,'Min',0);

% --- Executes on selection change in listbox2.
function listbox2_Callback(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox2


% --- Executes during object creation, after setting all properties.
function listbox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% allow multiple selection
set(hObject,'Max',2,'Min',0);

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

muscleInds = handles.listbox1.Value;
musclesToAnalyze = handles.listbox1.String(muscleInds);

outputInds = handles.listbox2.Value;
outputsToAnalyze = handles.listbox2.String(outputInds);

muscleAnalysisHelper(musclesToAnalyze, outputsToAnalyze, handles);

function muscleAnalysisHelper(muscles, outputs, handles)
import org.opensim.modeling.*
model = handles.model;
statesTrajectory = handles.stateTrajectory;

numStates = statesTrajectory.getSize();
numMuscles = numel(muscles);
numOutputs = numel(outputs);

reporter = TableReporter();
% Loop over each muscle
for indMuscle = 1:numMuscles
    % Loop over every output
    for indOutput = 1:numOutputs
        muscleName = muscles(indMuscle);
        outputName = outputs(indOutput);
        % no form of "getOutputValue<T>" in scripting, use a reporter
        % instead
        %outputsTemp(indOutput, model.getComponent(muscleName).getOutput(outputName));
        thisMusc = model.getComponent(muscleName);
        reporter.addToReport(thisMusc.getOutput(outputName));
    end
end

model.addComponent(reporter);
state = model.initSystem();

for indState = 0:numStates-1
    trajState = statesTrajectory.get(indState);
    
    % workaround to stateTrajectory bug
    state.setY(trajState.getY());
    state.setTime(trajState.getTime());
    
    model.realizeReport(state);
end

table = reporter.getTable();
sto = STOFileAdapter();
sto.write(table, 'muscleAnalysis.sto');

close(handles.figure1);
