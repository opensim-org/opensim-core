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

% Last Modified by GUIDE v2.5 09-Feb-2017 15:49:43

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

% NEWACT - Plot new activation.
function newact_Callback(hObject, eventdata, handles)

names = {'clear','newact'};

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
