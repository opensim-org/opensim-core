% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2017 Stanford University and the Authors             %
% Author(s): Dominic Farris                                               %
%                                                                         %
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         % 
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %

% prescribeMotionInModel.m                                                        
% Author: Dominic Farris

function prescribeMotionInModel(Model_In, Mot_In, Model_Out)

% Function to take an existing model file and coordinate data accessed from an IK solution
% and write it as a Natural Cubic Spline Function to the Prescribed
% Function method of a Coordinate to a model file. Based off work done by
% Dominic Farris
%
% Inputs - Model_In - Existing model stored in osim file
%        - Mot_In - A file contains motion data for the particular model
%        - Model_Out - The output file with prescribed motion
%
% e.g. prescribedMotionInModel('myInputModel.osim','myMotionFile', 'myOutputModel.osim')
%
% Author - Dominic Farris (North Carolina State University). Please
% acknowledge contribution in published academic works
% last updated - 17/07/2012

import org.opensim.modeling.*

% Argument checking
error(nargchk(0, 3, nargin));

% If there aren't enough arguments passed in system will ask user to
% manually select file(s)

if nargin < 1
    [Model_In, modelpath] = uigetfile('.osim', 'Please select a model file');
    [Mot_In, motpath] = uigetfile('.mot', 'Please select a motion file');
    fileoutpath = [Model_In(1:end-5),'_Prescribed.osim'];
    modelfilepath = [modelpath Model_In];
    motfilepath = [motpath Mot_In];    
elseif nargin < 2
    [Mot_In, motpath] = uigetfile('.mot', 'Please select a motion file');
    fileoutpath = [Model_In(1:end-5),'_Prescribed.osim'];  
    motfilepath = [motpath Mot_In];       
    modelfilepath = Model_In;    
elseif nargin < 3
    fileoutpath = [Model_In(1:end-5),'_Prescribed.osim'];
    modelfilepath = Model_In;    
    motfilepath = Mot_In;
else
    modelfilepath = Model_In;
    motfilepath = Mot_In;
    fileoutpath = Model_Out;
end

% Initialize model
osimModel=Model(modelfilepath);

% Create the coordinate storage object from the input .sto file
coordinateSto=Storage(motfilepath);

% Rename the modified Model
osimModel.setName('modelWithPrescribedMotion');

% get coordinate set from model, and count the number of coordinates
modelCoordSet = osimModel.getCoordinateSet();
nCoords = modelCoordSet.getSize();

% for all coordinates in the model, create a function and prescribe
for i=0:nCoords-1
    
    % construct ArrayDouble objects for time and coordinate values
    Time=ArrayDouble();
    coordvalue = ArrayDouble();

    % Get the coordinate set from the model
	currentcoord = modelCoordSet.get(i);

    % Get the Time stamps and Coordinate values
    coordinateSto.getTimeColumn(Time);
    coordinateSto.getDataColumn(currentcoord.getName(),coordvalue); 
    
    % Check if it is a rotational or translational coordinate
    motion = currentcoord.getMotionType();
    
    % construct a SimmSpline object (previously NaturalCubicSpline)
    Spline = SimmSpline();
    
    %Now to write Time and coordvalue to Spline
    if strcmp(motion,'Rotational')
        % if the motion type is rotational we must convert to radians from degrees
        for j = 0:coordvalue.getSize()-1
            Spline.addPoint(Time.getitem(j),coordvalue.getitem(j)/(180/pi));
        end
    else % else we assume it's translational and can be left 'as is'
        for j = 0:coordvalue.getSize()-1
            Spline.addPoint(Time.getitem(j),coordvalue.getitem(j));
        end
    end

    % Add the SimmSpline to the PrescribedFunction of the Coordinate
    % being edited
    currentcoord.setPrescribedFunction(Spline);
    currentcoord.setDefaultIsPrescribed(1);
end

%  Save the Modified Model to a file
osimModel.print(fileoutpath);
disp(['The new model has been saved at ' fileoutpath]);
