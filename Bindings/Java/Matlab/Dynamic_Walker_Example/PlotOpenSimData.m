% ----------------------------------------------------------------------- 
% The OpenSim API is a toolkit for musculoskeletal modeling and           
% simulation. See http://opensim.stanford.edu and the NOTICE file         
% for more information. OpenSim is developed at Stanford University       
% and supported by the US National Institutes of Health (U54 GM072970,    
% R24 HD065690) and by DARPA through the Warrior Web program.             
%                                                                         
% Copyright (c) 2005-2013 Stanford University and the Authors             
% Author(s): Daniel A. Jacobs                                             
%                                                                         
% Licensed under the Apache License, Version 2.0 (the "License");         
% you may not use this file except in compliance with the License.        
% You may obtain a copy of the License at                                 
% http://www.apache.org/licenses/LICENSE-2.0.                             
%                                                                         
% Unless required by applicable law or agreed to in writing, software     
% distributed under the License is distributed on an "AS IS" BASIS,       
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         
% implied. See the License for the specific language governing            
% permissions and limitations under the License.                          
% ----------------------------------------------------------------------- 
%PlotOpensimData  
%  figHandle = PlotOpensimData(dataStructure, xQuantity, yQuantities) 
% creates a plot of the yQuanties vs the xQuantity using the supplied 
% string names.
%
% Note: To create the appropriate data structure, please use
% ReadOpenSimStorage or IntegrateOpenSimPlant before using this function
%
% Input:
%   dataStructure: a structure of formatted data
%       The stucture fields are:
%           name: A char array identifier of the data
%           nRows: the number of rows of data in the data field
%           nColumns: the number of columns of data in the data field
%           labels: an array of char arrays of names from the header file
%           data: a nRows by nColumnss matrix of data values
%   xQuantity: A char array of the name of an independent variable to plot
%   yQuantities: A cell of the names of the dependent variables to plot
%
% Usage:
%   PlotOpenSimData(outputData, 'time', {'Pelvis_tx'})
%   PlotOpenSimData(outputData, 'time', {'Pelvis_tx', 'Pelvis_ty'})
% ----------------------------------------------------------------------- 
function [figHandle, axisHandle] = PlotOpenSimData(dataStructure, ... 
    xQuantity, yQuantities)
% Set up plot parameters
colorOpts = {'b', 'g', 'r', 'c', 'm'};
lineOpts = {'-', '-.','--', ':'};
total = length(colorOpts)*length(lineOpts);
lineWidth = 2.5;

% Error Check Structure
if(~ischar(xQuantity))
    error('PlotOpensimData:InvalidArgument', [...
        '\tError in PlotOpensimData:\n', ...
        '\tThe argument xQuantities should be a char array']);
end
if(~iscell(yQuantities))
    error('PlotOpensimData:InvalidArgument', [...
        '\tError in PlotOpensimData:\n', ...
        '\tThe argument yQuantities should be a cell array of strings']);
end
if(~isfield(dataStructure, 'name'))
    error('PlotOpensimData:InvalidArgument', [...
        '\tError in PlotOpensimData:\n',...
        '\tThe argument dataStructure does not have a name field.']);
end
if(~isfield(dataStructure, 'nRows'))
    error('PlotOpensimData:InvalidArgument', [...
        '\tError in PlotOpensimData:\n',...
        '\tThe argument dataStructure does not have a nRows field.']);
end
if(~isfield(dataStructure, 'nColumns'))
    error('PlotOpensimData:InvalidArgument', [...
        '\tError in PlotOpensimData:\n',...
        '\tThe argument dataStructure does not have a nColumns field.']);
end
if(~isfield(dataStructure, 'labels'))
    error('PlotOpensimData:InvalidArgument', [...
        '\tError in PlotOpensimData:\n',...
        '\tThe argument dataStructure does not have a labels field.']);
end
if(~isfield(dataStructure, 'data'))
    error('PlotOpensimData:InvalidArgument', [...
        '\tError in PlotOpensimData:\n',...
        '\tThe argument dataStructure does not have a data field.']);
end

% Create figure
figHandle = figure;
axisHandle = gca();
hold on

% Add Data
indx = strcmp(dataStructure.labels(:),xQuantity);
for j = 1:1:length(yQuantities)
    indy = strcmp(dataStructure.labels(:),yQuantities{j});
    if(~any(indy))
        close(figHandle)
        error('PlotOpensimData:InvalidArgument', [...
            '\tError in PlotOpensimData:\n',...
            '\tThe quantity %s is not in dataStructure.labels', ...
            yQuantities{j}]);
    end
    plot(dataStructure.data(:,indx), dataStructure.data(:,indy), ...
        [lineOpts{floor(1+j/length(colorOpts))}, ...
        colorOpts{1+mod(j,length(colorOpts))}], 'linewidth', lineWidth);
    xlabel(xQuantity, 'Interpreter', 'none');
end
legend(yQuantities, 'Interpreter', 'none')
hold off
end