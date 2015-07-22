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
%ReadOpenSimData
%   OutputData = ReadOpenSimStorage(dataFileName) creates a data
%   structure from an OpenSim storage (.sto) or motion (.mot) file.
%
% Note: This function is designed for the data file format used in
% OpenSim 2.3.2+.  For older files, please convert the data file using
% the ConvertFiles... function in the tools menu of the OpenSim GUI
%
% Input:
%   dataFileName: An OpenSim data file (.sto or .mot)
%
% Output:
%   The output of this script is a Matlab structure named OutputData. The
%   format of this structure can be passed to PlotOpenSimFunction.m for
%   plotting.
%
%   The stucture fields are:
%       name: A char array identifier of the data
%       nRows: the number of rows of data in the data field
%       nColumns: the number of columns of data in the data field
%       labels: an array of char arrays of data names from the header file
%       data: a nRows by nColumnss matrix of data values
% -----------------------------------------------------------------------
function OutputData = ReadOpenSimData(dataFileName)
    % Check location with exists
    if(exist(dataFileName, 'file') == 0)
       error('ReadOpenSimStorage:InvalidArgument', ...
           ['\tError in ReadOpenSimStorage:\n', ...
           '\t%s cannot be found'], dataFileName);
    end

    % Create output structure
    OutputData = struct();

    % Import Data
    unparsedData = importdata(dataFileName, '\t');

    % First line in text always name
    OutputData.name = unparsedData.textdata{1};

    % Parse remaining text lines
    for i = 2:1:size(unparsedData.textdata,1)-1
        if(~isempty(unparsedData.textdata{i}))
            scan = textscan(unparsedData.textdata{i}, '%s', 'delimiter', '=');
            if(strcmp(scan{1}(1), 'version'))
                OutputData.version = str2double(scan{1}(2));
            elseif(strcmp(scan{1}(1), 'nRows'))
                OutputData.nRows = str2double(scan{1}(2));
            elseif(strcmp(scan{1}(1), 'nColumns'))
                OutputData.nColumns = str2double(scan{1}(2));
            elseif(strcmp(scan{1}(1), 'inDegrees'))
                OutputData.inDegrees = char(scan{1}(2));
            end
        end
    end

    % Add in labels and data
    OutputData.labels = unparsedData.colheaders;
    OutputData.data = unparsedData.data;
end

