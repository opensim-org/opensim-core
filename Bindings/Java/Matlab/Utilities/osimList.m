% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2018 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
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

classdef osimList < matlab.mixin.SetGet
% osimList(classname)
%   Use OpenSim lists to iterate through model components
%   OpenSim Utility Class
%   Inputs:
%   model               Reference to OpenSim Model (Model())
%   classname           OpenSim component class name ('Body', 'Muscle' 

properties
        model % a reference to an opensim model
        list; % an opensim list
    end
    
    methods 
        function obj = osimList(model,classname)
            % Constructor for osimList class. Takes a model and a classname
            % (string). constructs an instance of the osimList class with
            % properties model and list
            if nargin == 0
                error('no inputs to constructor')
            elseif nargin == 1
                error('constructor takes two inputs, not one')
            elseif nargin > 2
                error(['2 inputs required, ' num2str(nargin) 'given'])
            end
            
            % Use input string (classname) to determine the type of list
            % returned. Examples could be 'Body', 'Frame', 'Joint',
            % 'Muscle', 'Actuator'
            try
                eval(['list = model.get' classname 'List();']);
                disp('List creation Successful');
            catch 
                error(['OpenSim classname, ' classname ', does not exist']);
            end
            
            % allocate the list and model to local properties
            obj.list = list;
            obj.model = model;
        end
        function size = getSize(obj)
            % get the size of the list
            list = obj.list;
            li = list.begin();
            size = 0;
            while ~li.equals(list.end())
                 li.next();
                 size = size + 1;
            end
        end
        function names = getNames(obj)
            % get all names in the list as a Matlab array of strings
            list = obj.list;
            li = list.begin();
            names = [{}];
            while ~li.equals(list.end())
                names = [names {char(li.getName())}];
                li.next();
            end
            names = names';
        end
        function outputnames = getOutputNames(obj)
            % get all output names of the class as a Matlab array of strings
            list = obj.list;
            li = list.begin();
            outNames = li.getOutputNames();
            sz = outNames.size();
            outputnames = [];
            for j = 0 :sz - 1 
                outputnames = [outputnames {char(outNames.get(j))}];
            end
            outputnames = outputnames';
        end
        function reference = getByName(obj,name)
            % return a reference for a object in the list
            import org.opensim.modeling.*
            list = obj.list;
            li = list.begin();
            while ~li.equals(list.end())
                if strcmp(char(li.getName()),name)
                    reference = li.deref;
                    break
                end
                li.next();
            end 
            if ~exist('reference','var')
                error(['Component name not found: ' name])
            end
        end
        function reference = getByIndex(obj,index)
            % return a reference for a object in the list
            list = obj.list;
            li = list.begin();
            if index > 0
                for i = 1 : index 
                    li.next();
                end
            end
            
            if index > obj.getSize() - 1
                error(['index is out of bounds. Index range is 0 to ' num2str(obj.getSize()-1)])
            end
            reference = li.deref;
        end
    end
end

       