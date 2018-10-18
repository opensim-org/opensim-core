% osimList()
%   OsimList provides static methods to get a reference to a Model component
%   using the Model component interface. according to type by name. Use
%   this method when you are trying to get a reference to a single
%   component. 
% 
%   Get Number of components in a list;
%       nComps = osimList().getNumComponents(model, classname);
%   Get the all component names as strings;
%       names  = osimList().getComponentNames(model, classname);
%   Get a reference to a component;
%       ref    = osimList().getComponent(model, classname, name);
%     
%
%   When iterating through a list to get/set properties on multiple 
%   components, use the list iterator directly;
%
%   % Set a value for a property for all of the components of type 'Body'
%   list = model.getBodyList();
%   li = list.begin()
%   while ~li.equals(list.end())
%       li.next();
%       li.setSomePropertyValue(val);
%   end

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

    methods (Static)
        function nComps = getNumComponents(model, classname)
            % Get the number of components, of a type, in a model
            %   nComponents = osimList.getNumComponents(Model(), Classname)
            %   eg n = osimList.getNumComponents(model, 'Body')  
            
            % Validate the number of inputs and get the Component list
            list = osimList.getList(model, classname);
            
            % Get the size of the list
            li = list.begin();
            nComps = 0;
            while ~li.equals(list.end())
                 li.next();
                 nComps = nComps + 1;
            end
        end
        function names = getComponentNames(model, classname)
            % Get the names of all the components, of a type, in a model
            %   listOfNames = osimList.getComponentNames(Model(), Classname)
            %   eg names = osimList.getComponentNames(model, 'Body')  
            
            % Validate the number of inputs and get the Component list
            list = osimList.getList(model, classname);
            
            % Get all names in the list as a Matlab cell of strings
            li = list.begin();
            names = {};
            while ~li.equals(list.end())
                names = [names {char(li.getName())}];
                li.next();
            end
            names = names';
            
        end
        function reference = getComponent(model, classname, name)
            % Get a component reference from the list by name
            %   reference = osimList.getComponent(Model(), Classname,  String )
            %   eg ref = osimList.getComponent(model, 'Body', 'Pelvis')   
            
            % Validate the number of inputs and get the Component list
            list = osimList.getList(model, classname);
            
            % Get a reference to the component of interest
            li = list.begin();
            while ~li.equals(list.end())
                if strcmp(char(li.getName()),name)
                    reference = li.deref;
                    return
                end
                li.next();
            end 
            if ~exist('reference','var')
                error(['Component name not found: ' name])
            end
        end
        function outputnames = getOutputNames(model, classname)
            % Get a list of output names as a cell of strings
            %   OutputNames = osimList.getOutputNames(Model(), Classname)
            %   eg oNames = osimList.getOutputNames(model, 'Muscle')   
            
            % Validate the number of inputs and get the Component list
            list = osimList.getList(model, classname);
            
            % Get all the output names from a component 
            li = list.begin();
            outNames = li.getOutputNames();
            sz = outNames.size();
            outputnames = [];
            for j = 0 :sz - 1 
                outputnames = [outputnames {char(outNames.get(j))}];
            end
            outputnames = outputnames';
        end 
        function list = getList(model, classname)
            try
                eval(['list = model.get' classname 'List();']);
            catch 
                error(['No list for get' classname 'List() found']);
            end
        end
    end
end
 


       