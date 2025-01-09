function matlabcellarray = osimList2MatlabCell(model, classname)
%% osimList2MatlabCell()
%   Convert an OpenSim list into a cell array of OpenSim
%   references. The cell array can then be used to get
%   references to individual components. 
%
% 
%   % Get a cell array of references to all the bodies in a model
%   references = osimList2MatlabCell(model,'Body')
%   % Get the first body in the list.  
%   Pelvis = references{1}
%
%   osimList2MatlabCell() uses the ComponentList() and Interator() classes 
%   to iterate through a list to get the components. An example of using
%   the iterator is below;
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

%% import opensim libraries. 
import org.opensim.modeling.*

%% Try the givenclassname
try
    eval(['list = model.get' classname 'List();']);
    disp('List instantiation Successful');
catch 
    error(['No list for get' classname 'List() found']);
end

%% Get a reference to the component of interest
matlabcellarray = {};
li = list.begin();
while ~li.equals(list.end())
    matlabcellarray{end + 1,1} = li.deref;
    li.next();
end 

end







