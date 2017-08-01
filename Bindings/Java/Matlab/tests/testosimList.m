

import org.opensim.modeling.*

model = Model('../../../../OpenSim/Tests/shared/arm26.osim')

%% Body list
% get a Matlab BodyList object
bodylist = osimList(model,'Body');
% size of the list 
n = bodylist.getSize();  
% cell of strings
names = bodylist.getNames(); 
% cell of strings
outputnames = bodylist.getOutputNames(); 
% get a reference to a a body
body = bodylist.get(names{1}); 


%% Muscle List 
% get a Matlab MuscleList object
musclelist = osimList(model, 'Muscle');
% get the number of muscles
nm = musclelist.getSize();
% get the names of all the muslces
names = musclelist.getNames();
% get all the outputs from the class type
outputnames = musclelist.getOutputNames();
% get a reference to a muscle
muscle = musclelist.get(names{1});










