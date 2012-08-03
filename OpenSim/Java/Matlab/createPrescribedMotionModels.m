% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2012 Stanford University and the Authors             %
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


function createPrescribedMotionModels(subjectNumber,unprescribedModelIn)

subjectDir = ['C:\Documents and Settings\emarnold\My Documents\My Dropbox\__MultiSubjectFiberLengths\FiberLengthPipeline\' subjectNumber];
cd(subjectDir);

motionsToPrescribe = dir('./IKResults/analysis/*rad_q.sto');
nMotions = length(motionsToPrescribe);

%unprescribedModelIn = './hpl_03_markerscaled_emgMusc.patcor.osim'

for i= 1:nMotions;
        
    motionFileIn = ['./IKResults/analysis/' motionsToPrescribe(i,1).name];
    
    % create name of trial from .trc file name
    modelFileName = strrep(motionsToPrescribe(i,1).name,'_Kinematics_rad_q.sto',['_hpl_',subjectNumber,'.osim']);
    
    prescribedModelOut = ['.\PrescribedModels\' modelFileName]
    
   Command = ['..\PrescribedMotionModelTrunk\Release\prescribeMotionInModel.exe' ' ' unprescribedModelIn ' ' motionFileIn ' ' prescribedModelOut];
   system(Command)
   %fprintf(['\nPrescribed coordinates in:' prescribedModelOut '/n']);
   
   
   
end


end

