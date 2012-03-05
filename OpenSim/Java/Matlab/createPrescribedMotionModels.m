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

