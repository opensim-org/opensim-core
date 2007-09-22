
% de2_ss_walk1
%
% These lines create new setup files for a trial from the gait example.
% This code block will make all setup files and excitation constraints in a
% new CMC control constraints file for this trial.
RUN_CODE_BLOCK_DE2_SS_WALK1 = true;
if RUN_CODE_BLOCK_DE2_SS_WALK1
    gait2392ExampleDirectory = 'D:\programfiles\OpenSim\OpenSim\Matlab\SetupFileWriting\ExampleSetupFiles';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware_ss_auto';
    subjectName = 'delaware2';
    trialName = 'de2_ss_walk1';
    subjectMass = '65.9';
    subjectHeight = '1830.0';
    femurScaleFactor = '1.11031 1.11031 1.11031';
    tibiaScaleFactor = '0.962095 0.962095 0.962095';
    fxMinControlConstraintRRA = '-20.0';
    fxMaxControlConstraintRRA = '20.0';
    fyMinControlConstraintRRA = '-20.0';
    fyMaxControlConstraintRRA = '20.0';
    fzMinControlConstraintRRA = '-20.0';
    fzMaxControlConstraintRRA = '20.0';
    mxMinControlConstraintRRA = '-50.0';
    mxMaxControlConstraintRRA = '50.0';
    myMinControlConstraintRRA = '-50.0';
    myMaxControlConstraintRRA = '50.0';
    mzMinControlConstraintRRA = '-50.0';
    mzMaxControlConstraintRRA = '50.0';
    fxOptimalForceRRA = '4.0';
    fyOptimalForceRRA = '8.0';
    fzOptimalForceRRA = '4.0';
    mxOptimalForceRRA = '2.0';
    myOptimalForceRRA = '2.0';
    mzOptimalForceRRA = '2.0';
    fxMinControlConstraintCMC = '-1.0';
    fxMaxControlConstraintCMC = '1.0';
    fyMinControlConstraintCMC = '-1.0';
    fyMaxControlConstraintCMC = '1.0';
    fzMinControlConstraintCMC = '-1.0';
    fzMaxControlConstraintCMC = '1.0';
    mxMinControlConstraintCMC = '-1.0';
    mxMaxControlConstraintCMC = '1.0';
    myMinControlConstraintCMC = '-1.0';
    myMaxControlConstraintCMC = '1.0';
    mzMinControlConstraintCMC = '-1.0';
    mzMaxControlConstraintCMC = '1.0';
    fxOptimalForceCMC = '100.0';
    fyOptimalForceCMC = '200.0';
    fzOptimalForceCMC = '100.0';
    mxOptimalForceCMC = '80.0';
    myOptimalForceCMC = '80.0';
    mzOptimalForceCMC = '80.0';
    idResultsDirectory = './ResultsCMC';
    dynamicsEngine = 'sdfast';
    %dynamicsEngine = 'simbody';
    ikOptimizerAlgorithm    = 'cfsqp';
    rraOptimizerAlgorithm   = 'cfsqp';
    cmcOptimizerAlgorithm   = 'cfsqp';
    %ikOptimizerAlgorithm    = 'ipopt';
    %rraOptimizerAlgorithm   = 'ipopt';
    %cmcOptimizerAlgorithm   = 'ipopt';
    specialSettings = { subjectMass subjectHeight femurScaleFactor tibiaScaleFactor fxMinControlConstraintRRA ...
                        fxMaxControlConstraintRRA fyMinControlConstraintRRA fyMaxControlConstraintRRA ...
                        fzMinControlConstraintRRA fzMaxControlConstraintRRA mxMinControlConstraintRRA ...
                        mxMaxControlConstraintRRA myMinControlConstraintRRA myMaxControlConstraintRRA ...
                        mzMinControlConstraintRRA mzMaxControlConstraintRRA fxOptimalForceRRA ...
                        fyOptimalForceRRA fzOptimalForceRRA mxOptimalForceRRA myOptimalForceRRA ...
                        mzOptimalForceRRA fxMinControlConstraintCMC fxMaxControlConstraintCMC ...
                        fyMinControlConstraintCMC fyMaxControlConstraintCMC fzMinControlConstraintCMC ...
                        fzMaxControlConstraintCMC mxMinControlConstraintCMC mxMaxControlConstraintCMC ...
                        myMinControlConstraintCMC myMaxControlConstraintCMC mzMinControlConstraintCMC ...
                        mzMaxControlConstraintCMC fxOptimalForceCMC fyOptimalForceCMC fzOptimalForceCMC ...
                        mxOptimalForceCMC myOptimalForceCMC mzOptimalForceCMC idResultsDirectory ...
                        dynamicsEngine ikOptimizerAlgorithm rraOptimizerAlgorithm ...
                        cmcOptimizerAlgorithm };
    setupwrite( gait2392ExampleDirectory, outputDirectory, subjectName, trialName, specialSettings );
    initialPercent = 0;
    finalPercent = 150;
    inputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware_ss_auto';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_ss_auto';
    subjectName = 'delaware2';
    trialName = 'de2_ss_walk1';
    initialTimeCushion = 0.1;
    finalTimeCushion = 0.1;
    cmcInitialTimeAnticushion = 0.0;
    cmcFinalTimeAnticushion = 0.0;
    setSetupValuesForPercentGaitCycle( initialPercent, finalPercent, inputDirectory, outputDirectory, subjectName, trialName, initialTimeCushion, finalTimeCushion, cmcInitialTimeAnticushion, cmcFinalTimeAnticushion );
    excitationOffRegions = { 'semimem_r'  'stance 50  swing  30 ' ; ...
                             'semiten_r'  'stance 50  swing  30 ' ; ...
                             'bifemlh_r'  'stance 50  swing  30 ' ; ...
                             'bifemsh_r'  'swing  80  stance 50 ' ; ...
                             'sar_r'      'stance 0   stance 100' ; ...
                             'tfl_r'      'swing  0   swing  100' ; ...
                             'add_mag1_r' 'stance 50  stance 100' ; ...
                             'add_mag2_r' 'stance 50  stance 100' ; ...
                             'add_mag3_r' 'stance 50  stance 100' ; ...
                             'rect_fem_r' 'swing  40  swing  60 ' ; ...
                             'semimem_l'  'stance 50  swing  30 ' ; ...
                             'semiten_l'  'stance 50  swing  30 ' ; ...
                             'bifemlh_l'  'stance 50  swing  30 ' ; ...
                             'bifemsh_l'  'swing  80  stance 50 ' ; ...
                             'sar_l'      'stance 0   stance 100' ; ...
                             'tfl_l'      'swing  0   swing  100' ; ...
                             'add_mag1_l' 'stance 50  stance 100' ; ...
                             'add_mag2_l' 'stance 50  stance 100' ; ...
                             'add_mag3_l' 'stance 50  stance 100' ; ...
                             'rect_fem_l' 'swing  40  swing  60 ' };
    outputFileName = [trialName '_CMC_ControlConstraints_Constrained.xml'];
    constrainMuscleExcitationsForCMC( excitationOffRegions, outputDirectory, outputFileName, trialName );
end
