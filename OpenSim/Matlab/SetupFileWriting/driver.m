%
% BELOW ARE THE CODE BLOCKS I'M USING FOR MY SIMULATIONS RIGHT NOW
%

% de2_vslow_walk1
%
% These lines create new setup files for a trial from the gait example.
% This code block will make all setup files and excitation constraints in a
% new CMC control constraints file for this trial.
RUN_CODE_BLOCK_DE2_VSLOW_WALK1 = true;
if RUN_CODE_BLOCK_DE2_VSLOW_WALK1
    gait2392ExampleDirectory = 'D:\programfiles\OpenSim\OpenSim\Examples\Gait2392';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware_vslow_auto';
    subjectName = 'delaware2';
    trialName = 'de2_vslow_walk1';
    subjectMass = '65.9';
    subjectHeight = '1830.0';
    femurScaleFactor = '1.11031 1.11031 1.11031';
    tibiaScaleFactor = '0.962095 0.962095 0.962095';
    fxMinControlConstraintRRA2 = '-20.0';
    fxMaxControlConstraintRRA2 = '20.0';
    fyMinControlConstraintRRA2 = '-20.0';
    fyMaxControlConstraintRRA2 = '20.0';
    fzMinControlConstraintRRA2 = '-20.0';
    fzMaxControlConstraintRRA2 = '20.0';
    mxMinControlConstraintRRA2 = '-50.0';
    mxMaxControlConstraintRRA2 = '50.0';
    myMinControlConstraintRRA2 = '-50.0';
    myMaxControlConstraintRRA2 = '50.0';
    mzMinControlConstraintRRA2 = '-50.0';
    mzMaxControlConstraintRRA2 = '50.0';
    fxOptimalForceRRA2 = '4.0';
    fyOptimalForceRRA2 = '8.0';
    fzOptimalForceRRA2 = '4.0';
    mxOptimalForceRRA2 = '2.0';
    myOptimalForceRRA2 = '2.0';
    mzOptimalForceRRA2 = '2.0';
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
    cmcOptimizerAlgorithm = 'cfsqp';
    idResultsDirectory = './ResultsCMC';
    specialSettings = { subjectMass subjectHeight femurScaleFactor tibiaScaleFactor fxMinControlConstraintRRA2 ...
                        fxMaxControlConstraintRRA2 fyMinControlConstraintRRA2 fyMaxControlConstraintRRA2 ...
                        fzMinControlConstraintRRA2 fzMaxControlConstraintRRA2 mxMinControlConstraintRRA2 ...
                        mxMaxControlConstraintRRA2 myMinControlConstraintRRA2 myMaxControlConstraintRRA2 ...
                        mzMinControlConstraintRRA2 mzMaxControlConstraintRRA2 fxOptimalForceRRA2 ...
                        fyOptimalForceRRA2 fzOptimalForceRRA2 mxOptimalForceRRA2 myOptimalForceRRA2 ...
                        mzOptimalForceRRA2 fxMinControlConstraintCMC fxMaxControlConstraintCMC ...
                        fyMinControlConstraintCMC fyMaxControlConstraintCMC fzMinControlConstraintCMC ...
                        fzMaxControlConstraintCMC mxMinControlConstraintCMC mxMaxControlConstraintCMC ...
                        myMinControlConstraintCMC myMaxControlConstraintCMC mzMinControlConstraintCMC ...
                        mzMaxControlConstraintCMC fxOptimalForceCMC fyOptimalForceCMC fzOptimalForceCMC ...
                        mxOptimalForceCMC myOptimalForceCMC mzOptimalForceCMC cmcOptimizerAlgorithm ...
                        idResultsDirectory };
    setupwrite( gait2392ExampleDirectory, outputDirectory, subjectName, trialName, specialSettings );
    initialPercent = 0;
    finalPercent = 150;
    inputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware_vslow_auto';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_vslow_auto';
    subjectName = 'delaware2';
    trialName = 'de2_vslow_walk1';
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

% de2_ss_walk1
%
% These lines create new setup files for a trial from the gait example.
% This code block will make all setup files and excitation constraints in a
% new CMC control constraints file for this trial.
RUN_CODE_BLOCK_DE2_SS_WALK1 = true;
if RUN_CODE_BLOCK_DE2_SS_WALK1
    gait2392ExampleDirectory = 'D:\programfiles\OpenSim\OpenSim\Examples\Gait2392';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware_ss_auto';
    subjectName = 'delaware2';
    trialName = 'de2_ss_walk1';
    subjectMass = '65.9';
    subjectHeight = '1830.0';
    femurScaleFactor = '1.11031 1.11031 1.11031';
    tibiaScaleFactor = '0.962095 0.962095 0.962095';
    fxMinControlConstraintRRA2 = '-20.0';
    fxMaxControlConstraintRRA2 = '20.0';
    fyMinControlConstraintRRA2 = '-20.0';
    fyMaxControlConstraintRRA2 = '20.0';
    fzMinControlConstraintRRA2 = '-20.0';
    fzMaxControlConstraintRRA2 = '20.0';
    mxMinControlConstraintRRA2 = '-50.0';
    mxMaxControlConstraintRRA2 = '50.0';
    myMinControlConstraintRRA2 = '-50.0';
    myMaxControlConstraintRRA2 = '50.0';
    mzMinControlConstraintRRA2 = '-50.0';
    mzMaxControlConstraintRRA2 = '50.0';
    fxOptimalForceRRA2 = '4.0';
    fyOptimalForceRRA2 = '8.0';
    fzOptimalForceRRA2 = '4.0';
    mxOptimalForceRRA2 = '2.0';
    myOptimalForceRRA2 = '2.0';
    mzOptimalForceRRA2 = '2.0';
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
    cmcOptimizerAlgorithm = 'cfsqp';
    idResultsDirectory = './ResultsCMC';
    specialSettings = { subjectMass subjectHeight femurScaleFactor tibiaScaleFactor fxMinControlConstraintRRA2 ...
                        fxMaxControlConstraintRRA2 fyMinControlConstraintRRA2 fyMaxControlConstraintRRA2 ...
                        fzMinControlConstraintRRA2 fzMaxControlConstraintRRA2 mxMinControlConstraintRRA2 ...
                        mxMaxControlConstraintRRA2 myMinControlConstraintRRA2 myMaxControlConstraintRRA2 ...
                        mzMinControlConstraintRRA2 mzMaxControlConstraintRRA2 fxOptimalForceRRA2 ...
                        fyOptimalForceRRA2 fzOptimalForceRRA2 mxOptimalForceRRA2 myOptimalForceRRA2 ...
                        mzOptimalForceRRA2 fxMinControlConstraintCMC fxMaxControlConstraintCMC ...
                        fyMinControlConstraintCMC fyMaxControlConstraintCMC fzMinControlConstraintCMC ...
                        fzMaxControlConstraintCMC mxMinControlConstraintCMC mxMaxControlConstraintCMC ...
                        myMinControlConstraintCMC myMaxControlConstraintCMC mzMinControlConstraintCMC ...
                        mzMaxControlConstraintCMC fxOptimalForceCMC fyOptimalForceCMC fzOptimalForceCMC ...
                        mxOptimalForceCMC myOptimalForceCMC mzOptimalForceCMC cmcOptimizerAlgorithm ...
                        idResultsDirectory };
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

% de2_fast_walk1
%
% These lines create new setup files for a trial from the gait example.
% This code block will make all setup files and excitation constraints in a
% new CMC control constraints file for this trial.
RUN_CODE_BLOCK_DE2_FAST_WALK1 = true;
if RUN_CODE_BLOCK_DE2_FAST_WALK1
    gait2392ExampleDirectory = 'D:\programfiles\OpenSim\OpenSim\Examples\Gait2392';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware_fast_auto';
    subjectName = 'delaware2';
    trialName = 'de2_fast_walk1';
    subjectMass = '65.9';
    subjectHeight = '1830.0';
    femurScaleFactor = '1.11031 1.11031 1.11031';
    tibiaScaleFactor = '0.962095 0.962095 0.962095';
    fxMinControlConstraintRRA2 = '-20.0';
    fxMaxControlConstraintRRA2 = '20.0';
    fyMinControlConstraintRRA2 = '-20.0';
    fyMaxControlConstraintRRA2 = '20.0';
    fzMinControlConstraintRRA2 = '-20.0';
    fzMaxControlConstraintRRA2 = '20.0';
    mxMinControlConstraintRRA2 = '-50.0';
    mxMaxControlConstraintRRA2 = '50.0';
    myMinControlConstraintRRA2 = '-50.0';
    myMaxControlConstraintRRA2 = '50.0';
    mzMinControlConstraintRRA2 = '-50.0';
    mzMaxControlConstraintRRA2 = '50.0';
    fxOptimalForceRRA2 = '4.0';
    fyOptimalForceRRA2 = '8.0';
    fzOptimalForceRRA2 = '4.0';
    mxOptimalForceRRA2 = '2.0';
    myOptimalForceRRA2 = '2.0';
    mzOptimalForceRRA2 = '2.0';
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
    cmcOptimizerAlgorithm = 'cfsqp';
    idResultsDirectory = './ResultsCMC';
    specialSettings = { subjectMass subjectHeight femurScaleFactor tibiaScaleFactor fxMinControlConstraintRRA2 ...
                        fxMaxControlConstraintRRA2 fyMinControlConstraintRRA2 fyMaxControlConstraintRRA2 ...
                        fzMinControlConstraintRRA2 fzMaxControlConstraintRRA2 mxMinControlConstraintRRA2 ...
                        mxMaxControlConstraintRRA2 myMinControlConstraintRRA2 myMaxControlConstraintRRA2 ...
                        mzMinControlConstraintRRA2 mzMaxControlConstraintRRA2 fxOptimalForceRRA2 ...
                        fyOptimalForceRRA2 fzOptimalForceRRA2 mxOptimalForceRRA2 myOptimalForceRRA2 ...
                        mzOptimalForceRRA2 fxMinControlConstraintCMC fxMaxControlConstraintCMC ...
                        fyMinControlConstraintCMC fyMaxControlConstraintCMC fzMinControlConstraintCMC ...
                        fzMaxControlConstraintCMC mxMinControlConstraintCMC mxMaxControlConstraintCMC ...
                        myMinControlConstraintCMC myMaxControlConstraintCMC mzMinControlConstraintCMC ...
                        mzMaxControlConstraintCMC fxOptimalForceCMC fyOptimalForceCMC fzOptimalForceCMC ...
                        mxOptimalForceCMC myOptimalForceCMC mzOptimalForceCMC cmcOptimizerAlgorithm ...
                        idResultsDirectory };
    setupwrite( gait2392ExampleDirectory, outputDirectory, subjectName, trialName, specialSettings );
    initialPercent = 0;
    finalPercent = 150;
    inputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware_fast_auto';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware2_1.5_fast_auto';
    subjectName = 'delaware2';
    trialName = 'de2_fast_walk1';
    initialTimeCushion = 0.1;
    finalTimeCushion = 0.1;
    cmcInitialTimeAnticushion = 0.0;
    cmcFinalTimeAnticushion = 0.05;
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
                             'semimem_l'  'stance 50  swing  30 ' ; ...
                             'semiten_l'  'stance 50  swing  30 ' ; ...
                             'bifemlh_l'  'stance 50  swing  30 ' ; ...
                             'bifemsh_l'  'swing  80  stance 50 ' ; ...
                             'sar_l'      'stance 0   stance 100' ; ...
                             'tfl_l'      'swing  0   swing  100' ; ...
                             'add_mag1_l' 'stance 50  stance 100' ; ...
                             'add_mag2_l' 'stance 50  stance 100' ; ...
                             'add_mag3_l' 'stance 50  stance 100' };
    outputFileName = [trialName '_CMC_ControlConstraints_Constrained.xml'];
    constrainMuscleExcitationsForCMC( excitationOffRegions, outputDirectory, outputFileName, trialName );
end

% de3_vslow_walk1
%
% These lines create new setup files for a trial from the gait example.
% This code block will make all setup files and excitation constraints in a
% new CMC control constraints file for this trial.
RUN_CODE_BLOCK_DE3_VSLOW_WALK1 = true;
if RUN_CODE_BLOCK_DE3_VSLOW_WALK1
    gait2392ExampleDirectory = 'D:\programfiles\OpenSim\OpenSim\Examples\Gait2392';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware3_vslow_auto';
    subjectName = 'delaware3';
    trialName = 'de3_vslow_walk1';
    subjectMass = '72.6';
    subjectHeight = '1803.4';
    femurScaleFactor = '1.14724 1.14724 1.14724';
    tibiaScaleFactor = '0.988523 0.988523 0.988523';
    fxMinControlConstraintRRA2 = '-20.0';
    fxMaxControlConstraintRRA2 = '20.0';
    fyMinControlConstraintRRA2 = '-20.0';
    fyMaxControlConstraintRRA2 = '20.0';
    fzMinControlConstraintRRA2 = '-20.0';
    fzMaxControlConstraintRRA2 = '20.0';
    mxMinControlConstraintRRA2 = '-50.0';
    mxMaxControlConstraintRRA2 = '50.0';
    myMinControlConstraintRRA2 = '-50.0';
    myMaxControlConstraintRRA2 = '50.0';
    mzMinControlConstraintRRA2 = '-50.0';
    mzMaxControlConstraintRRA2 = '50.0';
    fxOptimalForceRRA2 = '4.0';
    fyOptimalForceRRA2 = '8.0';
    fzOptimalForceRRA2 = '4.0';
    mxOptimalForceRRA2 = '2.0';
    myOptimalForceRRA2 = '2.0';
    mzOptimalForceRRA2 = '2.0';
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
    cmcOptimizerAlgorithm = 'cfsqp';
    idResultsDirectory = './ResultsCMC';
    specialSettings = { subjectMass subjectHeight femurScaleFactor tibiaScaleFactor fxMinControlConstraintRRA2 ...
                        fxMaxControlConstraintRRA2 fyMinControlConstraintRRA2 fyMaxControlConstraintRRA2 ...
                        fzMinControlConstraintRRA2 fzMaxControlConstraintRRA2 mxMinControlConstraintRRA2 ...
                        mxMaxControlConstraintRRA2 myMinControlConstraintRRA2 myMaxControlConstraintRRA2 ...
                        mzMinControlConstraintRRA2 mzMaxControlConstraintRRA2 fxOptimalForceRRA2 ...
                        fyOptimalForceRRA2 fzOptimalForceRRA2 mxOptimalForceRRA2 myOptimalForceRRA2 ...
                        mzOptimalForceRRA2 fxMinControlConstraintCMC fxMaxControlConstraintCMC ...
                        fyMinControlConstraintCMC fyMaxControlConstraintCMC fzMinControlConstraintCMC ...
                        fzMaxControlConstraintCMC mxMinControlConstraintCMC mxMaxControlConstraintCMC ...
                        myMinControlConstraintCMC myMaxControlConstraintCMC mzMinControlConstraintCMC ...
                        mzMaxControlConstraintCMC fxOptimalForceCMC fyOptimalForceCMC fzOptimalForceCMC ...
                        mxOptimalForceCMC myOptimalForceCMC mzOptimalForceCMC cmcOptimizerAlgorithm ...
                        idResultsDirectory };
    setupwrite( gait2392ExampleDirectory, outputDirectory, subjectName, trialName, specialSettings );
    initialPercent = 0;
    finalPercent = 150;
    inputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware3_vslow_auto';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware3_1.5_vslow_auto';
    subjectName = 'delaware3';
    trialName = 'de3_vslow_walk1';
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

% de3_ss_walk1
%
% These lines create new setup files for a trial from the gait example.
% This code block will make all setup files and excitation constraints in a
% new CMC control constraints file for this trial.
RUN_CODE_BLOCK_DE3_SS_WALK1 = true;
if RUN_CODE_BLOCK_DE3_SS_WALK1
    gait2392ExampleDirectory = 'D:\programfiles\OpenSim\OpenSim\Examples\Gait2392';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware3_ss_auto';
    subjectName = 'delaware3';
    trialName = 'de3_ss_walk1';
    subjectMass = '72.6';
    subjectHeight = '1803.4';
    femurScaleFactor = '1.14724 1.14724 1.14724';
    tibiaScaleFactor = '0.988523 0.988523 0.988523';
    fxMinControlConstraintRRA2 = '-20.0';
    fxMaxControlConstraintRRA2 = '20.0';
    fyMinControlConstraintRRA2 = '-20.0';
    fyMaxControlConstraintRRA2 = '20.0';
    fzMinControlConstraintRRA2 = '-20.0';
    fzMaxControlConstraintRRA2 = '20.0';
    mxMinControlConstraintRRA2 = '-50.0';
    mxMaxControlConstraintRRA2 = '50.0';
    myMinControlConstraintRRA2 = '-50.0';
    myMaxControlConstraintRRA2 = '50.0';
    mzMinControlConstraintRRA2 = '-50.0';
    mzMaxControlConstraintRRA2 = '50.0';
    fxOptimalForceRRA2 = '4.0';
    fyOptimalForceRRA2 = '8.0';
    fzOptimalForceRRA2 = '4.0';
    mxOptimalForceRRA2 = '2.0';
    myOptimalForceRRA2 = '2.0';
    mzOptimalForceRRA2 = '2.0';
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
    cmcOptimizerAlgorithm = 'cfsqp';
    idResultsDirectory = './ResultsCMC';
    specialSettings = { subjectMass subjectHeight femurScaleFactor tibiaScaleFactor fxMinControlConstraintRRA2 ...
                        fxMaxControlConstraintRRA2 fyMinControlConstraintRRA2 fyMaxControlConstraintRRA2 ...
                        fzMinControlConstraintRRA2 fzMaxControlConstraintRRA2 mxMinControlConstraintRRA2 ...
                        mxMaxControlConstraintRRA2 myMinControlConstraintRRA2 myMaxControlConstraintRRA2 ...
                        mzMinControlConstraintRRA2 mzMaxControlConstraintRRA2 fxOptimalForceRRA2 ...
                        fyOptimalForceRRA2 fzOptimalForceRRA2 mxOptimalForceRRA2 myOptimalForceRRA2 ...
                        mzOptimalForceRRA2 fxMinControlConstraintCMC fxMaxControlConstraintCMC ...
                        fyMinControlConstraintCMC fyMaxControlConstraintCMC fzMinControlConstraintCMC ...
                        fzMaxControlConstraintCMC mxMinControlConstraintCMC mxMaxControlConstraintCMC ...
                        myMinControlConstraintCMC myMaxControlConstraintCMC mzMinControlConstraintCMC ...
                        mzMaxControlConstraintCMC fxOptimalForceCMC fyOptimalForceCMC fzOptimalForceCMC ...
                        mxOptimalForceCMC myOptimalForceCMC mzOptimalForceCMC cmcOptimizerAlgorithm ...
                        idResultsDirectory };
    setupwrite( gait2392ExampleDirectory, outputDirectory, subjectName, trialName, specialSettings );
    initialPercent = 0;
    finalPercent = 150;
    inputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware3_ss_auto';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware3_1.5_ss_auto';
    subjectName = 'delaware3';
    trialName = 'de3_ss_walk1';
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

% de3_fast_walk1
%
% These lines create new setup files for a trial from the gait example.
% This code block will make all setup files and excitation constraints in a
% new CMC control constraints file for this trial.
RUN_CODE_BLOCK_DE3_FAST_WALK1 = true;
if RUN_CODE_BLOCK_DE3_FAST_WALK1
    gait2392ExampleDirectory = 'D:\programfiles\OpenSim\OpenSim\Examples\Gait2392';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware3_fast_auto';
    subjectName = 'delaware3';
    trialName = 'de3_fast_walk1';
    subjectMass = '72.6';
    subjectHeight = '1803.4';
    femurScaleFactor = '1.14724 1.14724 1.14724';
    tibiaScaleFactor = '0.988523 0.988523 0.988523';
    fxMinControlConstraintRRA2 = '-20.0';
    fxMaxControlConstraintRRA2 = '20.0';
    fyMinControlConstraintRRA2 = '-50.0';
    fyMaxControlConstraintRRA2 = '50.0';
    fzMinControlConstraintRRA2 = '-20.0';
    fzMaxControlConstraintRRA2 = '20.0';
    mxMinControlConstraintRRA2 = '-200.0';
    mxMaxControlConstraintRRA2 = '200.0';
    myMinControlConstraintRRA2 = '-200.0';
    myMaxControlConstraintRRA2 = '200.0';
    mzMinControlConstraintRRA2 = '-200.0';
    mzMaxControlConstraintRRA2 = '200.0';
    fxOptimalForceRRA2 = '8.0';
    fyOptimalForceRRA2 = '16.0';
    fzOptimalForceRRA2 = '8.0';
    mxOptimalForceRRA2 = '4.0';
    myOptimalForceRRA2 = '4.0';
    mzOptimalForceRRA2 = '4.0';
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
    cmcOptimizerAlgorithm = 'cfsqp';
    idResultsDirectory = './ResultsCMC';
    specialSettings = { subjectMass subjectHeight femurScaleFactor tibiaScaleFactor fxMinControlConstraintRRA2 ...
                        fxMaxControlConstraintRRA2 fyMinControlConstraintRRA2 fyMaxControlConstraintRRA2 ...
                        fzMinControlConstraintRRA2 fzMaxControlConstraintRRA2 mxMinControlConstraintRRA2 ...
                        mxMaxControlConstraintRRA2 myMinControlConstraintRRA2 myMaxControlConstraintRRA2 ...
                        mzMinControlConstraintRRA2 mzMaxControlConstraintRRA2 fxOptimalForceRRA2 ...
                        fyOptimalForceRRA2 fzOptimalForceRRA2 mxOptimalForceRRA2 myOptimalForceRRA2 ...
                        mzOptimalForceRRA2 fxMinControlConstraintCMC fxMaxControlConstraintCMC ...
                        fyMinControlConstraintCMC fyMaxControlConstraintCMC fzMinControlConstraintCMC ...
                        fzMaxControlConstraintCMC mxMinControlConstraintCMC mxMaxControlConstraintCMC ...
                        myMinControlConstraintCMC myMaxControlConstraintCMC mzMinControlConstraintCMC ...
                        mzMaxControlConstraintCMC fxOptimalForceCMC fyOptimalForceCMC fzOptimalForceCMC ...
                        mxOptimalForceCMC myOptimalForceCMC mzOptimalForceCMC cmcOptimizerAlgorithm ...
                        idResultsDirectory };
    setupwrite( gait2392ExampleDirectory, outputDirectory, subjectName, trialName, specialSettings );
    initialPercent = 100;
    finalPercent = 250;
    inputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware3_fast_auto';
    outputDirectory = 'D:\programfiles\FCA\SU\Testing\delaware3_1.5_fast_auto';
    subjectName = 'delaware3';
    trialName = 'de3_fast_walk1';
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
                             'semimem_l'  'stance 50  swing  30 ' ; ...
                             'semiten_l'  'stance 50  swing  30 ' ; ...
                             'bifemlh_l'  'stance 50  swing  30 ' ; ...
                             'bifemsh_l'  'swing  80  stance 50 ' ; ...
                             'sar_l'      'stance 0   stance 100' ; ...
                             'tfl_l'      'swing  0   swing  100' ; ...
                             'add_mag1_l' 'stance 50  stance 100' ; ...
                             'add_mag2_l' 'stance 50  stance 100' ; ...
                             'add_mag3_l' 'stance 50  stance 100' };
    outputFileName = [trialName '_CMC_ControlConstraints_Constrained.xml'];
    constrainMuscleExcitationsForCMC( excitationOffRegions, outputDirectory, outputFileName, trialName );
end
