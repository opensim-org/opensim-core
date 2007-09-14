function figContents = ref_muscleActPlotLabelsDelawareSinglePlot()
% Purpose:  Returns a list of labels specifying the data to be plotted
%           in each subplot of each figure created by 
%           compare_muscleActFromMotDirectOverlaySinglePlot(). 
%
%           NOTES:
%           qPlotLabel{} specifies column headings of a motion file 
%               associated with the UW Gait Workflow; the corresponding 
%               data will be plotted on figNum(plotNum)
%           perryAbbr{} specifies muscle abbreviations as coded in
%               get_emgTimingFromPerry()
%           subplotTitle{} specifies title of each subplot
%           subplotAxisLabel{} specifies y-axis label of each subplot
%           subplotRange{} specifies the min, max values for scaling each
%               subplot
%           subplotTick{} specifies the y-tick labels of each subplot
%
% Output:   figContents returns a structure, formatted as follows:
%               *(figNum).qPlotLabel{plotNum}                                
%               *(figNum).perryAbbr{muscleAbbrNum}        
%               *(figNum).subplotTitle{plotNum}   
%               *(figNum).subplotAxisLabel{plotNum}   
%               *(figNum).subplotRange{plotNum}   
%               *(figNum).subplotTick{plotNum}
%
% ASA, 11-05, rev 2-06


% Figures 1-6: Gluteus maximus, medius, and minimus.
figContents(1).qPlotLabel{1} = {'glut_max1_l'};
figContents(2).qPlotLabel{1} = {'glut_max1_r'};
figContents(3).qPlotLabel{1} = {'glut_med1_l'};
figContents(4).qPlotLabel{1} = {'glut_med1_r'};
figContents(5).qPlotLabel{1} = {'glut_min1_l'};
figContents(6).qPlotLabel{1} = {'glut_min1_r'};
figContents(1).perryAbbr = {'GMAX upper', 'GMAX lower'};
figContents(2).perryAbbr = {'GMAX upper', 'GMAX lower'};
figContents(3).perryAbbr = {'GMED'};
figContents(4).perryAbbr = {'GMED'};
figContents(5).perryAbbr = {};
figContents(6).perryAbbr = {};
figContents(1).muscleActivationPlotIndices = {1};
figContents(2).muscleActivationPlotIndices = {1};
figContents(3).muscleActivationPlotIndices = {1};
figContents(4).muscleActivationPlotIndices = {1};
figContents(5).muscleActivationPlotIndices = {1};
figContents(6).muscleActivationPlotIndices = {1};
figContents(1).muscleNumbersInPlot = {1, 2};
figContents(2).muscleNumbersInPlot = {1, 2};
figContents(3).muscleNumbersInPlot = {1};
figContents(4).muscleNumbersInPlot = {1};
figContents(5).muscleNumbersInPlot = {};
figContents(6).muscleNumbersInPlot = {};
figContents(1).subplotTitle{1} = {'Gluteus Maximus'};
figContents(2).subplotTitle{1} = {'Gluteus Maximus'};
figContents(3).subplotTitle{1} = {'Gluteus Medius'};
figContents(4).subplotTitle{1} = {'Gluteus Medius'};
figContents(5).subplotTitle{1} = {'Gluteus Minimus'};
figContents(6).subplotTitle{1} = {'Gluteus Minimus'};
figContents(1).subplotAxisLabel{1} = 'muscle activation';
figContents(2).subplotAxisLabel{1} = 'muscle activation';
figContents(3).subplotAxisLabel{1} = 'muscle activation';
figContents(4).subplotAxisLabel{1} = 'muscle activation';
figContents(5).subplotAxisLabel{1} = 'muscle activation';
figContents(6).subplotAxisLabel{1} = 'muscle activation';
figContents(1).subplotRange{1} = [0 1];
figContents(2).subplotRange{1} = [0 1];
figContents(3).subplotRange{1} = [0 1];
figContents(4).subplotRange{1} = [0 1];
figContents(5).subplotRange{1} = [0 1];
figContents(6).subplotRange{1} = [0 1];
figContents(1).subplotTick{1}  = 0:0.5:1;
figContents(2).subplotTick{1}  = 0:0.5:1;
figContents(3).subplotTick{1}  = 0:0.5:1;
figContents(4).subplotTick{1}  = 0:0.5:1;
figContents(5).subplotTick{1}  = 0:0.5:1;
figContents(6).subplotTick{1}  = 0:0.5:1;


% Figures 7-12: Iliopsoas, TFL, sartorius.
figContents(7).qPlotLabel{1}  = {'iliacus_l'};
figContents(8).qPlotLabel{1}  = {'iliacus_r'};
figContents(9).qPlotLabel{1}  = {'sar_l'};
figContents(10).qPlotLabel{1} = {'sar_r'};
figContents(11).qPlotLabel{1} = {'tfl_l'};
figContents(12).qPlotLabel{1} = {'tfl_r'};
figContents(7).perryAbbr = {'IL'};
figContents(8).perryAbbr = {'IL'};
figContents(9).perryAbbr = {'SAR'};
figContents(10).perryAbbr = {'SAR'};
figContents(11).perryAbbr = {'TFL'};
figContents(12).perryAbbr = {'TFL'};
figContents(7).muscleActivationPlotIndices = {1};
figContents(8).muscleActivationPlotIndices = {1};
figContents(9).muscleActivationPlotIndices = {1};
figContents(10).muscleActivationPlotIndices = {1};
figContents(11).muscleActivationPlotIndices = {1};
figContents(12).muscleActivationPlotIndices = {1};
figContents(7).muscleNumbersInPlot = {1};
figContents(8).muscleNumbersInPlot = {1};
figContents(9).muscleNumbersInPlot = {1};
figContents(10).muscleNumbersInPlot = {1};
figContents(11).muscleNumbersInPlot = {1};
figContents(12).muscleNumbersInPlot = {1};
figContents(7).subplotTitle{1}  = {'Iliacus'};
figContents(8).subplotTitle{1}  = {'Iliacus'};
figContents(9).subplotTitle{1}  = {'Sartorius'};
figContents(10).subplotTitle{1} = {'Sartorius'};
figContents(11).subplotTitle{1} = {'Tensor fasciae latae'};
figContents(12).subplotTitle{1} = {'Tensor fasciae latae'};
figContents(7).subplotAxisLabel{1}  = 'muscle activation';
figContents(8).subplotAxisLabel{1}  = 'muscle activation';
figContents(9).subplotAxisLabel{1}  = 'muscle activation';
figContents(10).subplotAxisLabel{1}  = 'muscle activation';
figContents(11).subplotAxisLabel{1}  = 'muscle activation';
figContents(12).subplotAxisLabel{1} = 'muscle activation';
figContents(7).subplotRange{1}  = [0 1];
figContents(8).subplotRange{1}  = [0 1];
figContents(9).subplotRange{1}  = [0 1];
figContents(10).subplotRange{1}  = [0 1];
figContents(11).subplotRange{1}  = [0 1];
figContents(12).subplotRange{1} = [0 1];
figContents(7).subplotTick{1}  = 0:0.5:1;
figContents(8).subplotTick{1}  = 0:0.5:1;
figContents(9).subplotTick{1}  = 0:0.5:1;
figContents(10).subplotTick{1}  = 0:0.5:1;
figContents(11).subplotTick{1}  = 0:0.5:1;
figContents(12).subplotTick{1} = 0:0.5:1;


% Figures 13-16: Vasti, RF.
figContents(13).qPlotLabel{1} = {'vas_med_l'};
figContents(14).qPlotLabel{1} = {'vas_med_r'};
figContents(15).qPlotLabel{1} = {'rect_fem_l'};
figContents(16).qPlotLabel{1} = {'rect_fem_r'};
figContents(13).perryAbbr = {'VASmed', 'VASint', 'VASlat'};
figContents(14).perryAbbr = {'VASmed', 'VASint', 'VASlat'};
figContents(15).perryAbbr = {'RF'};
figContents(16).perryAbbr = {'RF'};
figContents(13).muscleActivationPlotIndices = {1};
figContents(14).muscleActivationPlotIndices = {1};
figContents(15).muscleActivationPlotIndices = {1};
figContents(16).muscleActivationPlotIndices = {1};
figContents(13).muscleNumbersInPlot = {1, 2, 3};
figContents(14).muscleNumbersInPlot = {1, 2, 3};
figContents(15).muscleNumbersInPlot = {1};
figContents(16).muscleNumbersInPlot = {1};
figContents(13).subplotTitle{1}  = {'Vastus Medius'};
figContents(14).subplotTitle{1}  = {'Vastus Medius'};
figContents(15).subplotTitle{1}  = {'Rectus Femoris'};
figContents(16).subplotTitle{1} = {'Rectus Femoris'};
figContents(13).subplotAxisLabel{1}  = 'muscle activation';
figContents(14).subplotAxisLabel{1}  = 'muscle activation';
figContents(15).subplotAxisLabel{1}  = 'muscle activation';
figContents(16).subplotAxisLabel{1} = 'muscle activation';
figContents(13).subplotRange{1} = [0 1];
figContents(14).subplotRange{1} = [0 1];
figContents(15).subplotRange{1} = [0 1];
figContents(16).subplotRange{1} = [0 1];
figContents(13).subplotTick{1}  = 0:0.5:1;
figContents(14).subplotTick{1}  = 0:0.5:1;
figContents(15).subplotTick{1}  = 0:0.5:1;
figContents(16).subplotTick{1}  = 0:0.5:1;


% Figures 17-22: SM, ST, BFLH, BFSH. 
figContents(17).qPlotLabel{1} = {'semimem_l'};
figContents(18).qPlotLabel{1} = {'semimem_r'};
figContents(19).qPlotLabel{1} = {'bifemlh_l'};
figContents(20).qPlotLabel{1} = {'bifemlh_r'};
figContents(21).qPlotLabel{1} = {'bifemsh_l'};
figContents(22).qPlotLabel{1} = {'bifemsh_r'};
figContents(17).perryAbbr = {'SM', 'ST'};
figContents(18).perryAbbr = {'SM', 'ST'};
figContents(19).perryAbbr = {'BFLH'};
figContents(20).perryAbbr = {'BFLH'};
figContents(21).perryAbbr = {'BFSH'};
figContents(22).perryAbbr = {'BFSH'};
figContents(17).muscleActivationPlotIndices = {1};
figContents(18).muscleActivationPlotIndices = {1};
figContents(19).muscleActivationPlotIndices = {1};
figContents(20).muscleActivationPlotIndices = {1};
figContents(21).muscleActivationPlotIndices = {1};
figContents(22).muscleActivationPlotIndices = {1};
figContents(17).muscleNumbersInPlot = {1, 2};
figContents(18).muscleNumbersInPlot = {1, 2};
figContents(19).muscleNumbersInPlot = {1};
figContents(20).muscleNumbersInPlot = {1};
figContents(21).muscleNumbersInPlot = {1};
figContents(22).muscleNumbersInPlot = {1};
figContents(17).subplotTitle{1} = {'Semimembranosus'};
figContents(18).subplotTitle{1} = {'Semimembranosus'};
figContents(19).subplotTitle{1} = {'Biceps Femoris Long Head'};
figContents(20).subplotTitle{1} = {'Biceps Femoris Long Head'};
figContents(21).subplotTitle{1} = {'Biceps Femoris Short Head'};
figContents(22).subplotTitle{1} = {'Biceps Femoris Short Head'};
figContents(17).subplotAxisLabel{1} = 'muscle activation';
figContents(18).subplotAxisLabel{1} = 'muscle activation';
figContents(19).subplotAxisLabel{1} = 'muscle activation';
figContents(20).subplotAxisLabel{1} = 'muscle activation';
figContents(21).subplotAxisLabel{1} = 'muscle activation';
figContents(22).subplotAxisLabel{1} = 'muscle activation';
figContents(17).subplotRange{1} = [0 1];
figContents(18).subplotRange{1} = [0 1];
figContents(19).subplotRange{1} = [0 1];
figContents(20).subplotRange{1} = [0 1];
figContents(21).subplotRange{1} = [0 1];
figContents(22).subplotRange{1} = [0 1];
figContents(17).subplotTick{1}  = 0:0.5:1;
figContents(18).subplotTick{1}  = 0:0.5:1;
figContents(19).subplotTick{1}  = 0:0.5:1;
figContents(20).subplotTick{1}  = 0:0.5:1;
figContents(21).subplotTick{1}  = 0:0.5:1;
figContents(22).subplotTick{1}  = 0:0.5:1;


% Figures 23-28: Adductors.
figContents(23).qPlotLabel{1} = {'add_brev_l'};
figContents(24).qPlotLabel{1} = {'add_brev_r'};
figContents(25).qPlotLabel{1} = {'add_mag1_l'};
figContents(26).qPlotLabel{1} = {'add_mag1_r'};
figContents(27).qPlotLabel{1} = {'pect_l'};
figContents(28).qPlotLabel{1} = {'pect_r'};
figContents(23).perryAbbr = {'AL'};
figContents(24).perryAbbr = {'AL'};
figContents(25).perryAbbr = {'ADM'};
figContents(26).perryAbbr = {'ADM'};
figContents(27).perryAbbr = {'GR'};
figContents(28).perryAbbr = {'GR'};
figContents(23).muscleActivationPlotIndices = {1};
figContents(24).muscleActivationPlotIndices = {1};
figContents(25).muscleActivationPlotIndices = {1};
figContents(26).muscleActivationPlotIndices = {1};
figContents(27).muscleActivationPlotIndices = {1};
figContents(28).muscleActivationPlotIndices = {1};
figContents(23).muscleNumbersInPlot = {1};
figContents(24).muscleNumbersInPlot = {1};
figContents(25).muscleNumbersInPlot = {1};
figContents(26).muscleNumbersInPlot = {1};
figContents(27).muscleNumbersInPlot = {1};
figContents(28).muscleNumbersInPlot = {1};
figContents(23).subplotTitle{1} = {'Adductor Brevis'};
figContents(24).subplotTitle{1} = {'Adductor Brevis'};
figContents(25).subplotTitle{1} = {'Adductor Magnus'};
figContents(26).subplotTitle{1} = {'Adductor Magnus'};
figContents(27).subplotTitle{1} = {'Pectineus'};
figContents(28).subplotTitle{1} = {'Pectineus'};
figContents(23).subplotAxisLabel{1} = 'muscle activation';
figContents(24).subplotAxisLabel{1} = 'muscle activation';
figContents(25).subplotAxisLabel{1} = 'muscle activation';
figContents(26).subplotAxisLabel{1} = 'muscle activation';
figContents(27).subplotAxisLabel{1} = 'muscle activation';
figContents(28).subplotAxisLabel{1} = 'muscle activation';
figContents(23).subplotRange{1} = [0 1];
figContents(24).subplotRange{1} = [0 1];
figContents(25).subplotRange{1} = [0 1];
figContents(26).subplotRange{1} = [0 1];
figContents(27).subplotRange{1} = [0 1];
figContents(28).subplotRange{1} = [0 1];
figContents(23).subplotTick{1}  = 0:0.5:1;
figContents(24).subplotTick{1}  = 0:0.5:1;
figContents(25).subplotTick{1}  = 0:0.5:1;
figContents(26).subplotTick{1}  = 0:0.5:1;
figContents(27).subplotTick{1}  = 0:0.5:1;
figContents(28).subplotTick{1}  = 0:0.5:1;


% Figures 29-34: Rotators.
figContents(29).qPlotLabel{1} = {'quad_fem_l'};
figContents(30).qPlotLabel{1} = {'quad_fem_r'};
figContents(31).qPlotLabel{1} = {'gem_l'};
figContents(32).qPlotLabel{1} = {'gem_r'};
figContents(33).qPlotLabel{1} = {'peri_l'};
figContents(34).qPlotLabel{1} = {'peri_r'};
figContents(29).perryAbbr = {};
figContents(30).perryAbbr = {};
figContents(31).perryAbbr = {};
figContents(32).perryAbbr = {};
figContents(33).perryAbbr = {};
figContents(34).perryAbbr = {};
figContents(29).muscleActivationPlotIndices = {};
figContents(30).muscleActivationPlotIndices = {};
figContents(31).muscleActivationPlotIndices = {};
figContents(32).muscleActivationPlotIndices = {};
figContents(33).muscleActivationPlotIndices = {};
figContents(34).muscleActivationPlotIndices = {};
figContents(29).muscleNumbersInPlot = {};
figContents(30).muscleNumbersInPlot = {};
figContents(31).muscleNumbersInPlot = {};
figContents(32).muscleNumbersInPlot = {};
figContents(33).muscleNumbersInPlot = {};
figContents(34).muscleNumbersInPlot = {};
figContents(29).subplotTitle{5}  = {'Quadratus Femoris'};
figContents(30).subplotTitle{6}  = {'Quadratus Femoris'};
figContents(31).subplotTitle{7}  = {'Gemelli'};
figContents(32).subplotTitle{8}  = {'Gemelli'};
figContents(33).subplotTitle{9}  = {'Piriformis'};
figContents(34).subplotTitle{10} = {'Piriformis'};
figContents(29).subplotAxisLabel{5}  = 'muscle activation';
figContents(30).subplotAxisLabel{6}  = {};
figContents(31).subplotAxisLabel{7}  = 'muscle activation';
figContents(32).subplotAxisLabel{8}  = {};
figContents(33).subplotAxisLabel{9}  = 'muscle activation';
figContents(34).subplotAxisLabel{10} = {};
figContents(29).subplotRange{5}  = [0 1];
figContents(30).subplotRange{6}  = [0 1];
figContents(31).subplotRange{7}  = [0 1];
figContents(32).subplotRange{8}  = [0 1];
figContents(33).subplotRange{9}  = [0 1];
figContents(34).subplotRange{10} = [0 1];
figContents(29).subplotTick{5}  = 0:0.5:1;
figContents(30).subplotTick{6}  = 0:0.5:1;
figContents(31).subplotTick{7}  = 0:0.5:1;
figContents(32).subplotTick{8}  = 0:0.5:1;
figContents(33).subplotTick{9}  = 0:0.5:1;
figContents(34).subplotTick{10} = 0:0.5:1;


% Figures 35-40: Plantarflexors.
figContents(35).qPlotLabel{1}  = {'med_gas_l'};
figContents(36).qPlotLabel{1}  = {'med_gas_r'};
figContents(37).qPlotLabel{1}  = {'soleus_l'};
figContents(38).qPlotLabel{1}  = {'soleus_r'};
figContents(39).qPlotLabel{1}  = {'flex_dig_l'};
figContents(40).qPlotLabel{1} = {'flex_dig_r'};
figContents(35).perryAbbr = {'GAS'};
figContents(36).perryAbbr = {'GAS'};
figContents(37).perryAbbr = {'SOL', 'PTIB'};
figContents(38).perryAbbr = {'SOL', 'PTIB'};
figContents(39).perryAbbr = {'FDL', 'FHL'};
figContents(40).perryAbbr = {'FDL', 'FHL'};
figContents(35).muscleActivationPlotIndices = {1};
figContents(36).muscleActivationPlotIndices = {1};
figContents(37).muscleActivationPlotIndices = {1};
figContents(38).muscleActivationPlotIndices = {1};
figContents(39).muscleActivationPlotIndices = {1};
figContents(40).muscleActivationPlotIndices = {1};
figContents(35).muscleNumbersInPlot = {1};
figContents(36).muscleNumbersInPlot = {1};
figContents(37).muscleNumbersInPlot = {1, 2};
figContents(38).muscleNumbersInPlot = {1, 2};
figContents(39).muscleNumbersInPlot = {1, 2};
figContents(40).muscleNumbersInPlot = {1, 2};
figContents(35).subplotTitle{1}  = {'Gastrocnemius Medial Head'};
figContents(36).subplotTitle{1}  = {'Gastrocnemius Medial Head'};
figContents(37).subplotTitle{1}  = {'Soleus'};
figContents(38).subplotTitle{1}  = {'Soleus'};
figContents(39).subplotTitle{1}  = {'Flexor Digitorum Longus'};
figContents(40).subplotTitle{1} = {'Flexor Digitorum Longus'};
figContents(35).subplotAxisLabel{1}  = 'muscle activation';
figContents(36).subplotAxisLabel{1}  = {};
figContents(37).subplotAxisLabel{1}  = 'muscle activation';
figContents(38).subplotAxisLabel{1}  = {};
figContents(39).subplotAxisLabel{1}  = 'muscle activation';
figContents(40).subplotAxisLabel{1} = {};
figContents(35).subplotRange{1}  = [0 1];
figContents(36).subplotRange{1}  = [0 1];
figContents(37).subplotRange{1}  = [0 1];
figContents(38).subplotRange{1}  = [0 1];
figContents(39).subplotRange{1}  = [0 1];
figContents(40).subplotRange{1} = [0 1];
figContents(35).subplotTick{1}  = 0:0.5:1;
figContents(36).subplotTick{1}  = 0:0.5:1;
figContents(37).subplotTick{1}  = 0:0.5:1;
figContents(38).subplotTick{1}  = 0:0.5:1;
figContents(39).subplotTick{1}  = 0:0.5:1;
figContents(40).subplotTick{1} = 0:0.5:1;


% Figures 41-46: Dorsiflexors, Evertors.
figContents(41).qPlotLabel{1} = {'tib_ant_l'};
figContents(42).qPlotLabel{1} = {'tib_ant_r'};
figContents(43).qPlotLabel{1} = {'ext_dig_l'};
figContents(44).qPlotLabel{1} = {'ext_dig_r'};
figContents(45).qPlotLabel{1} = {'per_long_l'};
figContents(46).qPlotLabel{1} = {'per_long_r'};
figContents(41).perryAbbr = {'TA', 'EHL'};
figContents(42).perryAbbr = {'TA', 'EHL'};
figContents(43).perryAbbr = {'EDL'};
figContents(44).perryAbbr = {'EDL'};
figContents(45).perryAbbr = {'PERlong', 'PERbrev'};
figContents(46).perryAbbr = {'PERlong', 'PERbrev'};
figContents(41).muscleActivationPlotIndices = {1};
figContents(42).muscleActivationPlotIndices = {1};
figContents(43).muscleActivationPlotIndices = {1};
figContents(44).muscleActivationPlotIndices = {1};
figContents(45).muscleActivationPlotIndices = {1};
figContents(46).muscleActivationPlotIndices = {1};
figContents(41).muscleNumbersInPlot = {1, 2};
figContents(42).muscleNumbersInPlot = {1, 2};
figContents(43).muscleNumbersInPlot = {1};
figContents(44).muscleNumbersInPlot = {1};
figContents(45).muscleNumbersInPlot = {1, 2};
figContents(46).muscleNumbersInPlot = {1, 2};
figContents(41).subplotTitle{1} = {'Tibialis Anterior'};
figContents(42).subplotTitle{1} = {'Tibialis Anterior'};
figContents(43).subplotTitle{1} = {'Extensor Digitorum Longus'};
figContents(44).subplotTitle{1} = {'Extensor Digitorum Longus'};
figContents(45).subplotTitle{1} = {'Peroneus Longus'};
figContents(46).subplotTitle{1} = {'Peroneus Longus'};
figContents(41).subplotAxisLabel{1} = 'muscle activation';
figContents(42).subplotAxisLabel{1} = 'muscle activation';
figContents(43).subplotAxisLabel{1} = 'muscle activation';
figContents(44).subplotAxisLabel{1} = 'muscle activation';
figContents(45).subplotAxisLabel{1} = 'muscle activation';
figContents(46).subplotAxisLabel{1} = 'muscle activation';
figContents(41).subplotRange{1} = [0 1];
figContents(42).subplotRange{1} = [0 1];
figContents(43).subplotRange{1} = [0 1];
figContents(44).subplotRange{1} = [0 1];
figContents(45).subplotRange{1} = [0 1];
figContents(46).subplotRange{1} = [0 1];
figContents(41).subplotTick{1}  = 0:0.5:1;
figContents(42).subplotTick{1}  = 0:0.5:1;
figContents(43).subplotTick{1}  = 0:0.5:1;
figContents(44).subplotTick{1}  = 0:0.5:1;
figContents(45).subplotTick{1}  = 0:0.5:1;
figContents(46).subplotTick{1}  = 0:0.5:1;


% Figures 47-52: Erector spinae, obliques.
figContents(47).qPlotLabel{1} = {'ercspn_l'};
figContents(48).qPlotLabel{1} = {'ercspn_r'};
figContents(49).qPlotLabel{1} = {'intobl_l'};
figContents(50).qPlotLabel{1} = {'intobl_r'};
figContents(51).qPlotLabel{1} = {'extobl_l'};
figContents(52).qPlotLabel{1} = {'extobl_r'};
figContents(47).perryAbbr = {};
figContents(48).perryAbbr = {};
figContents(49).perryAbbr = {};
figContents(50).perryAbbr = {};
figContents(51).perryAbbr = {};
figContents(52).perryAbbr = {};
figContents(47).muscleActivationPlotIndices = {};
figContents(48).muscleActivationPlotIndices = {};
figContents(49).muscleActivationPlotIndices = {};
figContents(50).muscleActivationPlotIndices = {};
figContents(51).muscleActivationPlotIndices = {};
figContents(52).muscleActivationPlotIndices = {};
figContents(47).muscleNumbersInPlot = {};
figContents(48).muscleNumbersInPlot = {};
figContents(49).muscleNumbersInPlot = {};
figContents(50).muscleNumbersInPlot = {};
figContents(51).muscleNumbersInPlot = {};
figContents(52).muscleNumbersInPlot = {};
figContents(47).subplotTitle{1} = {'Erector Spinae'};
figContents(48).subplotTitle{1} = {'Erector Spinae'};
figContents(49).subplotTitle{1} = {'Internal Oblique'};
figContents(50).subplotTitle{1} = {'Internal Oblique'};
figContents(51).subplotTitle{1} = {'External Oblique'};
figContents(52).subplotTitle{1} = {'External Oblique'};
figContents(47).subplotAxisLabel{1} = 'muscle activation';
figContents(48).subplotAxisLabel{1} = 'muscle activation';
figContents(49).subplotAxisLabel{1} = 'muscle activation';
figContents(50).subplotAxisLabel{1} = 'muscle activation';
figContents(51).subplotAxisLabel{1} = 'muscle activation';
figContents(52).subplotAxisLabel{1} = 'muscle activation';
figContents(47).subplotRange{1} = [0 1];
figContents(48).subplotRange{1} = [0 1];
figContents(49).subplotRange{1} = [0 1];
figContents(50).subplotRange{1} = [0 1];
figContents(51).subplotRange{1} = [0 1];
figContents(52).subplotRange{1} = [0 1];
figContents(47).subplotTick{1}  = 0:0.5:1;
figContents(48).subplotTick{1}  = 0:0.5:1;
figContents(49).subplotTick{1}  = 0:0.5:1;
figContents(50).subplotTick{1}  = 0:0.5:1;
figContents(51).subplotTick{1}  = 0:0.5:1;
figContents(52).subplotTick{1}  = 0:0.5:1;

return;
