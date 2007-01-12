% c3dPreProScript_900045.m
% Purpose:  This script pre-processes experimental data for a 
%           'simulateable' segment of data read from the C3D file 
%           of a Gillette control subject. 
%
% Format for Subject/Trial Information:
%       subject is the 6-digit subject ID ('character array')
%       tInfo is a structure containing the following 'trial info':
%           *.trial  - trial number to analyze ('character array')
%           *.mass   - mass of subject in kg
%           *.speed  - average walking speed during trial in m/s
%           *.FP     - FP #s in the order hit; 
%                       my best guess from visual inspection (cell array)
%           *.limb   - limbs corresponding to FP strikes (cell array)
%           *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%           *.static - trial number of corresponding static trial
%                      (cell array: 1st cell = 'static trial #'
%                                   2nd cell = 'all' or 'min',
%                                   corresponding to available marker set
%       refTrials is a structure containing a list of reference trial #s
%           and other relevant information for computing ensemble-averaged 
%           EMG envelopes:
%             *(n Ref Trials).trial
%                           *.FP
%                           *.limb
%                           *.ictoMatrix
%
%           NOTES:
%           (1) tInfo can have any name, such as 'ss', 'fast', 'slow' ...
%           (2) set ictoMatrix = [] to call get_ictoEvents();
%               this function retrieves the necessary 'IC' and 'TO' events
%               if this information has not been stored previously.
%           (3) set static = {} to write only anatomical markers,
%               and not tracking markers, to file.
%
% ASA, 11-05; revised 12-05


%%% Subject 900045:    REPRESENTATIVE TRIAL SS
subject = '900045';
ss.trial = '15';
ss.mass = 66.0;
ss.speed = 1.12;
ss.FP = {1 2 3 4};
ss.limb = {'R', 'L', 'R' ,'L'};
ss.ictoMatrix = [  384   522   985  1154  1601; ...
                   985  1154  1601  1758  2254; ...
                  1601  1758  2254  2385  2844; ...
                  2254  2385  2844  2990  3474];
ss.static = {'2', 'all'};
refTrials(1).trial = '10';
refTrials(1).FP = {1 2 3 4};
refTrials(1).limb = {'L', 'R', 'L', 'R'};
refTrials(1).ictoMatrix = [  436   558  1027  1172  1661; ...
                            1027  1172  1661  1819  2289; ...
                            1661  1819  2289  2446  2943; ...
                            2289  2446  2943  3101  3573];
refTrials(2).trial = '11';
refTrials(2).FP = {3 4};
refTrials(2).limb = {'R', 'L'};
refTrials(2).ictoMatrix = [1457  1584  2061  2193  2637; ...
                           2061  2193  2637  2798  3231];
refTrials(3).trial = '14';
refTrials(3).FP = {1 2 3};
refTrials(3).limb = {'R', 'L', 'R'};
refTrials(3).ictoMatrix = [ 297   432   909  1027  1495; ...
                            909  1027  1495  1612  2097; ...
                           1495  1612  2097  2221  2718];
refTrials(4).trial = '16';
refTrials(4).FP = {1 2 3 4};
refTrials(4).limb = {'R', 'L', 'R', 'L'};
refTrials(4).ictoMatrix = [  348   477   924  1059  1511; ...
                             924  1059  1511  1646  2127; ...
                            1511  1646  2127  2266  2754; ...
                            2127  2266  2754  2842  3303];
prepro_GilletteControls(subject, ss);
detect_c3dContactEvents(subject, ss);
get_c3dEmgAveOnOff(subject, ss, refTrials);
clear ss refTrials;


 
% %%% Subject 900045:  REPRESENTATIVE TRIAL FAST
% subject = '900045';
% fast.trial = '27';
% fast.mass = 66.0;
% fast.speed = 1.45;
% fast.FP = {3 4};
% fast.limb = {'L', 'R'};
% fast.ictoMatrix = [823   936  1364  1475  1881; ...
%                   1364  1475  1881  2002  2412];
% fast.static = {'2', 'all'};
% refTrials(1).trial = '28';
% refTrials(1).FP = {3 4};
% refTrials(1).limb = {'L', 'R'};
% refTrials(1).ictoMatrix = [1137  1242  1643  1741  2169; ...
%                            1643  1741  2169  2254  2682];
% refTrials(2).trial = '30';
% refTrials(2).FP = {2 3};
% refTrials(2).limb = {'R', 'L'};
% refTrials(2).ictoMatrix = [615   711  1114  1216  1654; ...
%                            1114  1216  1654  1752  2187];
% refTrials(3).trial = '31';
% refTrials(3).FP = {2 3};
% refTrials(3).limb = {'R', 'L'};
% refTrials(3).ictoMatrix = [589   684  1101  1193  1602; ...
%                           1101  1193  1602  1707  2133];
% refTrials(4).trial = '32';
% refTrials(4).FP = {2 3 4};
% refTrials(4).limb = {'R', 'L', 'R'};
% refTrials(4).ictoMatrix = [730   819  1233  1350  1754; ...
%                           1233  1350  1754  1870  2295; ...
%                           1754  1870  2295  2370  2799];
% % prepro_GilletteControls(subject, fast);
% % detect_c3dContactEvents(subject, fast);
% get_c3dEmgAveOnOff(subject, fast, refTrials);
% clear fast refTrials;
% 
% 
% %%% Subject 900045:  REPRESENTATIVE TRIAL SLOW
% subject = '900045';
% slow.trial = '20';
% slow.mass = 66.0;
% slow.speed = 0.72;
% slow.FP = {1 2 3};
% slow.limb = {'L', 'R', 'L'};
% slow.ictoMatrix = [515   747  1250  1531  2042; ...
%                   1250  1531  2042  2298  2781; ...
%                   2042  2298  2781  3031  3528];
% slow.static = {'2', 'all'};
% refTrials(1).trial = '17';
% refTrials(1).FP = {1 2 3};
% refTrials(1).limb = {'L', 'R', 'L'};
% refTrials(1).ictoMatrix = [510   702  1245  1514  2011; ...
%                           1245  1514  2011  2180  2700; ...
%                           2011  2180  2700  2921  3447];
% refTrials(2).trial = '18';
% refTrials(2).FP = {1 2};
% refTrials(2).limb = {'L', 'R'};
% refTrials(2).ictoMatrix = [741   927  1499  1766  2268; ...
%                           1499  1766  2268  2455  2979];
% refTrials(3).trial = '19';
% refTrials(3).FP = {1 2 3};
% refTrials(3).limb = {'L', 'R', 'L'};
% refTrials(3).ictoMatrix = [1135  1359  1847  2033  2561; ...
%                            1847  2033  2561  2796  3285; ...
%                            2561  2796  3285  3495  4023];
% refTrials(4).trial = '21';
% refTrials(4).FP = {1 2};
% refTrials(4).limb = {'L', 'R'};
% refTrials(4).ictoMatrix = [252   522   967  1258  1746; ...
%                            967  1258  1746  2005  2475];
% 
% % prepro_GilletteControls(subject, slow);
% % detect_c3dContactEvents(subject, slow);
% get_c3dEmgAveOnOff(subject, slow, refTrials);
% clear slow refTrials;


% %%% Subject 900045:  REPRESENTATIVE TRIAL SLOW -- ADDITIONAL TRIAL 1
% subject = '900045';
% slow.trial = '18';
% slow.mass = 66.0;
% slow.speed = 0.71;
% slow.FP = {1 2};
% slow.limb = {'L', 'R'};
% slow.ictoMatrix = [741   927  1499  1766  2268; ...
%                   1499  1766  2268  2455  2979];
% slow.static = {'2', 'all'};
% refTrials(1).trial = '17';
% refTrials(1).FP = {1 2 3};
% refTrials(1).limb = {'L', 'R', 'L'};
% refTrials(1).ictoMatrix = [510   702  1245  1514  2011; ...
%                           1245  1514  2011  2180  2700; ...
%                           2011  2180  2700  2921  3447];
% refTrials(2).trial = '19';
% refTrials(2).FP = {1 2 3};
% refTrials(2).limb = {'L', 'R', 'L'};
% refTrials(2).ictoMatrix = [1135  1359  1847  2033  2561; ...
%                            1847  2033  2561  2796  3285; ...
%                            2561  2796  3285  3495  4023];                      
% refTrials(3).trial = '20';
% refTrials(3).FP = {1 2 3};
% refTrials(3).limb = {'L', 'R', 'L'};
% refTrials(3).ictoMatrix = [515   747  1250  1531  2042; ...
%                           1250  1531  2042  2298  2781; ...
%                           2042  2298  2781  3031  3528];                       
% refTrials(4).trial = '21';
% refTrials(4).FP = {1 2};
% refTrials(4).limb = {'L', 'R'};
% refTrials(4).ictoMatrix = [252   522   967  1258  1746; ...
%                            967  1258  1746  2005  2475];
% prepro_GilletteControls(subject, slow);
% detect_c3dContactEvents(subject, slow);
% get_c3dEmgAveOnOff(subject, slow, refTrials);
% clear slow refTrials;
% 
% 
% %%% Subject 900045:  REPRESENTATIVE TRIAL XSLOW
% subject = '900045';
% xslow.trial = '26';
% xslow.mass = 66.0;
% xslow.speed = 0.49;
% xslow.FP = {3 4};
% xslow.limb = {'L', 'R'};
% xslow.ictoMatrix = [3565  4005  4620  5036  5643; ...
%                     4620  5036  5643  6139  6660];
% xslow.static = {'2', 'all'};
% refTrials(1).trial = '23';
% refTrials(1).FP = {3 4};
% refTrials(1).limb = {'L', 'R'};
% refTrials(1).ictoMatrix = [3771  4347  5062  5658  6363; ...
%                            5062  5658  6363  6942  7551];
% % prepro_GilletteControls(subject, xslow);
% % detect_c3dContactEvents(subject, xslow);
% get_c3dEmgAveOnOff(subject, xslow, refTrials);
% clear xslow refTrials;
 

%%% Subject 900045:  'SIMULATEABLE' TRIALS (REJECTED)
% subject = '900045';
% ss.trial = '10';
% ss.mass = 66.0;
% ss.speed = 1.08;
% ss.FP = {1 2 3 4};
% ss.limb = {'L', 'R', 'L', 'R'};
% ss.ictoMatrix = [  436   558  1027  1172  1661; ...
%                   1027  1172  1661  1819  2289; ...
%                   1661  1819  2289  2446  2943; ...
%                   2289  2446  2943  3101  3573];
% plot_c3dTrialvsCycle(subject, ss);
% clear ss;
% 
% subject = '900045';
% ss.trial = '11';
% ss.mass = 66.0;
% ss.speed = 1.21;
% ss.FP = {3 4};
% ss.limb = {'R', 'L'};
% ss.ictoMatrix = [1457  1584  2061  2193  2637; ...
%                  2061  2193  2637  2798  3231];
% plot_c3dTrialvsCycle(subject, ss);
% clear ss;
% 
% subject = '900045';
% ss.trial = '14';
% ss.mass = 66.0;
% ss.speed = 1.19;
% ss.FP = {1 2 3};
% ss.limb = {'R', 'L', 'R'};
% ss.ictoMatrix = [ 297   432   909  1027  1495; ...
%                   909  1027  1495  1612  2097; ...
%                  1495  1612  2097  2221  2718];
% plot_c3dTrialvsCycle(subject, ss);
% clear ss;
% 
% subject = '900045';
% ss.trial = '15';
% ss.mass = 66.0;
% ss.speed = 1.12;
% ss.FP = {1 2 3 4};
% ss.limb = {'R', 'L', 'R' ,'L'};
% ss.ictoMatrix = [  384   522   985  1154  1601; ...
%                    985  1154  1601  1758  2254; ...
%                   1601  1758  2254  2385  2844; ...
%                   2254  2385  2844  2990  3474];
% plot_c3dTrialvsCycle(subject, ss);
% clear ss;
% 
% subject = '900045';
% ss.trial = '16';
% ss.mass = 66.0;
% ss.speed = 1.11;
% ss.FP = {1 2 3 4};
% ss.limb = {'R', 'L', 'R', 'L'};
% ss.ictoMatrix = [  348   477   924  1059  1511; ...
%                    924  1059  1511  1646  2127; ...
%                   1511  1646  2127  2266  2754; ...
%                   2127  2266  2754  2842  3303];
% plot_c3dTrialvsCycle(subject, ss);
% clear ss;
% 
% subject = '900045';
% fast.trial = '27';
% fast.mass = 66.0;
% fast.speed = 1.45;
% fast.FP = {3 4};
% fast.limb = {'L', 'R'};
% fast.ictoMatrix = [823   936  1364  1475  1881; ...
%                   1364  1475  1881  2002  2412];
% plot_c3dTrialvsCycle(subject, fast);
% clear fast;
% 
% subject = '900045';
% fast.trial = '28';
% fast.mass = 66.0;
% fast.speed = 1.53;
% fast.FP = {3 4};
% fast.limb = {'L', 'R'};
% fast.ictoMatrix = [1137  1242  1643  1741  2169; ...
%                    1643  1741  2169  2254  2682];
% plot_c3dTrialvsCycle(subject, fast);
% clear fast;
% 
% subject = '900045';
% fast.trial = '30';
% fast.mass = 66.0;
% fast.speed = 1.48;
% fast.FP = {2 3};
% fast.limb = {'R', 'L'};
% fast.ictoMatrix = [615   711  1114  1216  1654; ...
%                   1114  1216  1654  1752  2187];
% plot_c3dTrialvsCycle(subject, fast);
% clear fast;
% 
% subject = '900045';
% fast.trial = '31';
% fast.mass = 66.0;
% fast.speed = 1.56;
% fast.FP = {2 3};
% fast.limb = {'R', 'L'};
% fast.ictoMatrix = [589   684  1101  1193  1602; ...
%                   1101  1193  1602  1707  2133];
% plot_c3dTrialvsCycle(subject, fast);
% clear fast;
% 
% subject = '900045';
% fast.trial = '32';
% fast.mass = 66.0;
% fast.speed = 1.51;
% fast.FP = {2 3 4};
% fast.limb = {'R', 'L', 'R'};
% fast.ictoMatrix = [730   819  1233  1350  1754; ...
%                   1233  1350  1754  1870  2295; ...
%                   1754  1870  2295  2370  2799];
% plot_c3dTrialvsCycle(subject, fast);
% clear fast;
% 
% subject = '900045';
% slow.trial = '17';
% slow.mass = 66.0;
% slow.speed = 0.79;
% slow.FP = {1 2 3};
% slow.limb = {'L', 'R', 'L'};
% slow.ictoMatrix = [510   702  1245  1514  2011; ...
%                   1245  1514  2011  2180  2700; ...
%                   2011  2180  2700  2921  3447];
% plot_c3dTrialvsCycle(subject, slow);
% clear slow;
% 
% subject = '900045';
% slow.trial = '18';
% slow.mass = 66.0;
% slow.speed = 0.71;
% slow.FP = {1 2};
% slow.limb = {'L', 'R'};
% slow.ictoMatrix = [741   927  1499  1766  2268; ...
%                   1499  1766  2268  2455  2979];
% plot_c3dTrialvsCycle(subject, slow);
% clear slow;
% 
% subject = '900045';
% slow.trial = '20';
% slow.mass = 66.0;
% slow.speed = 0.72;
% slow.FP = {1 2 3};
% slow.limb = {'L', 'R', 'L'};
% slow.ictoMatrix = [515   747  1250  1531  2042; ...
%                   1250  1531  2042  2298  2781; ...
%                   2042  2298  2781  3031  3528];
% plot_c3dTrialvsCycle(subject, slow);
% clear slow;% subject = '900045';
% slow.trial = '19';
% slow.mass = 66.0;
% slow.speed = 0.81;
% slow.FP = {1 2 3};
% slow.limb = {'L', 'R', 'L'};
% slow.ictoMatrix = [1135  1359  1847  2033  2561; ...
%                    1847  2033  2561  2796  3285; ...
%                    2561  2796  3285  3495  4023];
% plot_c3dTrialvsCycle(subject, slow);
% clear slow;

% 
% subject = '900045';
% slow.trial = '21';
% slow.mass = 66.0;
% slow.speed = 0.74;
% slow.FP = {1 2};
% slow.limb = {'L', 'R'};
% slow.ictoMatrix = [252   522   967  1258  1746; ...
%                    967  1258  1746  2005  2475];
% plot_c3dTrialvsCycle(subject, slow);
% clear slow;
% 
% subject = '900045';
% xslow.trial = '23';
% xslow.mass = 66.0;
% xslow.speed = 0.39;
% xslow.FP = {3 4};
% xslow.limb = {'L', 'R'};
% xslow.ictoMatrix = [3771  4347  5062  5658  6363; ...
%                     5062  5658  6363  6942  7551];
% plot_c3dTrialvsCycle(subject, xslow);
% clear xslow;
% 
% subject = '900045';
% xslow.trial = '26';
% xslow.mass = 66.0;
% xslow.speed = 0.49;
% xslow.FP = {3 4};
% xslow.limb = {'L', 'R'};
% xslow.ictoMatrix = [3565  4005  4620  5036  5643; ...
%                     4620  5036  5643  6139  6660];
% plot_c3dTrialvsCycle(subject, xslow);
% clear xslow;
