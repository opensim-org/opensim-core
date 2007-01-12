function [] = overlay_jntAngMomTrialData(gcR, gcL, tInfo, figHandle)
% Purpose:  Overlays plots of joint angles and moments vs gait cycle
%           onto the figure template window specified by figHandle  
%           for all R and L cycles of a single trial read from the 
%           C3D file of a Gillette control subject.
%           
% Input:    gcR and gcL are structures with the following format:
%               *(cycleNum).subject
%               *(cycleNum).trial            .num,   .type 
%               *(cycleNum).video            .rate,  .nframes
%               *(cycleNum).jntcenters()     .label, .data
%               *(cycleNum).jntangles()      .label, .data 
%               *(cycleNum).jntmoments()     .label, .data
%               *(cycleNum).jntpowers()      .label, .data
%               *(cycleNum).analog           .rate,  .ratio
%               *(cycleNum).grf()            .label, .data
%               *(cycleNum).emg()            .label, .data
%           tInfo is a structure containing the following 'trial info':
%               *.trial - trial number to analyze ('character array')
%               *.mass  - mass of subject in kg
%               *.speed - average walking speed during trial in m/s
%               *.FP    - FP #s in the order hit (cell array) 
%               *.limb  - limbs corresponding to FP strikes (cell array)
%               *.ictoMatrix - matrix of ic, oto, oic, to, & icNext events, 
%                               in analog frames, one row per FP hit 
%           figHandle is the handle number of the current figure window
%
% NOTES:    1. This code assumes that gcR and gcL contain data for at least 
%              one cycle (i.e., FP hit) per limb.
%           2. Before returning to the calling function, the user is prompted
%              to select 'print' or 'done' from a menu.
%           3. It remains unclear to me whether the joint moment data in
%              the C3D file is consistently in units of N/mm or N/mm/kg.  
%              This function currently assumes that the data are already 
%              normalized by mass (even though this would be inconsistent 
%              with the units noted in the C3D file); however, it would be 
%              easy to normalize by tInfo.mass if this is ever needed.  
%
% Called Functions:
%           ref_c3dPlotLabels(source)
%           Suptitle(titleString)
%
% ASA, 9-05


% Specify attributes of figure window and specify figure(figHandle)
% as the current figure window.
nPlotRows = 5;                      
nPlotCols = 3;
figure(figHandle);

% Specify attributes of subplots.
rLineStyle = '-';                       % style for plotting R cycle data
rLineWidth = 1.0;                  
rLineColor = 'r';
lLineStyle = '--';                      % style for plotting L cycle data 
lLineWidth = 1.0;                   
lLineColor = 'b';

% Get lists of labels corresponding to the R and L joint angles in 
% structures gcR and gcL.

nAngles = length(gcR(1).jntangles);
for jntangleIndex = 1:nAngles
    RjaLabels{jntangleIndex} = gcR(1).jntangles(jntangleIndex).label;
    LjaLabels{jntangleIndex} = gcL(1).jntangles(jntangleIndex).label;
end

% Get lists of labels corresponding to the R and L joint moments in 
% structures gcR and gcL.
nMoments = length(gcR(1).jntmoments);
for jntmomentIndex = 1:nMoments
    RjmLabels{jntmomentIndex} = gcR(1).jntmoments(jntmomentIndex).label;
    LjmLabels{jntmomentIndex} = gcL(1).jntmoments(jntmomentIndex).label;
end

% Get labels and data indices corresponding to each subplot.
[RpltLabels, RpltCol] = ref_c3dPlotLabels('R');
[LpltLabels, LpltCol] = ref_c3dPlotLabels('L');

% Get number of video frames in each cycle, 
%   and create % gait cycle arrays for video data.
% Store results in a structure with the following format:
%   gaitCycle.R(nRcycles).nVideoFrames, .vCycle
%            .L(nLcycles).nVideoFrames, .vCycle
nRcycles = length(gcR);
nLcycles = length(gcL);
for cycleNum = 1:nRcycles
    gaitCycle.R(cycleNum).nVideoFrames = gcR(cycleNum).video.nframes;
    delta = 100.0/(double(gaitCycle.R(cycleNum).nVideoFrames) - 1);
    gaitCycle.R(cycleNum).vCycle = 0:delta:100;
end
for cycleNum = 1:nLcycles
    gaitCycle.L(cycleNum).nVideoFrames = gcL(cycleNum).video.nframes;
    delta = 100.0/(double(gaitCycle.L(cycleNum).nVideoFrames) - 1);
    gaitCycle.L(cycleNum).vCycle =  0:delta:100;
end

% Retrieve and plot data for each R gait cycle ...
for cycleNum = 1:nRcycles
    for plotNum = 1:12         % plot joint angles
        if plotNum ~= 10       % reserve subplot 10 for legend
            jaIndex = strmatch(RpltLabels{plotNum}, RjaLabels, 'exact');
            colNum = RpltCol(plotNum);
            for rowNum = 1:gaitCycle.R(cycleNum).nVideoFrames
                data(rowNum) = ...   
                    gcR(cycleNum).jntangles(jaIndex).data{rowNum, colNum};
            end
            subplot(nPlotRows, nPlotCols, plotNum);
            hold on;
            r = plot(gaitCycle.R(cycleNum).vCycle, data);
            set(r, 'LineStyle', rLineStyle, 'LineWidth', rLineWidth, ...
                        'Color', rLineColor);
            clear data;
        end
    end
    for plotNum = 13:15       % plot normalized joint moments, if available
        if ~isempty(tInfo.mass) & nMoments > 0
            jmIndex = strmatch(RpltLabels{plotNum}, RjmLabels, 'exact');
            colNum = RpltCol(plotNum);
            for rowNum = 1:gaitCycle.R(cycleNum).nVideoFrames
                data(rowNum) = ...   
                    gcR(cycleNum).jntmoments(jmIndex).data{rowNum, colNum};
            end
            Nm_perKg = data/1000.0;             % convert Nmm/kg to Nm/kg?                                    
            subplot(nPlotRows, nPlotCols, plotNum);
            hold on;
            r = plot(gaitCycle.R(cycleNum).vCycle, Nm_perKg);
            set(r, 'LineStyle', rLineStyle, 'LineWidth', rLineWidth, ...
                        'Color', rLineColor);
            clear data;
        end
    end
end
   
% Retrieve and plot data for each L gait cycle ...
for cycleNum = 1:nLcycles
    for plotNum = 1:12         % plot joint angles
        if plotNum ~= 10       % reserve subplot 10 for legend
            jaIndex = strmatch(LpltLabels{plotNum}, LjaLabels, 'exact');
            colNum = LpltCol(plotNum);
            for rowNum = 1:gaitCycle.L(cycleNum).nVideoFrames
                data(rowNum) = ...   
                    gcL(cycleNum).jntangles(jaIndex).data{rowNum, colNum};
            end
            subplot(nPlotRows, nPlotCols, plotNum);
            hold on;
            l = plot(gaitCycle.L(cycleNum).vCycle, data);
            set(l, 'LineStyle', lLineStyle, 'LineWidth', lLineWidth, ...
                        'Color', lLineColor);
            clear data;
        end
    end
    for plotNum = 13:15       % plot normalized joint moments, if available
        if ~isempty(tInfo.mass) & nMoments > 0
            jmIndex = strmatch(LpltLabels{plotNum}, LjmLabels, 'exact');
            colNum = LpltCol(plotNum);
            for rowNum = 1:gaitCycle.L(cycleNum).nVideoFrames
                data(rowNum) = ...   
                    gcL(cycleNum).jntmoments(jmIndex).data{rowNum, colNum};
            end
            Nm_perKg = data/1000.0;             % convert Nmm/kg to Nm/kg?                                    
            subplot(nPlotRows, nPlotCols, plotNum);
            hold on;
            l = plot(gaitCycle.L(cycleNum).vCycle, Nm_perKg);
            set(l, 'LineStyle', lLineStyle, 'LineWidth', lLineWidth, ...
                        'Color', lLineColor);
            clear data;
        end
    end
end
        
% Add legend.
fpString = 'FP #s Hit:  ';
limbString = 'Order:  ';
for fpHitNum = 1:length(tInfo.FP)
    fpString = strcat(fpString, {' '}, num2str(tInfo.FP{fpHitNum}));
    limbString = strcat(limbString, {' '}, tInfo.limb{fpHitNum});
end
legendString{1} = fpString;
legendString{2} = limbString;
legendString{3} = 'R - solid red';
legendString{4} = 'L - dashed blue';
subplot(nPlotRows, nPlotCols, 10);
for i = 1:length(legendString)
    x = 0;
    y = 1 - 0.2*i;
    text(x, y, legendString{i}, 'FontSize', 8);
end
axis off;

% Add title.
titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
    'Joint Angles and Moments:  Subject ', char(gcR(1).subject), '-',  ...
    tInfo.trial, ', Speed ', tInfo.speed, ' m/s');
Suptitle(titleString);      % MATLAB m-file for adding "super title"

% Query user:  Send to printer?
done = 0;
while ~done
    query = 'Send figure to printer?';
    opt1 = 'print';
    opt2 = 'done';
    userInput = menu(query, opt1, opt2);
    switch userInput
        case 1
            orient(figHandle, 'tall');  
            printCommand = ['print -f', num2str(figHandle), ' -r600'];
            eval(printCommand);
            done = 1;
        case 2
            done = 1;
    end
end
return;
