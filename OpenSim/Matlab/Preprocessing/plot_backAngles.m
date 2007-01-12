function [] = plot_backAngles(backAngles, pelvAnglesNmodel, ...
                               sternumAngles, trunk_rz, vTime, figHandle);
% Purpose:  Generates plots of: 
%           (1) the computed back extension, bending, and rotation angles
%               in the model CS.  
%           (2) the measured pelvis tilt, obliquity, and rotation angles
%               in the model CS.
%           (3) the angle between the sternum (sternal notch - xyphoid) 
%               and the vertical axis in the sagittal plane, as computed
%               for the model.
%           (4) the angle between the sternum (sternal notch - xyphoid) 
%               and the vertical axis in the sagittal plane, as computed
%               from the measured marker data.
%
% Input:    backAngles describes the orientation of the model's trunk CS
%             with respect to the pelvis CS:
%                *.extension(nVideoFrames)
%                 .bending(")
%                 .rotation(")
%           pelvAnglesNmodel describes the orientation of the pelvis 
%             with respect to the lab, in the model CS
%                *.tilt(nVideoFrames)
%                 .obliquity(")
%                 .rotation(")
%            sternumAngles describes the angle between the sternum 
%             and the vertical axis in the sagittal plane
%             (+ if sternum is tilted anteriorly wrt vertical axis):
%                *.model(1)
%                *.expt(nVideoFrames)
%           trunk_rz describes the orientation of the trunk CS, from the
%               model markers, to the model CS (rotation about model Z)
%           vTime is an array of time values corresponding to video frames
%           figHandle is the number of the current figure window
%
% ASA, 10-05


% Specify attributes of figure window.
nPlotRows = 2;                      
nPlotCols = 3; 
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.15*scnwidth, 0.05*scnheight, 0.8*scnwidth, 0.8*scnheight];
figColor = 'w';                
                   
% Specify attributes of subplots.
tFontName = 'helvetica';            % attributes of subplot titles
tFontSize = 9;
tVerticalAlignment = 'middle';
aFontName = 'helvetica';           % attributes of axis labels and ticks
aFontSize = 8;
aTickDir = 'out';
bLineStyle = '-';                  % style for computed back angles
bLineWidth = 1.5;                  
bLineColor = 'r';
pLineStyle = '--';                  % style for measured pelvis angle
pLineWidth = 1.5;                  
pLineColor = 'k';
cLineStyle = '-';                  % style for computed sternum angle
cLineWidth = 1.5;                  
cLineColor = 'r';
mLineStyle = '-';                  % style for measured sternum angle
mLineWidth = 1.5;                  
mLineColor = 'k';
qLineStyle = ':';                  % style for -mean(pelvic tilt)
qLineWidth = 1.5;
qLineColor = 'k';
zLineStyle = '-';                  % attributes of zero line
zLineWidth = 0.5;
zLineColor = [0.3, 0.3, 0.3];

% Specify time axis limits.
tmin = 0;
tmax = max(vTime);
tval = tmin:0.5:tmax;

% Specify zero line.
zeroT = [tmin tmax];
zeroY = [0 0];

% Generate figure window.
figure(figHandle);    
clf;
set(gcf, 'Position', figPos, 'Color', figColor);

% Plot lateral bending vs time; overlay pelvis list for reference.
subplot(nPlotRows, nPlotCols, 1);
    b = plot(vTime, backAngles.bending);
    set(b, 'LineStyle', bLineStyle, 'LineWidth', bLineWidth, ...
            'Color', bLineColor);
    hold on;
    p = plot(vTime, pelvAnglesNmodel.obliquity);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    hold on;
    z = plot(zeroT, zeroY);                      % add zero line
    set(z, 'LineStyle', zLineStyle, 'LineWidth', zLineWidth, ...
            'Color', zLineColor);
    title('trunk bending, pelvis list (R down+)');
        t = get(gca, 'title');
        set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
            'VerticalAlignment', tVerticalAlignment);
    xlabel('time (s)');
        a = get(gca, 'xlabel');
        set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    ylabel('joint angle (deg)');
        a = get(gca, 'ylabel');
        set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, ...
        'FontName', aFontName, 'FontSize', aFontSize, ...
        'TickDir', aTickDir);
    set(gca, 'ylim', [-30 30], 'ytick', [-30:20:30], ...
        'FontName', aFontName, 'FontSize', aFontSize, ...
        'TickDir', aTickDir);
    set(gca, 'Box', 'off');
            
% Plot back extension vs time; overlay pelvis tilt for reference. 
% Also overlay -mean(pelvic tilt)
subplot(nPlotRows, nPlotCols, 2);
    b = plot(vTime, backAngles.extension);
    set(b, 'LineStyle', bLineStyle, 'LineWidth', bLineWidth, ...
            'Color', bLineColor);
    hold on;
    p = plot(vTime, pelvAnglesNmodel.tilt);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    hold on;
    meanTilt = mean(pelvAnglesNmodel.tilt);
    q = plot(zeroT, [-1*meanTilt -1*meanTilt]);
    set(q, 'LineStyle', qLineStyle, 'LineWidth', qLineWidth, ...
            'Color', qLineColor);
    hold on; 
    z = plot(zeroT, zeroY);                      % add zero line
    set(z, 'LineStyle', zLineStyle, 'LineWidth', zLineWidth, ...
            'Color', zLineColor);
    title('trunk extension, pelvis tilt (post+)');
        t = get(gca, 'title');
        set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
            'VerticalAlignment', tVerticalAlignment);
    xlabel('time (s)');
        a = get(gca, 'xlabel');
        set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, ...
        'FontName', aFontName, 'FontSize', aFontSize, ...
        'TickDir', aTickDir);
    set(gca, 'ylim', [-30 30], 'ytick', [-30:20:30], ...
        'FontName', aFontName, 'FontSize', aFontSize, ...
        'TickDir', aTickDir);
    set(gca, 'Box', 'off');

        
% Plot back rotaton vs time; overlay pelvis rotation for reference.  
subplot(nPlotRows, nPlotCols, 3);
    b = plot(vTime, backAngles.rotation);
    set(b, 'LineStyle', bLineStyle, 'LineWidth', bLineWidth, ...
            'Color', bLineColor);
    hold on;
    p = plot(vTime, pelvAnglesNmodel.rotation);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', pLineWidth, ...
            'Color', pLineColor);
    hold on;
    z = plot(zeroT, zeroY);                      % add zero line
    set(z, 'LineStyle', zLineStyle, 'LineWidth', zLineWidth, ...
            'Color', zLineColor);
    title('trunk rotation, pelvis rotation (R int+)');
        t = get(gca, 'title');
        set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
            'VerticalAlignment', tVerticalAlignment);
    xlabel('time (s)');
        a = get(gca, 'xlabel');
        set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, ...
        'FontName', aFontName, 'FontSize', aFontSize, ...
        'TickDir', aTickDir);
    set(gca, 'ylim', [-30 30], 'ytick', [-30:20:30], ...
        'FontName', aFontName, 'FontSize', aFontSize, ...
        'TickDir', aTickDir);
    set(gca, 'Box', 'off');
                        
        
% Plot sternum angles for reference.
subplot(nPlotRows, nPlotCols, 5);
    c = plot([tmin tmax], [sternumAngles.model sternumAngles.model]);
    set(c, 'LineStyle', cLineStyle, 'LineWidth', cLineWidth, ...
            'Color', cLineColor);
    hold on;
    m = plot(vTime, sternumAngles.expt);
    set(m, 'LineStyle', mLineStyle, 'LineWidth', mLineWidth, ...
            'Color', mLineColor);
    hold on;
    z = plot(zeroT, zeroY);                      % add zero line
    set(z, 'LineStyle', zLineStyle, 'LineWidth', zLineWidth, ...
            'Color', zLineColor);
    title('sternum angle (post+)');
        t = get(gca, 'title');
        set(t, 'FontName', tFontName, 'FontSize', tFontSize, ...
            'VerticalAlignment', tVerticalAlignment);
    xlabel('time (s)');
        a = get(gca, 'xlabel');
        set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    ylabel('angle from vertical in sagittal plane (deg)');
        a = get(gca, 'ylabel');
        set(a, 'FontName', aFontName, 'FontSize', aFontSize);
    set(gca, 'xlim', [tmin tmax], 'xtick', tval, ...
        'FontName', aFontName, 'FontSize', aFontSize, ...
        'TickDir', aTickDir);
    set(gca, 'ylim', [-10 40], 'ytick', [0:20:40], ...
        'FontName', aFontName, 'FontSize', aFontSize, ...
        'TickDir', aTickDir);
    set(gca, 'Box', 'off');
   
% Add notes. 
 subplot(nPlotRows, nPlotCols, 4);
    legendString{1} = 'pelvis angles: black dashed';
    legendString{2} = '-1*pelvis tilt: black dotted';
    legendString{3} = 'back angles: red solid';
    legendString{4} = strcat({'trunk Rz for model (deg):  '}, ...
                                     num2str(round(trunk_rz)));
    for i = 1:length(legendString)
        textPosX = 0.01;
        textPosY = 1 - 0.12*i;
        text(textPosX, textPosY, legendString{i}, 'FontSize', 8);
    end
  axis off;
  
% Leave subplot6 empty.
subplot(nPlotRows, nPlotCols, 6);
axis off;
return;