function [] = overlay_jointMomentFromLiterature(time, qLiterature, color)
% Purpose:  Plots the qLiterature data vs time on the current subplot.
%
% Input:    qLiterature is an array of joint moment values
%           time is an array of corresponding time values
%           color is a string indicating what color the curve should be
%
% CTJ, 08-07, adapted from:
% ASA, 11-05, revised 2-06


% Specify attributes of subplots.
pLineStyle = '-';                       
pLineWidth = 0.5;                  
pLineColor = color;

% Plot data vs time.
if ~isempty(qLiterature)
    hold on;
    p = plot(time, qLiterature);
    set(p, 'LineStyle', pLineStyle, 'LineWidth', ...
             pLineWidth, 'Color', pLineColor);
end
return;
