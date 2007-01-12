function [] = create_emgEnsembleAveFigs(emgEnsembleAve, cycle, ...
                                            TOI_emgCyc, tInfo, nChannels, ref_dataFormat);
% Purpose:  Plots ensemble-averaged EMG envelopes +/- 1 SD for each of
%           nChannels and overlays EMG data for the trial of interest.
%           Reference EMG on/off times from Perry (1992) are plotted 
%           for comparison.
%
% Input:    emgEnsembleAve is a structure with the following format:
%               *(emgChannel).limb 
%                            .muscle 
%                            .envAve
%                            .envSD
%           cycle is an array of % gait cycle vales corresponding to
%               emgEnsembleAve
%           TOI_emgCyc is a structure with the following format,
%             corresponding to the trial of interest, in analog frames:
%               *.R{cycleNum}{emgChannel}
%               *.L{cycleNum}{emgChannel}
%                       .raw, .band, .rect, .low, .nrect, .nlow
%               *.subject - subject number  
%               *.trial   - trial number
%           tInfo is a structure containing the following 'trial info',
%               in addition to other information:
%               *.speed - average walking speed during trial in m/s
%           nChannels specifies the number of EMG channels
%               
% Called Functions:
%   plot_emgEnsembleAve(emgEnsembleAve, cycle)
%   plot_emgEnvCycle(emgTOI, emgChannel)
%   get_emgTimingFromPerry(muscAbbr)
%   plot_emgPerryCycle(perryData)
%   Suptitle(titleString)
%
% ASA, 12-05


% Specify attributes of figure windows.
nPlotRows = 3;                      
nPlotCols = 1;
nSubPlots = nPlotRows * nPlotCols;  
set(0, 'Units', 'pixels');          
scnsize = get(0, 'ScreenSize');
scnwidth = scnsize(3);
scnheight = scnsize(4);
figPos = [0.20*scnwidth, 0.05*scnheight, 0.70*scnwidth, 0.85*scnheight];
figColor = 'w';

% For each analog EMG channel ...
for emgChannel = ref_dataFormat.emgChannelsOfInterest

    % Generate figure window.
    figHandle = emgChannel;   
    figure(figHandle);
    clf;
    set(gcf, 'Position', figPos, 'Color', figColor);
    
    % Plot ensemble-averaged EMG data.
    subplot(nPlotRows, nPlotCols, 1);
    plot_emgEnsembleAve(emgEnsembleAve(emgChannel), cycle);

    % Plot trial-specific EMG data.
    subplot(nPlotRows, nPlotCols, 2);
    if strcmpi(emgEnsembleAve(emgChannel).limb, 'R')
        plot_emgEnvCycle(TOI_emgCyc.R, emgChannel);
    elseif strcmpi(emgEnsembleAve(emgChannel).limb, 'L')
        plot_emgEnvCycle(TOI_emgCyc.L, emgChannel);
    end
    
    % Plot reference data from Perry (1992).
    subplot(nPlotRows, nPlotCols, 3);
    clear perryData;
    nRefMuscles = length(ref_dataFormat.muscRefList{emgChannel});
    for refNum = 1:nRefMuscles
        perryData(refNum) = ...
            get_emgTimingFromPerry(ref_dataFormat.muscRefList{emgChannel}{refNum});
    end
	if nRefMuscles
		plot_emgPerryCycle(perryData);
	end
    
    % Add title.
    titleString = sprintf('%s%s%s%s%s%3.2f%s', ...
        'Comparison of EMG Envelopes:  Subject ', ...
        char(TOI_emgCyc.subject), '-', tInfo.trial, ', Speed ', ...
        tInfo.speed, ' m/s');
    Suptitle(titleString);      % MATLAB m-file for adding "super title"

    % Query user.
    done = 0;
	quit = 0;
    while ~done
        query = 'Send figure to printer?';
        opt1 = 'print';
        opt2 = 'done';
        opt3 = 'quit';
        userInput = menu(query, opt1, opt2, opt3);
        switch userInput
            case 1
                orient(figHandle, 'tall');  
                printCommand = ...
                   ['print -f', num2str(figHandle), ' -r600'];
                eval(printCommand);
                done = 1;
            case 2
                done = 1;
			case 3
				quit = 1;
				done = 1;
        end
    end

	if quit
		break;
	end
end
return;
