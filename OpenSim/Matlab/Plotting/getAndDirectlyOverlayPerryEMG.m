function getAndDirectlyOverlayPerryEMG(perryAbbr, muscleActivationPlotIndices, muscleNumbersInPlot, nPlotRows, nPlotCols, ictoEvents, tInfo, timeRange, ref_dataFormat)

% Plot reference EMG on/off data from Perry (1992).
% NOTES: 
%   1) Call overlay_perryEMG after format_muscleExtFromMot
%      to facilitate placement of bars on subplots.
%   2) I'm calling some functions created for detect_c3dEmgOnOff(),
%      so the argument lists aren't optimized for this application...
if ~isempty(perryAbbr)
	for refNum = 1:length(perryAbbr)
		perryData(refNum) = get_emgTimingFromPerry(perryAbbr{refNum});
	end

	for limbIndex = 1:2
		switch limbIndex
			case 1
				limb = 'L';
			case 2 
				limb = 'R';
		end
		clear perryScaled;
        timeAxisIsPercentGc = strcmpi( tInfo.timeAxisLabel, 'percent of gait cycle' );
        perryScaled = convert_perryDataToTime(perryData, ...
                           ictoEvents, tInfo.analogRate, limb, tInfo, ref_dataFormat);
        if timeAxisIsPercentGc
            % Convert perryScaled to % gait cycle
            numberOfOnOffMuscles = length( perryScaled );
            for i = 1:numberOfOnOffMuscles
                sizeOfOnOff = size( perryScaled(i).onoff );
                numberOfColumnsInOnOff = sizeOfOnOff(2);
                for j = 1:numberOfColumnsInOnOff
                    % Convert current column to % gait cycle
                    perryScaled(i).onoff(:,j) = convert_timeToCycle( perryScaled(i).onoff(:,j), ...
                        tInfo.gcLimb, tInfo, ictoEvents, tInfo.analogRate, ref_dataFormat );
                end
                sizeOfOnOffAlt = size( perryScaled(i).onoffAlt );
                numberOfColumnsInOnOffAlt = sizeOfOnOffAlt(2);
                for j = 1:numberOfColumnsInOnOffAlt
                    % Convert current column to % gait cycle
                    perryScaled(i).onoffAlt(:,j) = convert_timeToCycle( perryScaled(i).onoffAlt(:,j), ...
                        tInfo.gcLimb, tInfo, ictoEvents, tInfo.analogRate, ref_dataFormat );
                end
            end
        end
        for muscleNum = 1:length(perryAbbr)
            currentPerryScaled = perryScaled(muscleNum);
            currentPlotIndex = muscleActivationPlotIndices{limbIndex, muscleNum};
            muscleNumInPlot = muscleNumbersInPlot{muscleNum};
    		subplot(nPlotRows, nPlotCols, currentPlotIndex);
        	hold on;
            directlyOverlay_perryEMG(currentPerryScaled, muscleNumInPlot, timeRange, tInfo.fgColor);
        end
	end
end
