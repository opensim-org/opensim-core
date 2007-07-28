function getAndDirectlyOverlayPerryEMG(perryAbbr, muscleActivationPlotIndices, muscleNumbersInPlot, nPlotRows, nPlotCols, ictoEvents, tInfo, timeRange, ref_dataFormat)

% Plot reference EMG on/off data from Perry (1992).
% NOTES: 
%   1) Call overlay_perryEMG after format_muscleExtFromMot
%      to facilitate placement of black bars on subplots.
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
		perryScaled = convert_perryDataToTime(perryData, ...
						   ictoEvents, tInfo.analogRate, limb, tInfo, ref_dataFormat);
        for muscleNum = 1:length(perryAbbr)
            currentPerryScaled = perryScaled(muscleNum);
            currentPlotIndex = muscleActivationPlotIndices{limbIndex, muscleNum};
            muscleNumInPlot = muscleNumbersInPlot{muscleNum};
    		subplot(nPlotRows, nPlotCols, currentPlotIndex);
        	hold on;
            directlyOverlay_perryEMG(currentPerryScaled, muscleNumInPlot, timeRange);
        end
	end
end
