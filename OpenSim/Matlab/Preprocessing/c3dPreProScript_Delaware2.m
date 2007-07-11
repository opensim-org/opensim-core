function c3dPreProScript_Delaware2(subjectname, trialname)

if nargin < 1
	trialname = 'ss_walk1';
end

[sInfo, tInfo] = ref_trialInfoDelaware2(subjectname, trialname);

trial = tInfo.(trialname);

%plot icto to test it...
if 0
q = process_grf(trial.ancFile,'','',0);
build_delaware_ictoMatrix(trial.contactRanges, trial.limb{1}, trial.FP{1}, q);
return;
end

%plot emg for testing
if 0
c=read_c3dFile(trial.c3dFile, ref_dataFormatDelaware);
for i=1:length(c.emg)
	pro=process_emgChannel(c,i);
	if mod((i-1),8)==0
		figure;
	end
	subplot(4,2,mod(i-1,8)+1);
	plot(pro.nrect,'g');
	axis([1 length(pro.nrect) 0 1]);
	hold on
	plot(pro.nlow,'b','LineWidth',2);
	[limb, muscle] = get_emgLabels(i, ref_dataFormatDelaware);
	title(sprintf('%s - %s',muscle,limb));
	hold off
end
return;
end

ss.mass = sInfo.mass;
ss.trial = trial.trial;
ss.speed = trial.speed;
ss.ictoMatrix = trial.ictoMatrix;
ss.FP = trial.FP;
ss.limb = trial.limb;
ss.static = {};
refTrials={};

fileNames = {trial.c3dFile};

%prepro_GilletteControls(sInfo.subject, ss, ref_dataFormatDelaware, fileNames{1});
%detect_c3dContactEvents(sInfo.subject, ss, ref_dataFormatDelaware);
get_c3dEmgAveOnOff(sInfo.subject, ss, refTrials, ref_dataFormatDelaware, fileNames);
clear ss refTrials;
