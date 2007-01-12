function labelPrefix = get_emgColumnPrefix(limb, muscle, ref_dataFormat)

labelPrefix = '';

for i=1:length(ref_dataFormat.emgMuscleToColumnPrefix)
	if strcmp(ref_dataFormat.emgMuscleToColumnPrefix{i}{1},muscle)
		labelPrefix = ref_dataFormat.emgMuscleToColumnPrefix{i}{2};
		break;
	end
end

if labelPrefix
	if strcmpi(limb,'R')
		labelPrefix = [labelPrefix '_r'];
	elseif strcmpi(limb,'L')
		labelPrefix = [labelPrefix '_l'];
	else
		error('ERROR');
	end
else
	error(sprintf('getEmgLabelPrefix: did not find limb=''%s'' muscle=''%s''', limb, muscle));
end

return;
