function columns = find_columns_by_label(labels, label_selection)
% label_selection either a cell array of strings or a single string

n = length(labels);

if isa(label_selection, 'char')
	fixed_label_selection = { label_selection };
else
	fixed_label_selection = label_selection;
end

matches = cell(1,n);
for i=1:n
	matches{i} = false;
end
for i=1:length(fixed_label_selection)
	grepresult = regexp(labels, fixed_label_selection{i});
	for j=1:n
		if length(grepresult{j})
			matches{j} = true;
		end
	end
end	

columns = [];
for i=1:n
	if matches{i}
		columns = [columns i];
	end
end
