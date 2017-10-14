function osimTable = UpdateMarkerNames(osimTable)
% UpdateMarkerNames used to replace illegal MatLab Struct char's from table
% column names. Function replaces illegal char's with underscore. 

% Import Java Lib's
import org.opensim.modeling.*
% Define set of illegal chars
illegalChars = [{' '} {'*'} {'.'} {'/'} {'-'}];
% Get a reference to the table column labels
labels = osimTable.getColumnLabels();
% Check each label name and replace any illegal char's
for i = 0 : labels.size() - 1
    label = char(labels.get(i));
    % replace illegal chars with underscore
    for u = 1 : length(illegalChars)
        if ~isempty(strfind(label, illegalChars{u}))
            disp_label = strrep(label,illegalChars{u}, '_');
            disp(['Illegal Coloumn label. ' label ' changed to ' disp_label ]);
            label = disp_label;
        end
    end
    labels.set(i,label);
end
% Set the new column labels
osimTable().setColumnLabels(labels)
end