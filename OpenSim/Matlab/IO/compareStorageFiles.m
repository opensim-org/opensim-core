function [err, labels] = compareStorageFiles(file1, file2)
% Utility to compare two storage files that are supposed to contain the
% same data, with perhaps different column orders or a major subset of
% columns.
% USAGE: [error, col_labels] = compareStorageFiles(file1, file2)
%
% Ajay Seth
% Stanford University

S1 = read_motionFile(file1);
S2 = read_motionFile(file2);

nc1 = length(S1.labels);
nc2 = length(S2.labels);

k = 0;
for I = 1:nc1,
    col = strmatch(S1.labels{I}, S2.labels);
    if ~isempty(col),
        %if multiple indices with th same label get column closest I
        [val, ind] = min(abs(col-I));
        k = k+1;
        err(:,k) = [S2.data(:,col(ind))-S1.data(:,I)];
        labels{k} = S1.labels{I};
    end
end