function s=rms_column_info(labels,data,columns,scale)

if nargin < 4
	scale = 1;
end

err = abs(data(:,columns));
qm=sqrt(mean(err.^2)); % compute array of RMS errors (each entry is for one gencoord)
qmm=max(qm); % compute max RMS error over all gencoords
qmi=find(qm==qmm);

s=sprintf('%g (%s)', qmm*scale, labels{columns(qmi)}); % print max RMS error and gencoords with that max RMS error
