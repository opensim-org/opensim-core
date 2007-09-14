function s=mean_rms_column_info(data,columns,scale)

if nargin < 4
	scale = 1;
end

err = abs(data(:,columns));
qm=sqrt(mean(err.^2)); % compute array of RMS errors (each entry is for one gencoord)
qmm=mean(qm); % compute mean RMS error over all gencoords

s=sprintf('%g', qmm*scale); % print mean RMS error and gencoords with that mean RMS error
