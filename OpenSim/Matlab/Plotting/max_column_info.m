function s=max_column_info(labels,data,columns,scale)

if nargin < 4
	scale = 1;
end

qm=max(abs(data(:,columns)));
qmm=max(qm);
qmi=find(qm==qmm);

s=sprintf('%g (%s)', qmm*scale, labels{columns(qmi)});
