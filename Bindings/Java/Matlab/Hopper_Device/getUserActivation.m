function [xy] = getUserActivation(color)


w = [0 5 0 1];
axis(w), hold on, grid on

title(getString(message('SPLINES:resources:axesLabel_WhenYouAreDone')))
pts = line('Xdata',NaN,'Ydata',NaN,'marker','o','LineWidth',1.5,'Color',color);
pts2 = line('Xdata',NaN,'Ydata',NaN,'LineStyle','--','LineWidth',1.5,'Color',color);

maxpnts = 100; xy = zeros(2,maxpnts);
while 1
    for j=2:maxpnts
        try
            [x,y] = ginput(1);
        catch ME
            display('ERROR: ginput failed')
        end
        if isempty(x)||x<w(1)||x>w(2)||y<w(3)||y>w(4)
            xy(1,j) = 5;
            xy(2,j) = xy(2,j-1);
            
            set(pts,'Xdata',xy(1,1:j),'Ydata',xy(2,1:j))
            break;
        end
        xy(:,j) = [x;y];
        if j>1
            set(pts,'Xdata',xy(1,1:j),'Ydata',xy(2,1:j))
            set(pts2,'Xdata',[xy(1,j) 5],'Ydata',[xy(2,j) xy(2,j)])
        else
            set(pts,'Xdata',x,'Ydata',y)
        end
    end
    
    if j>1, break, end
    
end
title(' ')
xy(:,j) = [5;xy(2,j-1)];
xy(:,j+1:maxpnts)=[];

