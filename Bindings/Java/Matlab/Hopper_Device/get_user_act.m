function [xy] = get_user_act()

% GET_USER_ACT 

w = [0 5 0 1];
axis(w), hold on, grid on

pts = line('Xdata',NaN,'Ydata',NaN,'marker','o');

maxpnts = 100; xy = zeros(2,maxpnts);
while 1
    for j=1:maxpnts
        try 
            [x,y] = ginput(1);
        catch ME
            display('ERROR: ginput failed')
        end
      if isempty(x)||x<w(1)||x>w(2)||y<w(3)||y>w(4), break, end
      xy(:,j) = [x;y];
      if j>1
         set(pts,'Xdata',xy(1,1:j),'Ydata',xy(2,1:j))
      else
         set(pts,'Xdata',x,'Ydata',y)
         
      end
   end
   if j>1, break, end

end 

xy(:,j:maxpnts)=[];

