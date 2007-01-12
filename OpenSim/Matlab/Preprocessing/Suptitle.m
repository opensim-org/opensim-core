function hout=Suptitle(str)
%SUPTITLE:   Puts a title above all subplots (a "super title").
%	SUPTITLE('text') adds text to the top of the figure
%	above all subplots. Use this function after all subplot commands.
%
% Warning:  If the figure or axis units are non-default, 
%           this function will break.
%
% Drea Thomas 6/15/95 drea@mathworks.com
% Define parameters used to position the supertitle:
plotregion = .97;   % amount of figure window devoted to subplots
                    % was 0.92   
                    
titleypos = 0.98;   % Y-position of title in normalized coordinates
                    % was 0.95
                    
fs = get(gcf,'defaultaxesfontsize')+4;      % fontsize 
fudge = 0.2;        % fudge factor to adjust y spacing between subplots
                    % was 1
                    
haold = gca;
figunits = get(gcf,'units');
% Get the (approximate) difference between full height 
% (plot + title + xlabel) and bounding rectangle.
	if (~strcmp(figunits,'pixels'))
		set(gcf,'units','pixels');
		pos = get(gcf,'position');
		set(gcf,'units',figunits);
    else
		pos = get(gcf,'position');
	end
	ff = (fs-4)*1.27*5/pos(4)*fudge;
        % The 5 here reflects about 3 characters of height below
        % an axis and 2 above. 1.27 is pixels per point.
% Determine the bounding rectange for all the plots
h = findobj(gcf,'Type','axes');     % change suggested by Stacy J. Hills
max_y=0;
min_y=1;
oldtitle =0;
for i=1:length(h),
	if (~strcmp(get(h(i),'Tag'),'suptitle')),
		pos=get(h(i),'pos');
		if (pos(2) < min_y), min_y=pos(2)-ff/5*3;end;
		if (pos(4)+pos(2) > max_y), max_y=pos(4)+pos(2)+ff/5*2;end;
	else,
		oldtitle = h(i);
	end
end
if max_y > plotregion,
	scale = (plotregion-min_y)/(max_y-min_y);
	for i=1:length(h),
		pos = get(h(i),'position');
		pos(2) = (pos(2)-min_y)*scale+min_y;
		pos(4) = pos(4)*scale-(1-scale)*ff/5*3;
		set(h(i),'position',pos);
	end
end
np = get(gcf,'nextplot');
set(gcf,'nextplot','add');
if (oldtitle),
	delete(oldtitle);
end
ha=axes('pos',[0 1 1 1],'visible','off','Tag','suptitle');
ht=text(.5,titleypos-1,str);
set(ht,'horizontalalignment','center','fontsize',fs, 'Interpreter', 'none');
set(gcf,'nextplot',np);
axes(haold);
if nargout,
	hout=ht;
end
