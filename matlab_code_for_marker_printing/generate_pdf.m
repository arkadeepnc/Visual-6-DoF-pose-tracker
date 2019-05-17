clc ;
clear;
close all;
n = 5; % no of sides of pentagon
s = 2.0; % cm length of side of polygon
marker_size = 1.778;
cur_dir = pwd;

rad_circum_circ = s/(2*sin(pi/n));

nf=(fullfile(cd,'aruco_images')) ;%loading images from a different folder 
S = dir(fullfile(nf,'*.jpg'));
% for k = 1:numel(S)
%     F = fullfile(nf,S(k).name);
%     I = imread(F);
%     imshow(I)
%     S(k).data = I; % optional, save data.
% end

fig = figure;
rows = 6;
cols = 2;
for i = 1:cols
    for j = 1:rows
        cent = [2*rad_circum_circ*i,2*rad_circum_circ*j];
        a = draw_polygon(cent ,rad_circum_circ,5);
        plot(a(:,1),a(:,2),'k')
        hold all
 
        mark_num = rows*(i-1) + (j-1) +1;
        mark_name = fullfile(nf,S(mark_num).name);
 
        img = imread(mark_name);
        img = flipud(img);
%         img = insertText(img,cent+[0,1.5*marker_size], num2str(mark_num),'FontSize',20);
        im = image(img); 
        im.XData = [cent(1)-marker_size/2,cent(1)+marker_size/2]; 
        im.YData = [cent(2)-marker_size/2,cent(2)+marker_size/2]; 

    end
end
colormap(gca, gray(256))
%

% Force MATLAB to render the figure so that the axes
% are created and the properties are updated
drawnow
% Define the axes' 'units' property
% Note: this does not mean that one cm of the axes equals
%  one cm on the axis ticks.  The 'position' property
%  will also need to be adjusted so that these match
set(gca,'units','centimeters')
% Force the x-axis limits and y-axis limits to honor your settings,
% rather than letting the renderer choose them for you
set(gca,'xlimmode','manual','ylimmode','manual')
% Get the axes position in cm, [locationX locationY sizeX sizeY]
% so that we can reuse the locations
axpos = get(gca,'position');
% Use the existing axes location, and map the axes size (in cm) to the
%  axes limits so there is a true size correspondence
set(gca,'position',[0 0 abs(diff(xlim)) abs(diff(ylim))])
% Optional: Since we are forcing the x-axis limits and y-axis limits,
% the print out may not display the desired tick marks. In order to keep
% these, you can select "File-->Preferences-->Figure Copy Template".  Then
% choose "Keep axes limits and tick spacing" in the "Uicontrols and axes"
% Frame.  Click on "Apply to Figure" and then "OK".
% Print the figure to paper in real size.

% Print to a file in real size and look at the result
print(gcf,'-dpng','-r0','sine.png')
print ("FillPageFigure", '-dpdf')