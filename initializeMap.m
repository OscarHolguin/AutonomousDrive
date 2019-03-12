%% Initialize map for Hermelinda Linda
% Get figure hanlder
f = figure(1);

% Draw contour
rectangle('Position',[0 0 mapSizeX mapSizeY])
hold on

% Draw obstacles
for o = 1:length(obstacles)
    rectangle('Position',[obstacles(o,1:2) obstacleSizeX obstacleSizeY],'EdgeColor','k','FaceColor','k')
end

% Draw start and end points
plot(xStart,yStart,'d','color','b','LineWidth',2)
plot(xEnd,yEnd,'x','color','r','LineWidth',2)

% TODO: Fix title text
title('Title goes here')
set(gca,'FontSize',16);

% Adjust axis size
axis([0 mapSizeX 0 mapSizeY])

% Remove axis lines
set(gca,'XTick',[])
set(gca,'YTick',[])

% Set white background
set(gcf,'Color','w');

% Plot empty route and get handler
routeHandler = plot(zeros(numOfChanges + 2,1),zeros(numOfChanges + 2,1),'LineWidth',2);