%% Hermelinda Linda
% Gentic algorithm that finds a route through a map with obstacles

%% Settings
% Map size
mapSizeX = 1000; % [km]
mapSizeY = 1000; % [km]

% Hermelinda initial coordinates
xStart = 10; % [km]
yStart = 10; % [km]

% Goal coordinates
xEnd = 950; % [km]
yEnd = 950; % [km]

% Speed
minSpeed = 10; % [km/h]
maxSpeed = 140; % [km/h]

% Changes
numOfChanges = 50;

% Obstacles
numOfObstacles = 10;
obstacleSizeX = 50; % [km]
obstacleSizeY = 50; % [km]

% Elitism
enableElitism = 1;
elitismFraction = 0.1;

% Population
populationSize = 100;

% Mutation
mutationProbability = 0.0001;

% Timeout
% TODO: IMPLEMENT
enableTimeout = 0;
timeoutMinutes = 10;

%% Map
% Create map with non-overlapping obstacles
obstacles = zeros(numOfObstacles,4);
placedObstacles = 0;
while placedObstacles < numOfObstacles
    % Create new obstacle candidate
    newObstacleX_0 = (mapSizeX - obstacleSizeX) * rand;
    newObstacleX_f = newObstacleX_0 + obstacleSizeX;
    newObstacleY_0 = (mapSizeY - obstacleSizeY) * rand;
    newObstacleY_f = newObstacleY_0 + obstacleSizeY;
    
    % Evaluate candidate against key coordinates
    xStartOverlap = newObstacleX_0 < xStart && xStart < newObstacleX_f;
    yStartOverlap = newObstacleY_0 < yStart && yStart < newObstacleY_f;
    xEndOverlap = newObstacleX_0 < xEnd && xEnd < newObstacleX_f;
    yEndOverlap = newObstacleY_0 < yEnd && yEnd < newObstacleY_f;
    
    if (xStartOverlap && yStartOverlap) || (xEndOverlap && yEndOverlap)
        continue
    end
    
    % Evaluate cantidate on map
    validObstacle = true;
    for o = 1:placedObstacles
        xOverlap_0 = obstacles(o,1) < newObstacleX_0 && newObstacleX_0 < obstacles(o,1) + obstacleSizeX;
        xOverlap_f = obstacles(o,1) < newObstacleX_f && newObstacleX_f < obstacles(o,1) + obstacleSizeX;
        yOverlap_0 = obstacles(o,2) < newObstacleY_0 && newObstacleY_0 < obstacles(o,2) + obstacleSizeY;
        yOverlap_f = obstacles(o,2) < newObstacleY_f && newObstacleY_f < obstacles(o,2) + obstacleSizeY;
        
        if (xOverlap_0 || xOverlap_f) && (yOverlap_0 || yOverlap_f)
            validObstacle = false;
            break
        end
    end
    
    % Add valid candidate
    if validObstacle
        placedObstacles = placedObstacles + 1;
        obstacles(placedObstacles,:) = [newObstacleX_0, newObstacleY_0, newObstacleX_f, newObstacleY_f];
    end
end

%% Initial population
% Allocate memory
theLiving = cell(populationSize,1);
specimenFitness = zeros(populationSize,1);

% Randomize population
for specimen = 1:populationSize
    % Speed
    theLiving{specimen}(:,2) = minSpeed + (maxSpeed - minSpeed) * rand(numOfChanges + 1, 1);
    
    % Direction
    theLiving{specimen}(:,1) = 2 * pi * rand(numOfChanges + 1, 1);
end

%% Evolution
% TODO: Loop until at least one specimen achieves the goal or timeout

route = zeros(numOfChanges + 2, 2);
route(1,:) = [xStart, yStart];

initializeMap

while true
    % % % Evaluation % % %
    for specimen = 1:populationSize
        % Generate route
        survived = true;
        for m = 1:(numOfChanges + 1)
            [deltaX, deltaY] = pol2cart(theLiving{specimen}(m,1),theLiving{specimen}(m,2));
            route(m + 1,:) = route(m,:) + [deltaX, deltaY];
            
            % Check for out of bounds
            outOfBounds = any(route(m + 1,:) < 0) || (route(m + 1,1) > mapSizeX) || (route(m + 1,2) > mapSizeY);
            
            if outOfBounds || detectCrash(obstacles,route(m,:),route(m + 1,:))
                % fitness is closeness from route(movement) to goal
                % TODO: Get fitness
                survived = false;
                break
            end
        end
        
        if survived
            % fitness
        end
        
        % Get fitness
        
%         if any(route(:) < 0) || any(route(:,1) > mapSizeX) || any(route(:,2) > mapSizeY)
%             specimenFitness(specimen) = 0;
%         else
%             
%         end
    end
    
    % Check and save (RAM) the all-time best
    
    % % % Survival of the fittest % % %
    % Acquire targets
    % Kill and substitute via reproduction
    
    % % % Mutations % % %
    
    % % % Breaking mechanisms % % %
    break % TODO: Change this to breaking mechanisms
end

% %% initial movements 
%  for i=1:M 
%      for k=2:Change
%          ptx=xp(i,k-1)+velocity(k-1).*cos(pi/180*rot(i,k-1)); %initial position+velocity*angle
%           pty=yp(i,k-1)+velocity(k-1).*sin(pi/180*rot(i,k-1));
%           while sign(ptx)==-1 ||sign(pty)==-1 %avoid negative path 
%           rot=randi([-180 180],Change);
%           ptx=xp(i,k-1)+velocity(k-1).*cos(pi/180*rot(i,k-1)); 
%           pty=yp(i,k-1)+velocity(k-1).*sin(pi/180*rot(i,k-1));
%           end
%           xp(i,k)=ptx;
%           yp(i,k)=pty;          
%      end
%      plot(xp(i,:),yp(i,:),'--'); 
%      xlim([0 1000]);
%      ylim([0,1000]); 
%      hold on; 
%  end
