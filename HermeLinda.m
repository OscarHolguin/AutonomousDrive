%% Hermelinda Linda
% Gentic algorithm that finds a route through a map with obstacles

%% Settings
% Map size
mapSizeX = 1000; % [km]
mapSizeY = 1000; % [km]

% Hermelinda initial coordinates
startX = 10; % [km]
startY = 10; % [km]

% Goal coordinates
endX = 950; % [km]
endY = 950; % [km]

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
enableTimeout = 0;
timeoutMinutes = 10;

%% Map
% TODO: Create map with non-overlapping obstacles

%% Initial population
% TODO: Create randomized population

%% Evolution
% TODO:
% - Loop until at least one specimen achieves the goal or timeout

while true
    % % % Evaluation % % %
    % Generate routes from current population
    % Get fitness
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
