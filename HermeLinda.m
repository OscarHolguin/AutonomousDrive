%% Hermelinda Linda
% Gentic algorithm that finds a route through a map with obstacles

%% Settings
% Generations
enableMaxGenerations = 1;
maxGenerations = 20000;

% Timeout
enableTimeout = 1;
timeoutMinutes = 5;

% Map size
mapSizeX = 1000; % [km]
mapSizeY = 1000; % [km]

% Hermelinda initial coordinates
xStart = 10; % [km]
yStart = 10; % [km]

% Goal
xEnd = 950; % [km]
yEnd = 950; % [km]
acceptanceRadius = 10; % [km]

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
enableElitism = 0;
elitismFraction = 0.05;

% Population
populationSize = 200;

paternalProbability = 0.6;

% Mutation
mutationProbability = 0.0005;
mutationRangeFraction = 0.3;

%% Calculated settings
% Elitism
if enableElitism == 0
    elitismFraction = 0;
end
eliteAmount = floor(populationSize * elitismFraction);
nonEliteIdx = (eliteAmount + 1):populationSize;

% Timeout
timeout = timeoutMinutes * 60;

% Goal
goal = [xEnd yEnd];

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
bestRoute = NaN(size(route));
bestRoute(1,:) = route(1,:);
bestFitness = 0;

initializeMap

% TODO: Move tic
tic

generation = 1;
goalAchieved = false;
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
                % Get fitness, closest distance before out of bounds / crash
                specimenFitness(specimen) = 1 / sqrt(sum( (goal - route(m,:)) .^ 2 ));
                survived = false;
                
                % Check if the goal was achieved
                if sqrt(sum( (goal - route(m,:)) .^ 2 )) <= acceptanceRadius
                    goalAchieved = true;
                    changeIdx = m;
                end
                
                % End route
                break
            end
        end
        
        % Get the fitness for those who survived
        if survived
           specimenFitness(specimen) = 1 / sqrt(sum( (goal - route(end,:)) .^ 2 ));
           
           % Check if the goal was achieved
           if sqrt(sum( (goal - route(end,:)) .^ 2 )) <= acceptanceRadius
               goalAchieved = true;
               changeIdx = numOfChanges + 1;
               break
           end
        elseif goalAchieved
            break
        end
    end
    
    % Sort population
    [~,bestIdx] = sort(specimenFitness,'descend');
    
    % Check and save (RAM) the all-time best
    if specimenFitness(bestIdx(1)) > bestFitness
        % Save the data into memory
        bestFitness = specimenFitness(bestIdx(1));
        bestSpecimen = theLiving{bestIdx(1)};
        bestGeneration = generation;
        
        % Get route
        specimen = bestIdx(1);
        bestRoute(2:end,:) = NaN;
        for m = 1:(numOfChanges + 1)
            [deltaX, deltaY] = pol2cart(theLiving{specimen}(m,1),theLiving{specimen}(m,2));
            bestRoute(m + 1,:) = bestRoute(m,:) + [deltaX, deltaY];
            
            % Check for out of bounds
            outOfBounds = any(bestRoute(m + 1,:) < 0) || (bestRoute(m + 1,1) > mapSizeX) || (bestRoute(m + 1,2) > mapSizeY);
            
            % End route calculation
            if outOfBounds || detectCrash(obstacles,bestRoute(m,:),bestRoute(m + 1,:))
                bestRoute(m + 1,:) = [NaN NaN];
                break
            end
        end
        
        % Update route data
        routeHandler.XData = bestRoute(:,1);
        routeHandler.YData = bestRoute(:,2);
        
        % Pause to update display
        pause(0)
        
        % Break if achieved goal
        if goalAchieved
            break
        end
    end
    
    
    % % % Survival of the fittest % % %
    % Acquire targets
    killed = 0;
    killIdx = false(1,populationSize);
    while killed < populationSize / 2
        % Go from unfittest to fittest
        unfitToFit = flip(bestIdx(nonEliteIdx)); % Only the non elite
        
        for s = 1:(populationSize - eliteAmount)
            specimen = unfitToFit(s);
            % Always include probability of survival (also for unfittest, elitism exception)
            if (killIdx(specimen) == false) && (exp(find(specimen == bestIdx,1)/populationSize - 1.1) >= rand)
                % Acquire target
                killIdx(specimen) = true;
                
                % Increase counter and check
                killed = killed + 1;
                if killed >= populationSize / 2
                    break
                end
            end
        end
    end
    
    % Kill and substitute via reproduction
    replaceWithBaby = find(killIdx);
    for newBaby = 1:length(replaceWithBaby)
        % Only search in the ones not to be killed
        allowedIdx = ~killIdx;
        
        % Get first parent
        firstParentIdx = 0;
        lookingForFirstParent = true;
        while lookingForFirstParent
            % The most fit are the firsts in line
            orderedCandidatesIdx = bestIdx(allowedIdx);
            
            for candidate = 1:length(orderedCandidatesIdx)                
                if rand < paternalProbability
                    lookingForFirstParent = false;
                    firstParentIdx = orderedCandidatesIdx(candidate);
                    allowedIdx(firstParentIdx) = false;
                    break
                end
            end
        end
        
        % Get second parent
        secondParentIdx = 0;
        lookingForSecondParent = true;
        while lookingForSecondParent
            % The most fit are the firsts in line
            orderedCandidatesIdx = bestIdx(allowedIdx);
            
            for candidate = 1:length(orderedCandidatesIdx)
                if rand < paternalProbability
                    lookingForSecondParent = false;
                    secondParentIdx = orderedCandidatesIdx(candidate);
                    break
                end
            end
        end
        
        % Make baby
        firstParent = randi([0, 1], [numOfChanges + 1, 1]);
        secondParent = ~firstParent;
        theLiving{replaceWithBaby(newBaby)} = firstParent .* theLiving{firstParentIdx} + secondParent .* theLiving{secondParentIdx};
    end
    
    % % % Mutations % % %
    for specimen = 1:populationSize
        % Indices
        mutate = rand(numOfChanges + 1, 1) < mutationProbability;
        
        % Speed mutations
        speedMutation = mutationRangeFraction * (maxSpeed - minSpeed) * rand(numOfChanges + 1, 1);
        % Direction mutations
        directionMutation = mutationRangeFraction * 2 * pi * rand(numOfChanges + 1, 1);
        % Mutation sign
        mutationSign = randi([0, 1],[numOfChanges + 1, 2]);
        mutationSign(mutationSign == 0) = -1;
        
        % Mutate
        theLiving{specimen}(mutate,1) = theLiving{specimen}(mutate,1) + mutationSign(mutate,1) .* speedMutation(mutate);
        theLiving{specimen}(mutate,2) = theLiving{specimen}(mutate,2) + mutationSign(mutate,2) .* directionMutation(mutate);
        
        % Limit speed
        theLiving{specimen}(theLiving{specimen}(:,1) > maxSpeed, 1) = maxSpeed;
        theLiving{specimen}(theLiving{specimen}(:,1) < minSpeed, 1) = minSpeed;
    end
    
    % % % Breaking mechanisms % % %
    if (enableMaxGenerations == 1 && maxGenerations == generation)
        break
    end
    
    if (enableTimeout == 1 && toc >= timeout)
        break
    end
    
    generation = generation + 1;
end

if goalAchieved
    fprintf('Goal achieved!\n')
    fprintf('Achieved generation %i.\n',generation)
    toc
else
    fprintf('Goal not achieved.\n')
    fprintf('Achieved generation %i.\n',generation)
end
