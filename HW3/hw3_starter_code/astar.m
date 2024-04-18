function[fpath, cost, displaymap] = astar(envmap, start, goal, epsilon)
% Implements the A-star motion planner for a point-robot to find a
% collision-free path from start to goal
% INPUTS:
% envmap  - Map of the environment, envmap(y,x) = 1 means the cell
%           (x,y) is an obstacle, envmap(y,x) = 0 means it is free
% start   - Robot start [x,y] = [col,row]
% goal    - Robot goal [x,y] = [col,row]
% epsilon - (Optional) Epsilon for epsilon a-star >= 1.0 (Default = 1.0)
% OUTPUTS:
% fpath      - Final collision-free path (N x 2, each row is [x,y])
% cost       - Cost of the final path
% displaymap - Map highlighted with closed & open states from a-star

if nargin < 3
    error('Need to pass in map, start and goal');
end

if nargin < 4
    epsilon = 1.0;
end
if epsilon < 0
    error('Epsilon has to be >= 0.0');
end
fprintf('Using epsilon = %f \n', epsilon);

% Have a map for display
displaymap = envmap;
fg = figure(101); hold on;
imgg = imagesc(displaymap);
t1 = text(start(1), start(2), 'S'); set(t1,'Color','r','Fontsize',15);
t2 = text(goal(1), goal(2), 'G'); set(t2,'Color','g','Fontsize',15);
title('A-star planning');
xlim([1,size(displaymap,2)]);
ylim([1,size(displaymap,1)]);

%% TODO: 
% Implement A-star to find a collision free path. 
%  a) Use cost of g+h for a-star
%  where the heuristic (h) is Euclidean distance to the goal. Each transition
%  has a cost equal to the length of the transition:
%    cost of action (dx, dy) = sqrt(dx^2 + dy^2);
% (So actions that move left,right,up,down have cost = 1,
%  actions that move diagonally have cost = sqrt(2)). 
%  Use an 8-connected neighbourhood for A-star.
%  b) A state is an obstacle if:
%       envmap(state(2), state(1)) = 1
%     A state is free if:
%       envmap(state(2), state(1)) = 0
%  c) To check if a state is within limits, use the function "checkLimits"
%  d) Display the closed/open states for A-star as the search progresses.
%  You can easily do this by setting these states to have a specific value
%  in the variable "displaymap". For example, open states can have
%  value = 2 & closed states can have value = 3. Note that value of 0 means
%  free & 1 means obstacles.
%     displaymap(openState(2), openState(1)) = 2
%     displaymap(closedState(2), closedState(1)) = 3
%  e) Most implementations of A-star use a priority queue to sort the open
%  states before choosing the next one to expand. MATLAB does not have a 
%  native priority queue, but you can avoid this by saving the
%  costs/corresponding states in two arrays and retrieving the min at each
%  step:
%       [minval, minid] = min(f_cost) % f_cost is a vector of costs
% minval is minimum cost value and minid is the element with the min cost.
%   f) Once you have reached the goal state, you will need to backtrack to
%   find the path from start to goal.
% Note (x,y) = (col,row) in MATLAB

actions = [1, 0; 0, 1; -1, 0; 0, -1; 1, 1; -1, -1; 1, -1; -1, 1];
costs = [1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)];

openSet = [start];
cameFrom = zeros(size(envmap, 1), size(envmap, 2), 2);
gScore = inf(size(envmap));
gScore(start(2), start(1)) = 0;
fScore = inf(size(envmap));
fScore(start(2), start(1)) = heuristic(start, goal);

% ct = 0;
% while(1)
%     %%% TODO: Run till goal state is reached or there are no open states
%     % If state is open -> displaymap(state(2), state(1)) = 2
%     % If state is closed -> displaymap(state(2), state(1)) = 3
    
%     % Display intermittently - assumes that displaymap has been modified
%     % properly by the student

%     if ~mod(ct,200)
%         figure(fg);
%         set(imgg, 'CData', displaymap);
%         drawnow;
%     end
    
%     % Increment counter
%     ct = ct+1;
% end
ct = 0;
while ~isempty(openSet)
    % Find the node in openSet with the lowest fScore
    [~, idx] = min(arrayfun(@(x) fScore(openSet(x,2), openSet(x,1)), 1:size(openSet,1)));
    current = openSet(idx,:);
    
    % Check if goal is reached
    if isequal(current, goal)
        fpath = reconstruct_path(cameFrom, current);
        cost = gScore(goal(2), goal(1));
        break;
    end
    
    % Move current node from open to closed set
    openSet(idx,:) = [];
    displaymap(current(2), current(1)) = 3;  % Mark as closed
    
    % Explore neighbors
    for i = 1:size(actions,1)
        neighbor = current + actions(i,:);
        if within_bounds(neighbor, size(envmap)) && envmap(neighbor(2), neighbor(1)) == 0
            tentative_gScore = gScore(current(2), current(1)) + costs(i);
            if tentative_gScore < gScore(neighbor(2), neighbor(1))
                cameFrom(neighbor(2), neighbor(1), :) = current;
                gScore(neighbor(2), neighbor(1)) = tentative_gScore;
                fScore(neighbor(2), neighbor(1)) = tentative_gScore + epsilon * heuristic(neighbor, goal);
                if ~ismember(neighbor, openSet, 'rows')
                    openSet = [openSet; neighbor];
                    displaymap(neighbor(2), neighbor(1)) = 2;  % Mark as open
                end
            end
        end
    end

    % Update display every few iterations
    if ~mod(ct,200)
        figure(fg);
        set(imgg, 'CData', displaymap);
        drawnow;
    end

    ct = ct+1;
end

%%% TODO: Backtrack to find shortest path & cost of the path
% You need to set variables fpath & cost

%%%

% Final display before exit
figure(fg);
set(imgg, 'CData', displaymap);
plot(fpath(:,1), fpath(:,2), 'k.-', 'Linewidth',2); % Display final path
drawnow;
fprintf('Number of closed states: %d \n', sum(sum(displaymap == 3)));

end

function h = heuristic(node, goal)
    % Euclidean distance
    h = sqrt(sum((node - goal).^2));
end

function valid = within_bounds(node, map_size)
    % Check if the node is within the map boundaries
    valid = all(node >= 1) && node(1) <= map_size(2) && node(2) <= map_size(1);
end

function path = reconstruct_path(cameFrom, current)
    % Reconstruct the path from start to goal
    path = current;
    while any(cameFrom(current(2), current(1), :))
        current = squeeze(cameFrom(current(2), current(1), :))';
        path = [current; path];
    end
end