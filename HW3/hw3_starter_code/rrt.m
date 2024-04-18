function[fpath, cost] = rrt(envmap, start, goal, deltaStep)
% Implements the RRT motion planner for a point-robot to find a
% collision-free path from start to goal
% INPUTS:
% envmap    - Map of the environment, envmap(y,x) = 1 means the cell
%           (x,y) is an obstacle, envmap(y,x) = 0 means it is free
% start     - Robot start [x,y] = [col,row]
% goal      - Robot goal [x,y] = [col,row]
% deltaStep - (Optional) Approx. number of cells by which to extend the graph in
%             direction of the nearest-neigbour on the graph (Default: 10)
% OUTPUTS:
% fpath     - Final collision-free path (N x 2, each row is [x,y])
% cost      - Cost of the final path (sum of squared distances between nodes on the path)

if nargin < 3
    error('Need to pass in map, start and goal');
end

if nargin < 4
    deltaStep = 10;
end
fprintf('Deltastep = %f \n',deltaStep);

% Figure
fg = figure(101); hold on;
imagesc(envmap); 
t1 = text(start(1), start(2), 'S'); set(t1,'Color','r','Fontsize',15);
t2 = text(goal(1), goal(2), 'G'); set(t2,'Color','g','Fontsize',15);
title('RRT planning');
xlim([1,size(envmap,2)]);
ylim([1,size(envmap,1)]);

%% TODO:
% Implement RRT to find a collision free path. Note that the edges also
% have to be collision-free, not just the nodes.
%   a) Sample states at random 99% of time, with 1% probability sample the goal state
%   b) Extend the graph by "deltaStep" steps from the nearest
%   neighbour of the sample in the direction of the sample (along the
%   straight line). If sample is within deltaStep distance, use it directly
%   c) Approximate collision checking for the (approx.) straight line path
%   between two states can be done using the function "collcheckstline".
%   This also returns the states along the straight line path.
%   d) A state is an obstacle if:
%       envmap(state(2), state(1)) = 1
%      A state is free if:
%       envmap(state(2), state(1)) = 0
%   e) Run till you find a state on the graph which is near the goal state
%   f) Display the progression of the graph generation for RRT (figure fg).
%   You can use the "plot" command in MATLAB to plot the edges of the graph
%   as you build it.
%   g) Use checkLimits.m to check for limits for the point robot. Round-off
% states for collision checking.
%   h) Once you have reached the goal state, you will need to backtrack to
%   find the path from start to goal.
% 
% Note (x,y) = (col,row) in MATLAB sense
tree.nodes = start;
tree.edges = [];
tree.costs = 0;
goalReached = false;

ct = 0;
while(1)
    if goalReached
        break;
    end
    %%% TODO: Run till goal is reached
    % Sample random state (sample goal with 1% prob)

    % Get Nearest Neighbour on graph (out of collision, within limits)

    % Extend graph towards sample by deltaStep to get xnew (check for path collision)

    % Add xnew to graph, do some display (intermittently)

    % Check for reaching goal & repeat
    
    %%%
    % Sample random point with a bias towards the goal
    if rand < 0.01
        sample = goal;
    else
        sample = [randi(size(envmap,2)), randi(size(envmap,1))];
    end

    % Find nearest node in the tree
    [minDist, idx] = min(sqrt(sum((tree.nodes - sample).^2, 2)));
    nearest = tree.nodes(idx, :);

    % Extend towards the sample
    direction = (sample - nearest) / norm(sample - nearest);
    if minDist > deltaStep
        xnew = nearest + direction * deltaStep;
    else
        xnew = sample;
    end

    % Check if the new node is in a valid position
    if collcheckstline(nearest, xnew, envmap)
        % Minimize the cost
        minCost = tree.costs(idx) + norm(xnew - nearest);
        newNodeIdx = size(tree.nodes, 1) + 1;
        tree.nodes = [tree.nodes; xnew];
        tree.edges = [tree.edges; idx newNodeIdx];
        tree.costs = [tree.costs minCost];
        
        % Try to reconnect through xnew
        for j = 1:size(tree.nodes, 1)-1
            if j == newNodeIdx
                continue;
            end
            if norm(tree.nodes(j, :) - xnew) < deltaStep && collcheckstline(tree.nodes(j, :), xnew, envmap)
                potentialCost = tree.costs(j) + norm(tree.nodes(j, :) - xnew);
                if potentialCost < minCost
                    tree.edges(tree.edges(:,2) == newNodeIdx, 1) = j;
                    tree.costs(newNodeIdx) = potentialCost;
                    minCost = potentialCost;
                end
            end
        end
        
        % Plot new edge
        parentNode = tree.edges(tree.edges(:,2) == newNodeIdx, 1);
        plot([tree.nodes(parentNode, 1) xnew(1)], [tree.nodes(parentNode, 2) xnew(2)], 'w-', 'LineWidth', 2);
        drawnow;

        % Check if goal is near
        if norm(xnew - goal) < deltaStep
            goalReached = true;
        end
    end
    % Display intermittently
    if ~mod(ct,200)
        figure(fg);
        drawnow;
    end
    
    % Increment counter
    ct = ct+1; 
    
end

%%% TODO: Backtrack to find the path between start & goal as well as the cost of the path
% You need to set variables fpath & cost

%%%
fpath = reconstruct_path(tree, size(tree.nodes, 1));
cost = tree.costs(find(all(tree.nodes == fpath(1, :), 2)));

% Draw a final time before exiting
figure(fg);
plot(fpath(:,1), fpath(:,2), 'k.-', 'Linewidth',2);
drawnow;

end

function path = reconstruct_path(tree, nodeIdx)
    % This function reconstructs the path from start to the node with index nodeIdx in the tree
    path = tree.nodes(nodeIdx, :);
    while nodeIdx ~= 1
        nodeIdx = tree.edges(tree.edges(:,2) == nodeIdx, 1); % Find the parent node
        path = [tree.nodes(nodeIdx, :); path]; % Prepend the parent node to the path
    end
end

function free = collcheckstline(p1, p2, map)
    % Bresenham's Line Algorithm for collision checking
    [cx, cy] = bresenham(p1(1), p1(2), p2(1), p2(2));
    free = all(map(sub2ind(size(map), cy, cx)) == 0);
end