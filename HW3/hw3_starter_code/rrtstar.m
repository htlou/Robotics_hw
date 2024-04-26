function [fpath, cost] = rrtstar(envmap, start, goal, deltaStep)
% Implements the RRT-Star motion planner for a point-robot to find a
% collision-free path from start to goal
% INPUTS:
% envmap  - Map of the environment, envmap(y,x) = 1 means the cell
%           (x,y) is an obstacle, envmap(y,x) = 0 means it is free
% start   - Robot start [x,y] = [col,row]
% goal    - Robot goal [x,y] = [col,row]
% deltaStep - (Optional) Approx. number of cells by which to extend the graph in
%             direction of the nearest-neigbour on the graph (Default: 10)
% OUTPUTS:
% fpath   - Final collision-free path (N x 2, each row is [x,y])
% cost    - Cost of the final path (sum of squared distances between nodes on the path)

if nargin < 3
    error('Need to pass in map, start and goal');
end
if nargin < 4
    deltaStep = 10;
end
fprintf('Deltastep = %f \n', deltaStep);

% Setup figure for display
fg = figure(101); hold on;
imagesc(envmap); colormap(gray);
t1 = text(start(1), start(2), 'S'); set(t1, 'Color', 'r', 'FontSize', 15);
t2 = text(goal(1), goal(2), 'G'); set(t2, 'Color', 'g', 'FontSize', 15);
title('RRT* Planning');
xlim([1, size(envmap,2)]);
ylim([1, size(envmap,1)]);

%% TODO:
% Implement RRT to find a collision free path. Note that the edges also
% have to be collision-free, not just the nodes.
%   a) Sample states at random 99% of time, 1% sample goal state
%   b) Extend the graph by "deltaStep" steps from the nearest
%   neighbour of the sample in the direction of the sample. If sample is
%   within deltaStep distance, use it directly
%   c) Check within a ball around "xnew" to find min cost state to "xnew".
%   Set that as the parent
%   d) Check if any other state in the ball around "xnew" has a better path
%   through "xnew". If so, rewire the graph.
%   c) Approximate collision checking for the (approximate) straight line path
%   between two states can be done using the function "collcheckstline"
%   d) Run till you find a state on the graph which is near the goal state
%   e) Display the progression of the graph generation for RRTStar (figure(fg))

% Initialize tree
tree.nodes = start;
tree.edges = [];
tree.costs = 0;  % Cost from start to the node
goalReached = false;

ct = 0;
% set an appropriate radius for rewiring
radius = 4;
while true
    if goalReached
        break;
    end

    % Sampling with goal bias
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

    if collcheckstline(nearest, xnew, envmap)
        % Find all nodes within the radius
        distances = sqrt(sum((tree.nodes - xnew).^2, 2));
        nearbyIndices = find(distances < radius);

        % Find minimum cost to xnew
        minCost = inf;
        minNode = idx;
        for j = nearbyIndices'
            if collcheckstline(tree.nodes(j, :), xnew, envmap)
                newCost = tree.costs(j) + norm(tree.nodes(j, :) - xnew);
                if newCost < minCost
                    minCost = newCost;
                    minNode = j;
                end
            end
        end

        % Add xnew to the tree
        newNodeIdx = size(tree.nodes, 1) + 1;
        tree.nodes = [tree.nodes; xnew];
        tree.edges = [tree.edges; minNode newNodeIdx];
        tree.costs = [tree.costs minCost];

        % Rewire the tree
        for j = nearbyIndices'
            if j == newNodeIdx, continue; end
            if collcheckstline(xnew, tree.nodes(j, :), envmap)
                potentialCost = minCost + norm(xnew - tree.nodes(j, :));
                if potentialCost < tree.costs(j)
                    tree.edges(tree.edges(:, 2) == j, 1) = newNodeIdx;
                    tree.costs(j) = potentialCost;
                end
            end
        end

        % Plot new edges and nodes
        parentNode = tree.edges(tree.edges(:, 2) == newNodeIdx, 1);
        plot([tree.nodes(parentNode, 1), xnew(1)], [tree.nodes(parentNode, 2), xnew(2)], 'w-', 'LineWidth', 2);
        drawnow;

        % Check if goal is near
        if norm(xnew - goal) < deltaStep
            goalReached = true;
        end
    end

    if ~mod(ct, 200)
        figure(fg);
        drawnow;
    end
    ct = ct + 1;
end

% Backtrack to find the path and cost
fpath = reconstruct_path(tree, size(tree.nodes, 1));
cost = tree.costs(find(all(tree.nodes == fpath(1, :), 2)));

% Plot final path
figure(fg);
plot(fpath(:,1), fpath(:,2), 'k.-', 'LineWidth', 2);
title('Final path by RRT*');
drawnow;

end

% Additional functions (reconstruct_path and collcheckstline) are assumed to be defined similarly as before

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

function [nearestNodeIndex, nearestNode] = nearestNode(tree, sample)
    % Computes the nearest node in the tree to the given sample point
    % Inputs:
    %   tree - Struct with at least one field called 'nodes' which is an Nx2 matrix of [x, y] coordinates
    %   sample - A 1x2 matrix of [x, y] coordinates representing the sample point
    % Outputs:
    %   nearestNodeIndex - Index of the nearest node in the tree's node list
    %   nearestNode - The coordinates of the nearest node as a 1x2 matrix

    % Calculate the squared Euclidean distances from the sample to each node in the tree
    distances = sum((tree.nodes - sample).^2, 2);
    
    % Find the index of the smallest distance
    [minDistance, nearestNodeIndex] = min(distances);
    
    % Extract the nearest node using the found index
    nearestNode = tree.nodes(nearestNodeIndex, :);
end

