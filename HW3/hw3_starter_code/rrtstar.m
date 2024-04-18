function [fpath, cost] = rrt_star(envmap, start, goal, deltaStep)
% Implements the RRT* motion planner to find a collision-free path
% INPUTS and OUTPUTS are same as RRT

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

% Initialize tree
tree.nodes = start;
tree.edges = [];
tree.costs = 0;  % Cost from start to the node
goalReached = false;

ct = 0;
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

