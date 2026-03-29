clc; clear; close all;
%% Case Configuration
% Case 1
cases(1).start = [1, 1];
cases(1).goal  = [10, 10];
cases(1).obs   = [1,5; 2,5; 3,5; 3,9; 4,2; 4,5; 4,7; 4,8; 5,2; 5,3; 5,7; 5,9; 6,3; 6,4; 6,5; 6,8; 6,9; 7,5; 7,9; 8,6; 8,7; 8,8; 9,5; 9,9; 10,9];
% Case 2
cases(2).start = [1, 10]; 
cases(2).goal  = [10, 10];
cases(2).obs   = [1,5; 2,5; 3,5; 4,5; 5,5; 6,5; 7,5; 8,5; 8,6; 9,5; 9,6; 10,5];
% Case 3
cases(3).start = [1, 1];
cases(3).goal  = [10, 10];
cases(3).obs   = [1,4; 1,5; 1,7; 1,8; 2,3; 2,4; 2,8; 3,6; 3,8; 4,3; 4,6; 4,8; 5,3; 5,4; 5,6; 5,9; 6,3; 6,4; 6,7; 7,3; 7,4; 7,7; 7,9; 8,4; 8,5; 8,9; 9,2; 9,6; 9,7; 10,4; 10,9];
% Case 4
cases(4).start = [1, 1]; 
cases(4).goal  = [10, 10];
cases(4).obs   = [2,1; 2,2; 2,3; 2,4; 2,5; 2,6; 2,7; 2,8; 4,3; 4,4; 4,5; 4,6; 4,7; 4,8; 4,9; 4,10; 7,9; 8,4; 8,5; 8,9; 9,2; 9,6; 9,7; 10,4; 10,9; 10,1;10,2;10,3];
% Case 5
cases(5).start = [1, 1]; 
cases(5).goal  = [10, 10];
cases(5).obs   = [1,5; 2,5; 2,6; 2,7; 2,8; 3,3; 3,4; 3,5; 3,9; 4,2; 4,3; 4,4; 4,5; 4,6; 4,7; 4,8; 4,9; 4,10; 5,5; 6,6; 7,7; 7,9; 8,4; 8,8; 10,4; 10,9];

max_x = 10; max_y = 10;

% Initialize total run time
total_run_time = 0;

% Print Table Header 
fprintf('  Case  |  Status   |  Computation Time (s) \n');

%% Main Automation Loop
for i = 1:length(cases)
    tic; % Start timer
    
    % Setup Map
    map = 2 * ones(max_x, max_y);
    start_node = cases(i).start;
    goal_node  = cases(i).goal;
    obs_list   = cases(i).obs;
    
    for j = 1:size(obs_list, 1)
        map(obs_list(j,1), obs_list(j,2)) = -1;
    end
    
    %% Initialize Lists
    open_list = []; 
    closed_list = []; 
    
    [obs_x, obs_y] = find(map == -1);
    if ~isempty(obs_x)
        closed_list = [obs_x, obs_y, zeros(length(obs_x), 5)];
    end
    
    g_start = 0; 
    h_start = calculateDistance(start_node, goal_node); 
    f_start = g_start + h_start; 
    open_list = [start_node, f_start, g_start, h_start, start_node]; 
    
    %% A* Search Loop
    path = []; 
    while ~isempty(open_list)
        [~, minIdx] = min(open_list(:, 3)); 
        current_node = open_list(minIdx, :); 
        curr_pos = current_node(1:2); 
        
        open_list(minIdx, :) = [];
        closed_list = [closed_list; current_node];
        
        if isequal(curr_pos, goal_node)
            path = reconstructPath(closed_list, start_node, goal_node); 
            break; 
        end 
        
        neighbors = getNeighbors(curr_pos, max_x, max_y, map); 
        for k = 1:size(neighbors, 1)
            neighbor = neighbors(k, :);
            if isNodeInList(neighbor, closed_list), continue; end
            
            g_neighbor = current_node(4) + calculateDistance(curr_pos, neighbor);
            h_neighbor = calculateDistance(neighbor, goal_node);
            f_neighbor = g_neighbor + h_neighbor;
            
            [in_open, open_idx] = isNodeInList(neighbor, open_list);
            if ~in_open || g_neighbor < open_list(open_idx, 4)
                neighbor_data = [neighbor, f_neighbor, g_neighbor, h_neighbor, curr_pos];
                if in_open
                    open_list(open_idx, :) = neighbor_data;
                else
                    open_list = [open_list; neighbor_data];
                end
            end 
        end
    end
    
    time_taken = toc; % Captured once path is found
    total_run_time = total_run_time + time_taken; % Add to total
    
    % Print result to Command Window
    if ~isempty(path)
        status = 'Success';
    else
        status = 'Failed ';
    end
    fprintf('   %02d   |  %s  |      %.6f\n', i, status, time_taken);
    
    %% Visualization
    figure(i); hold on; grid on;
    title(['Case ', num2str(i), ': Start [', num2str(start_node), '] Time: ', num2str(time_taken, '%.4f'), 's']);
    axis([1 max_x+1 1 max_y+1]);
    set(gca, 'XTick', 1:11, 'YTick', 1:11);
    
    plot(obs_x + 0.5, obs_y + 0.5, 'ro', 'MarkerFaceColor', 'r');
    plot(start_node(1) + 0.5, start_node(2) + 0.5, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(goal_node(1) + 0.5, goal_node(2) + 0.5, 'gd', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    
    if ~isempty(path)
        plot(path(:,1) + 0.5, path(:,2) + 0.5, 'k-', 'LineWidth', 2);
    end
end

% Print the Final Total Time
fprintf('------------------------------------------\n');
fprintf('  TOTAL RUN TIME FOR ALL CASES: %.6f s\n', total_run_time);

%% Helper Functions 
function d = calculateDistance(n1, n2), d = sqrt(sum((n1-n2).^2)); end
function neighbors = getNeighbors(node, rows, cols, grid) 
    r = node(1); c = node(2); 
    moves = [-1,0; 1,0; 0,-1; 0,1; -1,-1; -1,1; 1,-1; 1,1]; 
    neighbors = []; 
    for i = 1:size(moves, 1) 
        nr = r + moves(i,1); nc = c + moves(i,2); 
        if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols && grid(nr, nc) ~= -1 
            if abs(moves(i,1)) + abs(moves(i,2)) == 2 
                if grid(r + moves(i,1), c) == -1 || grid(r, c + moves(i,2)) == -1 
                    continue; 
                end 
            end 
            neighbors = [neighbors; nr, nc]; 
        end 
    end 
end
function [inList, idx] = isNodeInList(node, list) 
    inList = false; idx = 0; 
    if ~isempty(list) 
        match = find(list(:,1) == node(1) & list(:,2) == node(2), 1); 
        if ~isempty(match) 
            inList = true; idx = match; 
        end 
    end 
end
function path = reconstructPath(closedList, startNode, goalNode) 
    path = [goalNode]; curr = goalNode; 
    while ~isequal(curr, startNode) 
        idx = find(closedList(:,1) == curr(1) & closedList(:,2) == curr(2), 1); 
        parent = closedList(idx, 6:7); 
        path = [parent; path]; curr = parent; 
    end 
end