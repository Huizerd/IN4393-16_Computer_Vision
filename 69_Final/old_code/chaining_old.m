function point_view_matrix = chaining_old(matches_cell)
% CHAINING Constructs the point-view matrix with the matches found between
%   consecutive frames. This matrix has tracked points as columns, and
%   views/frames as rows and contains the indices of the descriptor for
%   each frame. So, if a certain descriptor can be seen in all frames, its
%   column is completely filled. Similarly, if it can be matched only
%   between frame 1 and 2, only the first 2 rows of its column will be
%   non-zero.
%
% Inputs:
% - matches_cell: cell array containing matches, with descriptor indices
%   for the 1st image in the 1st row, indices for the 2nd image in the 2nd
%   row. Each cell contains one frame pair (1-2, 2-3, 3-4, ... , 16-1).
%
% Outputs:
% - point_view_matrix: matrix containing matches between consecutive frames
%
% Jesse Hagenaars - 04.06.2018

% Convert cell array to matrix with NaNs
N_matches = cellfun('size', matches_cell, 2);
matches = NaN(2, max(N_matches), size(matches_cell, 2));

for i = 1:size(matches_cell, 2)
    
    matches(:, 1:N_matches(i), i) = matches_cell{:, i};
    
end

% Frames/views as rows, tracked points as columns
point_view_matrix = zeros(size(matches, 3), size(matches, 2));

% Different approach for 1st pair (1-2)
point_view_matrix(1:2, 1:size(matches, 2)) = matches(:, :, 1);

% Trim NaN/zero rows
point_view_matrix(:, ~any(point_view_matrix, 1)) = [];

for i = 2:size(matches, 3) - 1
    
    % Set intersection of indices in 1st frame of pair i and 2nd frame of
    % pair i - 1
    [~, ia, ib] = intersect(matches(1, :, i), point_view_matrix(i, :));
    
    % Write points of 2nd frame of pair i to row i + 1
    point_view_matrix(i + 1, ib) = matches(2, ia, i);
    
    % Add new matches
    new = matches(:, :, i);  % copy matches
    new(:, ia) = [];  % delete already matched rows, so only new left
    new(:, ~any(new, 1)) = [];  % delete NaNs
    add = zeros(size(point_view_matrix, 1), size(new, 2));
    add(i:i+1, :) = new;
    point_view_matrix = [point_view_matrix add]; 
    
end

% Different approach for last pair
[~, ia, ib] = intersect(matches(1, :, end), point_view_matrix(end, :), 'stable');  % intersection between frame end of pair end-1 and frame end of pair end-1-end
[~, ia2, ib2] = intersect(matches(2, ia, end), point_view_matrix(1, :), 'stable');  % of those matches, select those that also have an intersection between frame 1 of pair end-1 and frame 1 of pair 1-2, in order to add them to the correct column

% Copy sequences to first image
point_view_matrix(end-6:end, ib2) = [point_view_matrix(end-6:end-1, ib(ia2)); matches(1, ia2, end)];

% Delete columns they came from
point_view_matrix(:, ib(ia2)) = [];
   
end
