function point_view_matrix = chaining(matches)
% CHAINING Constructs the point-view matrix with the matches found between
% consecutive frames. This matrix has tracked points as columns, and
% views/frames as rows and contains the indices of the descriptor for each
% frame. So, if a certain descriptor can be seen in all frames, its column
% is completely filled. Similarly, if it can be matched only between frame
% 1 and 2, only the first 2 rows of its column will be non-zero.
%
% matches: matrix containing matches, with descriptor indices for the 1st
% image in the 2nd row, indices for the 2nd image in the 3rd row. The 3rd
% dimension is stacked with each frame pair (1-2, 2-3, 3-4, ... , 16-1).

% Frames/views as rows, tracked points as columns
point_view_matrix = zeros(size(matches, 3), size(matches, 2));

% Different approach for 1st pair (1-2)
point_view_matrix(1:2, 1:size(matches, 2)) = matches(2:3, :, 1);

% Trim NaN/zero rows
point_view_matrix(:, ~any(point_view_matrix, 1)) = [];

for i = 2:size(matches, 3) - 1
    
    % Set intersection of indices in 1st frame of pair i and 2nd frame of
    % pair i - 1
    [~, ia, ib] = intersect(matches(2, :, i), point_view_matrix(i, :));
    
    % Write points of 2nd frame of pair i to row i + 1
    point_view_matrix(i + 1, ib) = matches(3, ia, i);
    
    % Add new matches
    new = matches(2:3, :, i);  % copy matches
    new(:, ia) = [];  % delete already matched rows, so only new left
    new(:, ~any(new, 1)) = [];  % delete NaNs
    add = zeros(size(point_view_matrix, 1), size(new, 2));
    add(i:i+1, :) = new;
    point_view_matrix = [point_view_matrix add]; 
    
% Different approach for last pair (16-1)
[~, ia, ib] = intersect(matches(2, :, 16), point_view_matrix(end, :));  % intersection between frame 16 of pair 16-1 and frame 16 of pair 15-16
[~, ia2, ib2] = intersect(matches(3, ia, 16), point_view_matrix(1, :));  % of those matches, select those that also have an intersection between frame 1 of pair 16-1 and frame 1 of pair 1-2, in order to add them to the correct column
point_view_matrix(15:16, ib2) = matches(2:3, ia2, 16);

end
