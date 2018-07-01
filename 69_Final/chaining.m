function point_view_matrix = chaining(matches)
% CHAINING Constructs the point-view matrix with the matches found between
%   consecutive frames. This matrix has tracked points as columns, and
%   views/frames as rows and contains the indices of the descriptor for
%   each frame. So, if a certain descriptor can be seen in all frames, its
%   column is completely filled. Similarly, if it can be matched only
%   between frame 1 and 2, only the first 2 rows of its column will be
%   non-zero.
%
% Inputs:
% - matches: cell array containing matches, with descriptor indices for the
%   1st image in the 1st row, indices for the 2nd image in the 2nd row.
%   Each cell contains one frame pair (1-2, 2-3, 3-4, ... , 19-1).
%
% Outputs:
% - point_view_matrix: matrix containing matches between consecutive frames
%
% Jesse Hagenaars & Michiel Mollema - 04.06.2018

% Frames/views as rows, tracked points as columns
% Different approach for 1st pair (1-2)
point_view_matrix = matches{1, 1};

for i = 2:size(matches, 2)
    
    % Pair to match
    matching = matches{1, i};
    
    % Set intersection of indices in 1st frame of pair i & 2nd frame of
    % pair i - 1
    [~, ia, ib] = intersect(matching(1, :), point_view_matrix(i, :));
    
    % Write points of 2nd frame of pair i to row i + 1
    point_view_matrix(i+1, ib) = matching(2, ia);
    
    % Add new matches
    matching(:, ia) = [];  % delete already matched points
    point_view_matrix(i:i+1, end+1:end+size(matching, 2)) = matching;  % append to point-view matrix
    
end

% Move matches between 1st & last frame to their corresponding columns in
%   the 1st frame
[~, ia, ib] = intersect(point_view_matrix(1, :), point_view_matrix(end, :));
point_view_matrix(:, ia(2:end)) = point_view_matrix(:, ia(2:end)) + point_view_matrix(:, ib(2:end));  % skip 1st index (contains zeros)
point_view_matrix(:, ib(2:end)) = [];  % delete moved points in last frame

% Find matches between 1st & last frame without a column (yet)
nonzero_last = find(point_view_matrix(end, :));
nonzero_first = find(point_view_matrix(1, :));
no_member = ~ismember(nonzero_last, nonzero_first);
nonzero_last = nonzero_last(no_member);
tocopy = point_view_matrix(:, nonzero_last);

% Put these before 1st frame
point_view_matrix(:, nonzero_last) = [];
point_view_matrix = [tocopy point_view_matrix];

% Copy extra row elements to 1st row, delete last row
point_view_matrix(1 ,1:size(tocopy, 2)) = point_view_matrix(end, 1:size(tocopy, 2));
point_view_matrix = point_view_matrix(1:size(matches, 2),:); 
   
end
