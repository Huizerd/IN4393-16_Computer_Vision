function points = get_points(sift, pv_matrix)
% GET_POINTS Obtains x & y coordinates from SIFT features for each set
%   of frames in the point view matrix.
%
% Inputs:
% - sift: cell array of SIFT features and descriptors for each frame
% - pv_matrix: point view matrix, shape (N_frames, N_sift)
%
% Outputs:
% - points: matrix of x & y coordinates, shape (2 * N_frames, N_points),
%   rows of x & y coordinates alternate (so 1st 2 rows belong to 1st frame)
%
% Jesse Hagenaars & Michiel Mollema - 11.06.2018

% Create (2 * N_frames, N_points) point matrix
points = zeros(2 * size(pv_matrix, 1), size(pv_matrix, 2));

% Counter for point matrix
k = 1;

% Go over the rows of the point view matrix
for f = 1:size(pv_matrix, 1)
    
    % Discards the SIFT feature with index 0
    coords = sift{1, f}(1:2, nonzeros(pv_matrix(f, :)));
    
    % Put x & y coordinates in point matrix
    points(k:k+1, find(pv_matrix(f, :))) = coords;
    
    % Up counter
    k = k + 2;
    
end

% Find features visible in all frames
not_all_frames = ~all(points, 1);

% Delete those features
points(:, not_all_frames) = [];

end

