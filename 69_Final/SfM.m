function S = SfM(points)
% SFM Performs structure-from-motion and solves for the affine ambiguity.
%
% Inputs:
% - points: matrix of x & y coordinates, shape (2 * N_frames, N_points),
%   rows of x & y coordinates alternate (so 1st 2 rows belong to 1st frame)
%
% Outputs:
% - S: matrix of 3D points
%
% Jesse Hagenaars & Michiel Mollema - 11.06.2018

% Sizes
[N_frames, N_points] = size(points);
N_frames = N_frames / 2;

% Center points
points_center = points - repmat(sum(points, 2) / N_points, 1, N_points);

% SVD
[U, W, V] = svd(points_center);

% Decompose into measurements M and shape S
% Only top 3 singular values
M_hat = U(:, 1:3) * sqrt(W(1:3, 1:3));
S_hat = sqrt(W(1:3, 1:3)) * V(:, 1:3)';

% Solve for affine ambiguity
% What is A? Then use it in L0   
A  = M_hat;
L0 = inv(A' * A);

% Save Mhat for myfun
save('M_hat', 'M_hat')

% Solve for L
L = lsqnonlin(@cam_residuals, L0);

% Cholesky decomposition
C = chol(L, 'lower');

% Update M and S with C
M = M_hat * C;
S = pinv(C) * S_hat;

end