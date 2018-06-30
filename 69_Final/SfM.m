function [M, S, affine_amb_solved] = SfM(points_center)
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

% SVD
[U, W, V] = svd(points_center);

% Decompose into measurements M and shape S
% Only top 3 singular values
M_hat = U(:, 1:3) * sqrtm(W(1:3, 1:3));
S_hat = sqrtm(W(1:3, 1:3)) * V(:, 1:3)';

% Put affine ambiguity as conditional?

% Solve for affine ambiguity
% What is A? Then use it in L0   
% A = M_hat;
A = M_hat(1:2, :);
L0 = pinv(A' * A);

% Solve for L
% options = optimoptions(@lsqnonlin, 'Display', 'off');
options = optimoptions(@lsqnonlin, 'StepTolerance',1e-16,'OptimalityTolerance',1e-16,'FunctionTolerance',1e-16, 'MaxFunctionEvaluations', 10000, 'Display', 'iter');
% L = lsqnonlin(@(x)cam_residuals(x, M_hat), L0, ones(size(L0))*-1e-3, ones(size(L0))*1e-3, options);
L = lsqnonlin(@(x)cam_residuals(x, M_hat), L0, [], [], options);

% Check symmetry
symm = issymmetric(round(L, 3));

if ~symm
    disp('L is not symmetric!')
end

% Cholesky decomposition
[C, e] = chol(L, 'lower');

if e == 0
    
    % figure
    % pcshow(pointCloud(S_hat'))
    % figure
    % pcshow(pointCloud((pinv(C) * S_hat)'))
    
    % Update M and S with C
    M = M_hat * C;
    S = pinv(C) * S_hat;
    affine_amb_solved = 1;
    
else
    
    M = M_hat;
    S = S_hat;
    affine_amb_solved = 0;
    
end

end