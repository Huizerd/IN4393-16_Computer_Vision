function residuals = cam_residuals(L, M_hat)
% CAM_RESIDUALS Returns the residuals computed for every camera view point.
%
% Inputs:
% - L: matrix to be minimized
% - residuals: matrix containing the residuals for the N cameras,
%   shape (N, 4)
%
% Jesse Hagenaars & Michiel Mollema - 11.06.2018
% Written by IN4393-16 staff

% Pre-allocate the residual matrix
residuals = zeros(size(M_hat, 1) / 2, 4);

% Compute the residuals
for i = 1:size(M_hat, 1) / 2
    
    A_i = M_hat(i * 2 - 1:i * 2, :);
    D = A_i * L * A_i' - eye(2);
    residuals(i, :) = D(:);
    
end

end