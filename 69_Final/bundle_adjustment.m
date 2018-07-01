function E = bundle_adjustment(D, MS, m, n)
% BUNDLE_ADJUSTMENT Performs bundle adjustment, a.k.a. minimizes the
%   reprojection error.
%
% Inputs:
% - D: centered image coordinates
% - MS: vector containing both the camera motion matrix & 3D point matrix
% - m: number of cameras
% - n: number of points
%
% Outputs:
% - E: reprojection error to minimize
% 
% Jesse Hagenaars & Michiel Mollema - 01.07.2018

% Reprojection error
E = 0;

% Reshape back to M & S
S = reshape(MS(1:n*3), [3 n]);
M = reshape(MS(end-m*6+1:end), [2*m 3]);

% To make more readable
PX = M * S;

% Compute reprojection error
for i = 1:m
    for j = 1:n
        
        E = E + sqrt((D(i*2-1, j) - PX(i*2-1, j))^2 + (D(i*2, j) - PX(i*2, j))^2);
        
    end
end
end