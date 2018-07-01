function [F_best, inliers_1, inliers_2, inliers_match_idx] = eightpoint(x1, y1, x2, y2, matches, threshold, iter)
% EIGHTPOINT Determines the fundamental matrix F with the highest number of
%   inliers using the 8-point algorithm and RANSAC. Also returns the
%   indices and coordinates of the inliers for both images.
% 
% Inputs:
% - x1: vector containing all x-coordinates of SIFT features in first image
% - y1: vector containing all y-coordinates of SIFT features in first image
% - x2: vector containing all x-coordinates of SIFT features in second
%   image
% - y2: vector containing all y-coordinates of SIFT features in second
%   image
% - matches: array containing indices of matches SIFT features between both
%   images
% - threshold: threshold value for Sampson-distance (used to determine
%   inliers)
% - iter: number of iterations for RANSAC
% 
% Outputs:
% - F_best: Fundamental matrix with highest number of inliers
% - inliers_1: coordinates of inliers for first image
% - inliers_2: coordinates of inliers for second image
% - inliers_match_idx: indices of inliers for use with initial match array
% 
% Jesse Hagenaars & Michiel Mollema - 11.06.2018

%% Normalization

% For  first image
xcoords = x1(:, matches(1, :))';
ycoords = y1(:, matches(1, :))';
mx = mean(xcoords);
my = mean(ycoords);

p = [xcoords, ycoords, ones(size(xcoords))]';
d = mean(sqrt((xcoords - mx).^2 + (ycoords - my).^2));

T1 = [sqrt(2) / d  0  -mx * sqrt(2) / d; ...
      0  sqrt(2) / d  -my * sqrt(2) / d; ...
      0     0     1];
  
p_hat = T1 * p;

% For second image
xcoords = x2(:, matches(2, :))';
ycoords = y2(:, matches(2, :))';
mx = mean(xcoords);
my = mean(ycoords);

p_acc = [xcoords, ycoords, ones(size(xcoords))]';
d  = mean(sqrt((xcoords - mx).^2 + (ycoords - my).^2));

T2 = [sqrt(2) / d  0  -mx * sqrt(2) / d; ...
      0  sqrt(2) / d  -my * sqrt(2) / d; ...
      0     0     1];
  
p_hat_acc = T2 * p_acc;


%% RANSAC

n_inliers_best = 0;
N = iter;

for n = 1:N
    
    % Get 8 random pairs
    perm = randperm(length(xcoords));
    P    = 8;
    seed = perm(1:P);
    p_hat_used = p_hat (:, seed);
    p_hat_acc_used = p_hat_acc(:,seed);
    
    % Create new A matrix from normalized coordinates
    Aransac = zeros(P, 9);
    
    for i = 1:P
        
        Aransac(i, :) = [p_hat_used(1, i) * p_hat_acc_used(1, i) ...
                         p_hat_used(1, i) * p_hat_acc_used(2, i) ...
                         p_hat_used(1, i) ...
                         p_hat_used(2, i) * p_hat_acc_used(1, i) ...
                         p_hat_used(2, i) * p_hat_acc_used(2, i) ...
                         p_hat_used(2, i) ...
                         p_hat_acc_used(1, i) ...
                         p_hat_acc_used(2, i) ...
                         1];
            
    end
    
    % Determine fundamental matrix F
    [~, ~, Vransac] = svd(Aransac);  % Singular Value decomposition of A
    f_ransac = Vransac(:, end);  % Last column of V (corresponding to smallest singular value equals f
    F_ransac = reshape(f_ransac, 3, 3);  % Reshape f to matrix F
    
    % Force singularity in F
    [Uf, Df, Vf] = svd(F_ransac);
    Df(end, end) = 0;
    F_ransac = Uf * Df * Vf';
    
    % Denormalize fundamental matrix
    F_ransac_denorm = T2' * F_ransac * T1;
    
    % Find inliers for all points p
    Fp = F_ransac_denorm * p;
    Ftp_acc = F_ransac_denorm' * p_acc;
    d  = zeros(1, length(p));
    
    for i = 1:length(p)
        
        d(i) = (p_acc(:, i)' * F_ransac_denorm * p(:, i))^2 / ...
               (Fp(1, i)^2 + Fp(2, i)^2 + Ftp_acc(1, i)^2 + Ftp_acc(2, i)^2);
        
    end

    inliers     = find(d < threshold);
    n_inliers   = length(inliers);
    
    % Store best results
    if n_inliers > n_inliers_best  
        
        n_inliers_best = n_inliers;
        inliers_best = inliers;
        F_best = F_ransac_denorm;
        
    end
    
end

% Convert inliers indices to coordinates for first and second image
inliers_match_idx = matches(:, inliers_best);
inliers_1 = [x1(inliers_match_idx(1, :)); y1(inliers_match_idx(1, :))]';
inliers_2 = [x2(inliers_match_idx(2, :)); y2(inliers_match_idx(2, :))]';

end
