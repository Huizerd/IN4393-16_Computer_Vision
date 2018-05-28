%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assignment 6: Matching -> 8-point algorithm
% Jesse Hagenaars & Michiel Mollema - 28-05-2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Fbest, inliersBest] = eightpoint(A)

[~, ~, Va]      = svd(A); % Singular Value decomposition of A
f               = Va(:,end); % Last column of V (corresponding to smallest singular value equals f
F               = reshape(f, 3, 3); % Reshape f to matrix F

% Enforce singularity of fundamental matrix F
[Uf, Df, Vf]    = svd(F);
Df(end, end)    = 0;
F               = Uf * Df * Vf.';

%% Normalization
% For  first image
xcoords = A(:,3);
ycoords = A(:,6);
p       = [xcoords, ycoords, ones(size(xcoords))]';
mx  = mean(xcoords);
my  = mean(ycoords);
d   = mean(sqrt((xcoords-mx).^2 + (ycoords-my).^2));
T   = [[sqrt(2)/d, 0, -mx * sqrt(2)/d];...
        [0, sqrt(2)/d, -my * sqrt(2)/d];...
        [0, 0, 1]];
p_hat = T * p;

% For second image
xcoords = A(:,7);
ycoords = A(:,8);
p_acc   = [xcoords, ycoords, ones(size(xcoords))]';
mx  = mean(xcoords);
my  = mean(ycoords);
d   = mean(sqrt((xcoords-mx).^2 + (ycoords-my).^2));
T   = [[sqrt(2)/d, 0, -mx * sqrt(2)/d];...
        [0, sqrt(2)/d, -my * sqrt(2)/d];...
        [0, 0, 1]];
p_hat_acc = T * p_acc;

%% RANSAC
n_inliersBest = 0;
threshold = 0.1;
N = 1000;
for n = 1:N
    % Get 8 random pairs
    perm = randperm(length(xcoords));
    P    = 8;
    seed = perm(1:P);
    p_hat_used = p_hat(:, seed);
    p_hat_acc_used = p_hat_acc(:,seed);
    
    % Create new A matrix from normalized coordinates
    Aransac = zeros(P, 9);
    for i = 1:P
        Aransac(i,:) = [p_hat_used(1,i)*p_hat_acc_used(1,i),...
                p_hat_used(1,i)*p_hat_acc_used(2,i),...
                p_hat_used(1,i),...
                p_hat_used(2,i) * p_hat_acc_used(1,i),...
                p_hat_used(2,i) * p_hat_acc_used(2,i),...
                p_hat_used(2,i),...
                p_hat_acc_used(1,i),...
                p_hat_acc_used(2,i),...
                1];
    end
    
    % Determine fundamental matrix F
    [V, ~]   = eigs(Aransac' * Aransac, 1, 'smallestabs');
    f_ransac = V;
    F_ransac = reshape(f_ransac, 3, 3);
    
    % Find inliers for all n points in A
    Fp = F * p_hat;
    Ftp= F'* p_hat;
    d   = zeros(1, length(Fp));
    
    for i = 1:length(Fp)
        d(i) = (p_hat_acc(:,i)' * F_ransac * p_hat(:,i))^2 / ...
            (Fp(1,i)^2 + Fp(2,i)^2 + Ftp(1,i)^2 + Ftp(2,i)^2);
    end

    inliers     = find(d < threshold);
    n_inliers   = length(inliers);
    
    if n_inliers > n_inliersBest   
        n_inliersBest   = n_inliers;
        matchesBest     = 1;
        inliersBest     = inliers;
        Fbest           = F_ransac;  
    end
end

end