function [F, F_denorm, Fbest] = eightpoint(A)

[~, ~, Va]    = svd(A); % Singular Value decomposition of A
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

% Create new A matrix from normalized coordinates
Anorm = zeros(length(p_hat), 9);
for i = 1:length(p_hat)
    Anorm(i,:) = [p_hat(1,i)*p_hat_acc(1,i),...
                p_hat(1,i)*p_hat_acc(2,i),...
                p_hat(1,i),...
                p_hat(2,i) * p_hat_acc(1,i),...
                p_hat(2,i) * p_hat_acc(2,i),...
                p_hat(2,i),...
                p_hat_acc(1,i),...
                p_hat_acc(2,i),...
                1];
end

% Determine normalized F
[~, ~, Va_norm] = svd(Anorm);
f_norm   = Va_norm(:,end);
F_norm   = reshape(f_norm, 3, 3);

% Force singularity of F
[Uf_norm, Df_norm, Vf_norm] = svd(F_norm);
Df_norm(end, end) = 0;
F_norm  = Uf_norm * Df_norm * Vf_norm.';

% Denormalization of F
F_denorm = T.' * F_norm * T;

%% RANSAC
inliersBest = 0;
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
    
    % Find inliers
    Fp = F * p_hat;
    Ftp= F'* p_hat;
    d = (p_hat_acc' * F_ransac * p_hat)^2 / ...
        (Fp(1)^2 + Fp(2)^2 + Ftp(1)^2 + Ftp(2)^2);
    inliers = length(find(d < threshold));
    
    if inliers > inliersBest   
        inliersBest = inliers;
        Fbest = F_ransac;  
    end
end

end