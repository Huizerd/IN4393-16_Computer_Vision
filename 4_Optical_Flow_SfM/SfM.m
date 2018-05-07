function SfM(points)

% Sizes
[framesN, pointsN] = size(points);
framesN = framesN / 2;

% Center points
pointsCenter = points - repmat(sum(points, 2) / pointsN, 1, pointsN);

% SVD
[U, W, V] = svd(pointsCenter);

% Decompose into measurements M and shape S
% Only top 3 singular values
Mhat = U(:, 1:3) * sqrt(W(1:3, 1:3));
Shat = sqrt(W(1:3, 1:3)) * V(:, 1:3)';

% Solve for affine ambiguity
% What is A? Then use it in L0   
A  = Mhat;
L0 = inv(A' * A);

% Save Mhat for myfun
save('Mhat', 'Mhat')

% Solve for L
L = lsqnonlin(@myfun, L0);

% Cholesky decomposition
C = chol(L, 'lower');

% Update M and S with C
M = Mhat * C;
S = pinv(C) * Shat;

plot3(S(1, :), S(2,:), S(3,:),'b.');