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
A  = 
L0 = 

% Save M for myfun
save('M', 'M')

% Solve for L
L = lsqonlin(@myfun, L0);

% Impose metric constrains
% Is = Mhat(1:framesN, :);
% Js = Mhat(framesN+1:end, :);
% 
% gfun = @(a, b)[ a(1)*b(1), a(1)*b(2)+a(2)*b(1), a(1)*b(3)+a(3)*b(1), ...
%               a(2)*b(2), a(2)*b(3)+a(3)*b(2), a(3)*b(3)];
% 
% G = zeros(3*framesN, 6);
% 
% for f = 1:3*framesN
%     if f <= framesN
%         G(f, :) = gfun(Is(f,:), Is(f,:));
%     elseif f <= 2*framesN
%         G(f, :) = gfun(Js(mod(f, framesN+1)+1, :), Js(mod(f, framesN+1)+1, :));
%     else
%         G(f, :) = gfun(Is(mod(f, 2*framesN),:), Js(mod(f, 2*framesN),:));
%     end
% end
% 
% c = [ones(2*framesN, 1); zeros(framesN, 1)];

% Solve by SVD and mldivide
% [U, S, V] = svd(G);
% 
% hatl = U'*c;
% 
% y = [hatl(1)/S(1,1); hatl(2)/S(2,2); hatl(3)/S(3,3); hatl(4)/S(4,4); ...
%     hatl(5)/S(5,5); hatl(6)/S(6,6)];
% 
% l = V*y;
% 
% L = [l(1) l(2) l(3);...
%      l(2) l(4) l(5);...
%      l(3) l(5) l(6)] ;

% Cholesky decomposition
C = chol(L, 'lower');

% Update M and S with C
M = Mhat * C;
S = pinv(C) * Shat;

plot3(S(1, :), S(2,:), S(3,:),'b.');