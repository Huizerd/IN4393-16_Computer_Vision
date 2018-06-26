function E = bundle_adjustment(D, MS, m, n)

% Error to minimize
E = 0;

% Reshape back to M & S
M = reshape(MS(1:m*6), [2*m 3]);
S = reshape(MS(end-3*n+1:end), [3 n]);

% To make more readable
PX = M * S;

% E = D - PX;

for i = 1:m
    for j = 1:n
        
        E = E + sqrt((D(i*2-1, j) - PX(i*2-1, j))^2 + (D(i*2, j) - PX(i*2, j))^2);
        
    end
end

end