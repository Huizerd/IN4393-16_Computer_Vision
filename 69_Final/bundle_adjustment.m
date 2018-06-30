function E = bundle_adjustment(D, MS, m, n)

% Error to minimize
E = 0;

% Reshape back to M & S
S = reshape(MS(1:n*3), [3 n]);
M = reshape(MS(end-m*6+1:end), [2*m 3]);

% To make more readable
PX = M * S;

% E = D - PX;

for i = 1:m
    for j = 1:n
        
        E = E + sqrt((D(i*2-1, j) - PX(i*2-1, j))^2 + (D(i*2, j) - PX(i*2, j))^2);
        
    end
end

end