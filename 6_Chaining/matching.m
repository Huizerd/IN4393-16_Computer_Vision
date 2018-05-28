%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assignment 6: Matching -> Matching
% Jesse Hagenaards & Michiel Mollema - 28-05-2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [best_matches, A] = matching(desc1, desc2, n_matches)

% Image 1
output1 = dlmread(desc1, ' ', 2, 0);

x1 = output1(:, 1)';
y1 = output1(:, 2)';
% a1 = output1(:, 3)';
% b1 = output1(:, 4)';
% c1 = output1(:, 5)';
desc1 = output1(:, 6:end)';

% Image 2
output2 = dlmread(desc2, ' ', 2, 0);

x2 = output2(:, 1)';
y2 = output2(:, 2)';
% a2 = output2(:, 3)';
% b2 = output2(:, 4)';
% c2 = output2(:, 5)';
desc2 = output2(:, 6:end)';

% Matching
[matches, scores] = vl_ubcmatch(desc1, desc2, 2.);

% Get n best matches
n = n_matches;
best_matches = zeros(3, n);

m = 1;
k = 1;

while m <= n
    [best_matches(1,m), idx] = min(scores);

    scores(idx) = +Inf;

    % Nog ingebouwd dat ie alleen naar een bepaalde rectangle kijkt...
    if (x1(:,matches(1,idx)) > 750) && (x1(:,matches(1,idx)) < 1500) ...
       && (y1(:,matches(1,idx)) > 550) && (y1(:,matches(1,idx)) < 1100)

        best_matches(2:end,m) = matches(:,idx);
        m = m + 1;
    end
    
    k = k + 1;
    
    % Break to prevent infinite loop
    if k == length(scores)
        break
    end
    
end

% Above results in zeros if n_matches above nr of good matches!
% Gives problems below

% Create matrix A
A = zeros(n_matches, 9);

for row = 1:size(A, 1)
    A(row,:) = [x1(:,best_matches(2,row))*x2(:,best_matches(3,row))
                x1(:,best_matches(2,row))*y2(:,best_matches(3,row))
                x1(:,best_matches(2,row))
                y1(:,best_matches(2,row))*x2(:,best_matches(3,row))
                y1(:,best_matches(2,row))*y2(:,best_matches(3,row))
                y1(:,best_matches(2,row))
                x2(:,best_matches(3,row))
                y2(:,best_matches(3,row))
                1];
end

end