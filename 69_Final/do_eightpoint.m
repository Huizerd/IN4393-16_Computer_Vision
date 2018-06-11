function [F_ransac_denorm, inliers_1, inliers_2, inliers_idx] = do_eightpoint(sift, match_threshold, dist_threshold, n_matches, i)

if i < size(sift, 2)
    
    % Get coords and descriptors
    x1 = sift{1,i}(1,:);
    y1 = sift{1,i}(2,:);
    desc1 = sift{2,i}(:,:);

    x2 = sift{1,i+1}(1,:);
    y2 = sift{1,i+1}(2,:);
    desc2 = sift{2,i+1}(:,:);
    
else
    
    % Last pair: last & 1st image
    x1 = sift{1,i}(1,:);
    y1 = sift{1,i}(2,:);
    desc1 = sift{2,i}(:,:);

    x2 = sift{1,1}(1,:);
    y2 = sift{1,1}(2,:);
    desc2 = sift{2,1}(:,:);
    
end

% Get matches
[matches, ~] = vl_ubcmatch(desc1, desc2, match_threshold);

% % Get n best matches
% n = n_matches;
% best_matches = zeros(3, n);
% 
% % Get only useful matches in rectangle (kind of beun-oplossing, since
% % manual selection)
% m = 1;
% while m <= n
%     [best_matches(1,m), idx] = min(scores);
% 
%     scores(idx) = +Inf;
%

new_matches = [];
for m = 1:size(matches, 2)
    
    if (x1(:,matches(1,m)) > 885) && (x1(:,matches(1,m)) < 3286) && (y1(:,matches(1,m)) > 726) && (y1(:,matches(1,m)) < 1906)
        
        new_matches = [new_matches matches(:, m)];
        
    end
    
end
   
    
    
% 
% 
% if (x1(:,matches(1,idx)) > 885) && (x1(:,matches(1,idx)) < 3286) ...
%        && (y1(:,matches(1,idx)) > 726) && (y1(:,matches(1,idx)) < 1906)
% 
%         best_matches(2:end,m) = matches(:,idx);
%         m = m + 1;
%     end
% end


[F_ransac_denorm, inliers_1, inliers_2, inliers_idx] = eightpoint(x1, y1, x2, y2, new_matches, dist_threshold);