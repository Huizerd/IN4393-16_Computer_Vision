function [F_ransac_denorm, inliers_1, inliers_2, inliers_match_idx] = do_eightpoint(sift, match_threshold, dist_threshold, i)
% DO_EIGHTPOINT Performs 8-point algorithm between two images. First
% extracts coordinates and descriptors from SIFT features, matches the
% descriptors, keeps only interesting matches and then performs 8-point
% algorithm on these matches.
% 
% Inputs:
% - sift: cell-array containing SIFT features for both images
% - match_threshold: threshold used in feature matching
% - dist_threshold: threshold used in 8-point algorithm Sampson-distance
%   calculation
% - i: current index in SIFT features cell-array
% 
% Outputs:
% - F_ransac_denorm: Denormalized fundamental matrix F with largest amount
%   of inliers found using RANSAC
% - inliers_1: coordinates of all inliers for first image
% - inliers_2: coordinates of all inliers for second image
% - inliers_match_idx: indices of inliers for use with initial match array
% 
% Jesse Hagenaars & Michiel Mollema - 11.06.2018

%% Get coordinates and descriptors
x1 = sift{1,i}(1,:);
y1 = sift{1,i}(2,:);
desc1 = sift{2,i}(:,:);

x2 = sift{1,i+1}(1,:);
y2 = sift{1,i+1}(2,:);
desc2 = sift{2,i+1}(:,:);

%% Get matches
[matches, scores] = vl_ubcmatch(desc1, desc2, match_threshold);

% Only keep matches in centre of image (which contains the castle)
new_matches = [];
for m = 1:size(matches, 2)
    
    if (x1(:,matches(1,m)) > 885) && (x1(:,matches(1,m)) < 3286) && (y1(:,matches(1,m)) > 726) && (y1(:,matches(1,m)) < 1906)
        
        new_matches = [new_matches matches(:, m)];
        
    end
    
end

%% Perform the 8-point algorithm
[F_ransac_denorm, inliers_1, inliers_2, inliers_match_idx] = eightpoint(x1, y1, x2, y2, new_matches, dist_threshold);