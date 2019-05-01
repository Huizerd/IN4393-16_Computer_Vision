function [F_ransac_denorm, inliers_1, inliers_2, inliers_match_idx, new_matches] = do_eightpoint(sift, match_threshold, dist_threshold, iter, i)
% DO_EIGHTPOINT Performs 8-point algorithm between two images. First
%   extracts coordinates and descriptors from SIFT features, matches the
%   descriptors, keeps only interesting matches and then performs 8-point
%   algorithm on these matches.
% 
% Inputs:
% - sift: cell-array containing SIFT features for both images
% - match_threshold: threshold used in feature matching
% - dist_threshold: threshold used in 8-point algorithm Sampson-distance
%   calculation
% - iter: number of iterations for RANSAC
% - i: current index in SIFT features cell-array
% 
% Outputs:
% - F_ransac_denorm: Denormalized fundamental matrix F with largest amount
%   of inliers found using RANSAC
% - inliers_1: coordinates of all inliers for first image
% - inliers_2: coordinates of all inliers for second image
% - inliers_match_idx: indices of inliers for use with initial match array
% 
% . & Michiel Mollema - 11.06.2018

%% Get coordinates and descriptors

if i < size(sift, 2)
    
    % Get coords and descriptors
    x1 = sift{1, i}(1, :);
    y1 = sift{1, i}(2, :);
    desc1 = sift{2, i}(:, :);

    x2 = sift{1, i+1}(1, :);
    y2 = sift{1, i+1}(2, :);
    desc2 = sift{2, i+1}(:, :);
    
else
    
    % Last pair: last & 1st image
    x1 = sift{1, i}(1, :);
    y1 = sift{1, i}(2, :);
    desc1 = sift{2, i}(:, :);

    x2 = sift{1, 1}(1, :);
    y2 = sift{1, 1}(2, :);
    desc2 = sift{2, 1}(:, :);
    
end


%% Get matches

disp('matching...')

[matches, ~] = vl_ubcmatch(desc1, desc2, match_threshold);

% Bounding boxes for images
% x, y, w, h
bbox = [1120 740 1836 1040; ...
        1240 692 1820 1152; ...
        1340 656 1732 1216; ...
        1276 644 1696 1220; ...
        1172 636 1632 1324; ...
        1184 612 1504 1424; ...
        1212 628 1316 1476; ...
        1228 664 1420 1416; ...
        1272 732 1520 1260; ...
        1176 780 1636 1128; ...
        1036 784 1912 1028; ...
        1016 800 2036 1012; ...
         972 772 1956 1164; ...
        1084 736 1576 1248; ...
        1328 712 1408 1332; ...
        1352 644 1432 1344; ...
        1252 656 1520 1188; ...
        1124 740 1652 1100; ...
        1140 788 1836 1056];

% Only keep matches in bounding box (which contains the castle)
new_matches = [];

for m = 1:size(matches, 2)
    
    if i < size(sift, 2)
        if (x1(:, matches(1, m)) > bbox(i, 1)) && ...
           (x1(:, matches(1, m)) < bbox(i, 1) + bbox(i, 3)) && ...
           (y1(:, matches(1, m)) > bbox(i, 2)) && ...
           (y1(:, matches(1, m)) < bbox(i, 2) + bbox(i, 4)) && ...
           (x2(:, matches(2, m)) > bbox(i+1, 1)) && ...
           (x2(:, matches(2, m)) < bbox(i+1, 1) + bbox(i+1, 3)) && ...
           (y2(:, matches(2, m)) > bbox(i+1, 2)) && ...
           (y2(:, matches(2, m)) < bbox(i+1, 2) + bbox(i+1, 4))

            new_matches = [new_matches matches(:, m)];

        end    
    else
        if (x1(:, matches(1, m)) > bbox(i, 1)) && ...
           (x1(:, matches(1, m)) < bbox(i, 1) + bbox(i, 3)) && ...
           (y1(:, matches(1, m)) > bbox(i, 2)) && ...
           (y1(:, matches(1, m)) < bbox(i, 2) + bbox(i, 4)) && ...
           (x2(:, matches(2, m)) > bbox(1, 1)) && ...
           (x2(:, matches(2, m)) < bbox(1, 1) + bbox(1, 3)) && ...
           (y2(:, matches(2, m)) > bbox(1, 2)) && ...
           (y2(:, matches(2, m)) < bbox(1, 2) + bbox(1, 4))

            new_matches = [new_matches matches(:, m)];

        end
    end
end


%% Perform the 8-point algorithm

disp('RANSAC...')

[F_ransac_denorm, inliers_1, inliers_2, inliers_match_idx] = eightpoint(x1, y1, x2, y2, new_matches, dist_threshold, iter);


end
