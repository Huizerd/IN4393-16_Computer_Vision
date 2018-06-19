% IN4393-16: Final Assignment
% 
% Jesse Hagenaars & Michiel Mollema - 04.06.2018

clc; clear; close all

run('C:/Users/jesse/Documents/MATLAB/vlfeat/toolbox/vl_setup')
% run('/home/michiel/Programs/MATLAB/vlfeat/toolbox/vl_setup')

%% Settings

% Higher = more strict
match_threshold = 1.25;
dist_threshold = 10;

% For Harris
sigma = 1.2.^(-9:12);
threshold_R = 5e-5;


%% Load images

disp('Loading images...')

% Images
image_files = dir('model_castle_png/*.png');

for i = 1:length(image_files)
    
    current_image = imread([image_files(i).folder '/' image_files(i).name]);
    
    % Initialize arrays based on image size
    if i == 1
        images = uint8(zeros([size(current_image) length(image_files)]));
        images_gray = uint8(zeros([size(current_image, 1) size(current_image, 2) length(image_files)]));
    end
    
    images(:, :, :, i) = uint8(current_image);
    images_gray(:, :, i) = rgb2gray(current_image);
    
   
end

 
%% Feature point detection & extraction of SIFT descriptors (4 pts)

% Only do if file doesn't exist already
if ~exist('features/sift.mat', 'file')
    
    sift = {};  

    for i = 1:size(images_gray, 3)
        
        fprintf('Finding corners with Harris & detecting SIFT, image %d\n', i)
        
        % Get SIFT features based on Harris corners
        % [features, descriptors]
        [sift{1, i}, sift{2, i}] = get_sift_harris(images_gray(:, :, i), sigma, threshold_R);

    end
    
    % Save
    save('features/sift', 'sift')
    
else
    
    disp('Loading features...')
    
    % Load already detected features
    load('features/sift', 'sift')
    
end


%%  Normalized 8-point RANSAC to find best matches (4 pts)

% Only do if file doesn't exist already
if ~exist('matches/matches_8pt_RANSAC.mat', 'file')
    
    % Cell array of matches per frame pair
    matches_8pt_RANSAC = {};
    
    % Loop over images
    for i = 1:size(sift, 2)

        fprintf('8-point RANSAC, image %d\n', i)
        
        % Do 8-point RANSAC
        [F_ransac_denorm, inliers_1, inliers_2, inliers_idx] = do_eightpoint(sift, match_threshold, dist_threshold, i);
        matches_8pt_RANSAC{1, i} = inliers_idx;
        
        % Plot one of the 'problematic' images
        if i == 1
                            
            showMatchedFeatures(images(:, :, :, i), images(:, :, :, i+1), inliers_1, inliers_2)
            plot_eightpoint(images(:, :, :, i), images(:, :, :, i+1), inliers_1, inliers_2, F_ransac_denorm);
            
        end

    end

    save('matches/matches_8pt_RANSAC', 'matches_8pt_RANSAC')
    
else
    
    load('matches/matches_8pt_RANSAC', 'matches_8pt_RANSAC')
    
end

% If you want to plot a specific pair
% showMatchedFeatures(images(:, :, :, 15), images(:, :, :, 16), sift{1, 15}(1:2, matches_8pt_RANSAC{1, 15}(1, :))', sift{1, 16}(1:2, matches_8pt_RANSAC{1, 15}(2, :))')


%% Chaining (8 pts)

point_view_matrix = chaining(matches_8pt_RANSAC);


%% Stitching (12 pts)

% Cells to store 3D point set for each set of frames & set of points that
%   are common between 2 consecutive sets
S = {};
points_common = {};
colors = {};

% Use 4 consecutive frames each time
for f = 0:size(point_view_matrix, 1) - 1
    
    % Shift (cell) array circularly (current set of frames)
    pv_matrix_circ = circshift(point_view_matrix, -f, 1);
    sift_circ = circshift(sift, -f, 2);
    
    % Shift one more (next set of frames)
    pv_matrix_circ_next = circshift(point_view_matrix, -f-1, 1);
    sift_circ_next = circshift(sift, -f-1, 2); 
    
    % Get x, y for each SIFT descriptor (for current & next set)
    points = get_points(sift_circ(1, 1:4), pv_matrix_circ(1:4, :));
    points_next = get_points(sift_circ_next(1, 1:4), pv_matrix_circ_next(1:4, :));
    
    % Next set is shifted by 1 w.r.t. current set, so look for points that
    %   are present in the last 3 frames of current set, and first 3 frames
    %   of next set --> you end up with points that were visible for 5
    %   consecutive frames, which form the connection between both sets
    % 1:end-2 and 3:end since rows are alternating x & y
    [~, points_common{2, f+1}, points_common{1, f+1}] = intersect(points_next(1:end-2, :)', points(3:end, :)', 'rows');
    
    % Get color for later plotting
    color = [images(sub2ind(size(images), uint16(points(2, :)), uint16(points(1, :)), ones([1, size(points, 2)]), f+1 * ones([1, size(points, 2)]))); ...
              images(sub2ind(size(images), uint16(points(2, :)), uint16(points(1, :)), 2*ones([1, size(points, 2)]), f+1 * ones([1, size(points, 2)]))); ...
              images(sub2ind(size(images), uint16(points(2, :)), uint16(points(1, :)), 3*ones([1, size(points, 2)]), f+1 * ones([1, size(points, 2)])))];
    
    % Only do if there are at least 3 points
    if size(points, 2) > 2
    
        % Perform structure-from-motion and solve for affine ambiguity
        S{1, f+1} = SfM(points);
        colors{1, f+1} = color;
        
    end
    
end

% Stitch to this first set
S_final = S{1, 1};
colors_final = colors{1, 1};

% Also try with pointCloud object
scene = pointCloud(S{1, 1}', 'Color', uint8(colors{1, 1}'));
merge_size = 0.15;

% Go over the sets
for s = 0:size(S, 2) - 1
    
    % Shift cell array circularly
    S_circ = circshift(S, -s, 2);
    colors_circ = circshift(colors, -s, 2);
    
    % Minimum number of rows, to check if both sets > 0 points
    min_cols = min(cellfun('size', S_circ(1, 1:2), 2));
    
    % Both sets need to have > 0 points
    if min_cols > 0
        
        % Save colors
        colors_final = [colors_final colors_circ{1, 2}];
        
        % Get transformation between 3D point sets
        % Transpose since x, y, z have to be columns
        [~, ~, tform] = procrustes(S_circ{1, 1}(:, points_common{1,s+1})', S_circ{1, 2}(:, points_common{2,s+1})');
        
        % All rows are equal for c --> convert to 1 row to allow pointwise
        %   ops
        tform.c = tform.c(1, :);
        
        % Do transform, see MATLAB's procrustes documentation
        Z = tform.b * S_circ{1, 2}' * tform.T + tform.c;
        
        % Convert to pointCloud
        new_cloud = pointCloud(Z, 'Color', uint8(colors_circ{1, 2}'));
        
        % Extend final 3D point set --> check dimensions
        S_final = [S_final Z'];
        
        % Also merge clouds
        scene = pcmerge(scene, new_cloud, merge_size);
        
    end
    
end

% Check for close points in some way? --> can be done using pcmerge


%% Bundle adjustment (4 pts)




%% Eliminate affine ambiguity (4 pts)

% Already in SfM?


%% 3D model plotting (4 pts)

plot3(S_final(1, :), S_final(2, :), S_final(3, :), 'b.')