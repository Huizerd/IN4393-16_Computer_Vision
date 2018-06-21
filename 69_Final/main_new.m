% IN4393-16: Final Assignment
% 
% Jesse Hagenaars & Michiel Mollema - 04.06.2018

clc; clear; %close all

run('C:/Users/jesse/Documents/MATLAB/vlfeat/toolbox/vl_setup')
% run('/home/michiel/Programs/MATLAB/vlfeat/toolbox/vl_setup')

%% Settings

% Higher = more strict
match_threshold = 1.25;
dist_threshold = 25;

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
if ~exist('features/sift_edges.mat', 'file')
    
    sift = {};  

    for i = 1:size(images_gray, 3)
        
        fprintf('Finding corners with Harris & detecting SIFT, image %d\n', i)
        
        % Gaussian filter to smooth image
        % smooth = uint8(conv2(gaussian(3), gaussian(3), images_gray(:, :, i), 'same'));
        
        % Detect edges
        edges = edge(images_gray(:, :, i), 'Sobel');
        
        % Get SIFT features based on Harris corners
        % [features, descriptors]
        [sift{1, i}, sift{2, i}] = get_sift_harris(edges, images_gray(:, :, i), sigma, threshold_R);
        
        % Also get VLFeat SIFT and concatenate
        % Potentially many false matches? --> allright
        % [sift_vlfeat_f, sift_vlfeat_d] = vl_sift(single(images_gray(:, :, i)));
        
        % sift{1, i} = [sift{1, i} sift_vlfeat_f];
        % sift{2, i} = [sift{2, i} sift_vlfeat_d];

    end
    
    % Save
    save('features/sift_edges', 'sift')
    
else
    
    disp('Loading features...')
    
    % Load already detected features
    load('features/sift_edges', 'sift')
    
end


%%  Normalized 8-point RANSAC to find best matches (4 pts)

% Only do if file doesn't exist already
if ~exist('matches/matches_8pt_RANSAC_125_10_edges.mat', 'file')
    
    % Cell array of matches per frame pair
    matches_8pt_RANSAC = {};
    
    % Loop over images
    for i = 1:size(sift, 2)

        fprintf('8-point RANSAC, image %d\n', i)
        
        % Do 8-point RANSAC
        [F_ransac_denorm, inliers_1, inliers_2, inliers_idx] = do_eightpoint(sift, match_threshold, dist_threshold, i);
        matches_8pt_RANSAC{1, i} = inliers_idx;
        
        if i ~= size(sift, 2)
            figure;
            showMatchedFeatures(images(:, :, :, i), images(:, :, :, i+1), inliers_1, inliers_2) 
        end
    end

    save('matches/matches_8pt_RANSAC_125_10_edges', 'matches_8pt_RANSAC')
    
else   
    load('matches/matches_8pt_RANSAC_125_10_edges', 'matches_8pt_RANSAC')   
end

% If you want to plot a specific pair
% showMatchedFeatures(images(:, :, :, 1), images(:, :, :, 2), sift{1, 1}(1:2, matches_8pt_RANSAC{1, 1}(1, :))', sift{1, 2}(1:2, matches_8pt_RANSAC{1, 1}(2, :))')
% bound_x = [885 3286 3286 885 885]; bound_y = [726 726 1906 1906 726]; hold on; plot(bound_x, bound_y)

%% Chaining (8 pts)

point_view_matrix = chaining(matches_8pt_RANSAC);


%% Stitching (12 pts)

% Cells to store 3D point set for each set of frames & set of points that
%   are common between 2 consecutive sets
S = {};
points = {};
points_common = {};
colors = {};

% Use n consecutive frames each time
consec = [3 4];
count = 0;

for n = 1:length(consec)
    for f = 0:size(point_view_matrix, 1) - 1

        % Shift (cell) array circularly (current set of frames)
        pv_matrix_circ = circshift(point_view_matrix, -f, 1);
        sift_circ = circshift(sift, -f, 2);

        % Shift one more (next set of frames)
        pv_matrix_circ_next = circshift(point_view_matrix, -f-1, 1);
        sift_circ_next = circshift(sift, -f-1, 2); 

        % Get x, y for each SIFT descriptor (for current & next set)
        point = get_points(sift_circ(1, 1:consec(n)), pv_matrix_circ(1:consec(n), :));
        point_next = get_points(sift_circ_next(1, 1:consec(n)), pv_matrix_circ_next(1:consec(n), :));

        % Next set is shifted by 1 w.r.t. current set, so look for points that
        %   are present in the last 3 frames of current set, and first 3 frames
        %   of next set --> you end up with points that were visible for 5
        %   consecutive frames, which form the connection between both sets
        % 1:end-2 and 3:end since rows are alternating x & y
        [K, points_common{n+count+1, f+1}, points_common{n+count, f+1}] = intersect(point_next(1:end-2, :)', point(3:end, :)', 'rows', 'stable');

        % Get color for later plotting
        color = [images(sub2ind(size(images), uint16(point(2, :)), uint16(point(1, :)), ones([1, size(point, 2)]), f+1 * ones([1, size(point, 2)]))); ...
                 images(sub2ind(size(images), uint16(point(2, :)), uint16(point(1, :)), 2*ones([1, size(point, 2)]), f+1 * ones([1, size(point, 2)]))); ...
                 images(sub2ind(size(images), uint16(point(2, :)), uint16(point(1, :)), 3*ones([1, size(point, 2)]), f+1 * ones([1, size(point, 2)])))];

        % Only do if there are at least 3 points
        if size(point, 2) > 2

            % Perform structure-from-motion and solve for affine ambiguity
            S{n, f+1} = SfM(point);
            colors{n, f+1} = color;
            points{n, f+1} = point;

        end
    end
    
    % Increment count to write in next 2 rows
    count = count + 1;
    
end

% % Stitch to this first set
% S_final = S{1, 1};
% colors_final = colors{1, 1};
% d_sum = 0;
% 
% % Also try with pointCloud object
% inverted_scene = [S{1, 1}(1, :); -S{1, 1}(3, :); -S{1, 1}(2, :)]';
% scene = pointCloud(inverted_scene, 'Color', uint8(colors{1, 1}'));
% merge_size = 0.01;
% 
% % Go over the sets
% for s = 0:size(S, 2) - 1
%     
%     % Shift cell array circularly
%     S_circ = circshift(S, -s, 2);
%     colors_circ = circshift(colors, -s, 2);
%     
%     % Minimum number of rows, to check if both sets > 0 points
%     % min_cols = min(cellfun('size', S_circ(1, 1:2), 2));
%     
%     % Common points has to be > 0
%     if ~isempty(points_common{1, s+1})
%         
%         if s ~= size(S, 2) -1
%             
%             % Plot matching points
%             view_1 = S_circ{1, 1}(:, points_common{1, s+1})';
%             view_2 = S_circ{1, 2}(:, points_common{2, s+1})';
% 
%             % Save colors
%             colors_final = [colors_final colors_circ{1, 2}];
% 
%         else
% 
%             % Plot matching points
%             view_1 = S_circ{1, 2}(:, points_common{2, s+1})';
%             view_2 = S_circ{1, 1}(:, points_common{1, s+1})';
% 
%             % Save colors
%             colors_final = [colors_final colors_circ{1, 1}];
% 
%         end
%         
%         % Get transformation between 3D point sets
%         [d, K, tform] = procrustes(view_1, view_2);
%         
%         % All rows are equal for c --> convert to 1 row to allow pointwise
%         %   ops
%         tform.c = tform.c(1, :);
%         
%         if s ~= size(S, 2) - 1
% 
%             % Do transform, see MATLAB's procrustes documentation
%             Z = tform.b * S_circ{1, 2}' * tform.T + tform.c;
% 
%         else
% 
%             % Do transform, see MATLAB's procrustes documentation
%             Z = tform.b * S_circ{1, 1}' * tform.T + tform.c;
% 
%         end
%         
%         % SAVE Z IN S FOR NEXT TRANSFORM
%         if s ~= size(S, 2) - 1
%             S{1, s+2} = Z';
%         else
%             S{1, end} = Z';
%         end
%         
%         if d < 0.1
%             
%             d_sum = d_sum + d;
%             
%             if s ~= size(S, 2) - 1
% 
%                 % Convert to pointCloud
%                 new_cloud = pointCloud([Z(:, 1) -Z(:, 3) -Z(:, 2)], 'Color', uint8(colors_circ{1, 2}'));
% 
%             else
% 
%                 % Convert to pointCloud
%                 new_cloud = pointCloud([Z(:, 1) -Z(:, 3) -Z(:, 2)], 'Color', uint8(colors_circ{1, 1}'));
% 
%             end
% 
%             % Extend final 3D point set --> check dimensions
%             S_final = [S_final Z'];
% 
%             % Also merge clouds
%             scene = pcmerge(scene, new_cloud, merge_size);
%             
%             figure
%             subplot(1, 2, 1)
%             scatter3(view_1(:, 1), view_1(:, 2), view_1(:, 3), 'b.')
%             hold on
%             scatter3(view_2(:, 1), view_2(:, 2), view_2(:, 3), 'r.')
%             scatter3(K(:, 1), K(:, 2), K(:, 3), 'g.')
%             title(num2str(d))
%             subplot(1, 2, 2)
%             pcshow(scene, 'MarkerSize', 100)
%             
%         end    
%     end  
% end

% stitching_new
stitching_new_2

% Check for close points in some way? --> can be done using pcmerge


%% Bundle adjustment (4 pts)




%% Eliminate affine ambiguity (4 pts)

% Already in SfM?


%% 3D model plotting (4 pts)

figure;
pcshow(scene, 'MarkerSize', 100)
title(num2str(d_sum));

