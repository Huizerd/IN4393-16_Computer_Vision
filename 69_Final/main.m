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


%% Load data

disp('Loading data...')

% Images
image_files = dir('model_castle_png/*.png');

for i = 1:length(image_files)
    
    current_image = imread([image_files(i).folder '/' image_files(i).name]);
    
    % Initialize array based on image size
    if i == 1
        
        images = uint8(zeros([size(current_image) length(image_files)]));
        
    end
    
    images(:, :, :, i) = uint8(current_image);
   
end

% Convert to greyscale
images_grey = uint8(mean(images, 3));

% SIFT features, obtained using Harris-affine & Hessian-affine SIFT from
%   http://www.robots.ox.ac.uk/~vgg/research/affine/detectors.html
% feature_files = dir('features/concat_200_10/*.png.harhes.sift');
% harris_files = dir('features/extract_features2/*500.png.haraff.sift');
% hessian_files = dir('features/extract_features2/*500.png.hesaff.sift');
% sift_oxford = {};
% 
% for i = 1:length(harris_files)
%     
% %     current_features = dlmread([feature_files(i).folder '/' feature_files(i).name], ' ', 2, 0);
%     harris_features = dlmread([harris_files(i).folder '/' harris_files(i).name], ' ', 2, 0);
%     hessian_features = dlmread([hessian_files(i).folder '/' hessian_files(i).name], ' ', 2, 0);
%     current_features = cat(1, harris_features, hessian_features);
%     
%     sift_oxford{1, i} = current_features(:, 1:5)';
%     sift_oxford{2, i} = current_features(:, 6:end)';
%    
% end

 
%% Feature point detection & extraction of SIFT descriptors (4 pts)

% Only do if file doesn't exist already
if ~exist('features/sift_vlfeat.mat', 'file')
  
    % Using VLFeat's SIFT
    sift_vlfeat = {};
    results = {};
    
    sigma = 1.2.^(-9:12);

    for i = 1:size(images, 4)
        
        fprintf('Finding corners with Harris & detecting SIFT, image %d\n', i)
        
        % Gaussian filter to smooth image
%         smooth = uint8(conv2(gaussian(3), gaussian(3), images_grey(:, :, :, i), 'same'));
        
        % Detect edges
        edges = edge(images_grey(:, :, :, i), 'Sobel');
        
        % Get SIFT features based on Harris corners
        results{1, i} = getSIFT(edges, images_grey(:, :, :, i), sigma);
        
        % Plot results   
%         figure
%         imshow(images_grey(:, :, :, i)); hold on

%         for ii = 1:length(sigma)
%             
%             [r, c] = find(results{1, i}{ii, 2} == 1);
%             plot(c, r, 'g+')
%             
%         end
        
%         [sift_vlfeat{1, i}, sift_vlfeat{2, i}] = vl_sift(single(images_grey(:, :, i)));
        sift_vlfeat{1, i} = cat(2, results{1, i}{:, 5});
        sift_vlfeat{2, i} = cat(2, results{1, i}{:, 6});

    end
    
    % Save
    save('features/sift_vlfeat', 'sift_vlfeat')
    
else
    
    % Load already detected features
    load('features/sift_vlfeat', 'sift_vlfeat')
    
end

% Using Harris-affine & Hessian-affine SIFT from
%   http://www.robots.ox.ac.uk/~vgg/research/affine/detectors.html
%   --> loaded in above section


%%  Normalized 8-point RANSAC to find best matches (4 pts)

% Only do if file doesn't exist already
if ~exist('matches/matches_8pt_RANSAC.mat', 'file')
    
    % Cell array of matches per frame pair
    matches_8pt_RANSAC = {};
    
    % Loop over images
    for i = 1:size(sift_vlfeat, 2)

        fprintf('8-point RANSAC, image %d\n', i)
        
        % Do 8-point RANSAC
        [F_ransac_denorm, inliers_1, inliers_2, inliers_idx] = do_eightpoint(sift_vlfeat, match_threshold, dist_threshold, i);
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

%% Chaining (8 pts)

point_view_matrix = chaining(matches_8pt_RANSAC);


%% Stitching (12 pts)
% 
% % Cells to store 3D point set for each set of frames
% S = {};
% 
% % Use 4 consecutive frames each time
% for f = 0:size(point_view_matrix, 1) - 1
%     
%     % Shift (cell) array circularly
%     pv_matrix_circ = circshift(point_view_matrix, -f, 1);
%     sift_vlfeat_circ = circshift(sift_vlfeat, -f, 2);
%     
%     % Get x, y for each SIFT descriptor
%     points = get_points(sift_vlfeat_circ, pv_matrix_circ(1:4, :));
%     
%     if size(points, 2) > 2
%     
%         % Perform structure-from-motion and solve for affine ambiguity
%         S{1, f+1} = SfM(points);
%         
%     end
%     
% end
% 
% % Extend this with transformation
% S_final = S{1, 1};
% 
% % Stitch various 3D point sets together
% for s = 0:size(S, 2) - 1
%     
%     % Shift cell array circularly
%     S_circ = circshift(S, s, 2);
%     
%     % Minimum number of rows (equal rows needed for procrustes
%     min_rows = min(cellfun('size', S_circ(1, 1:2), 2));
%     
%     if min_rows > 0
%         
%         % Get transformation between 3D point sets
%         [d, Z] = procrustes(S_circ{1, 1}(:, 1:min_rows)', S_circ{1, 2}(:, 1:min_rows)');
% 
%         % Extend final 3D point set --> check dimensions
%         S_final = [S_final Z'];
%         
%     end
%     
% end
% 
% % Check for close points in some way?
% 
% 
% %% Bundle adjustment (4 pts)
% % Levenberg-Marquardt (sparse variant)
% 
% 
% 
% %% Eliminate affine ambiguity (4 pts)
% 
% % Already in SfM?
% 
% 
% %% 3D model plotting (4 pts)
% 
% plot3(S_final(1, :), S_final(2, :), S_final(3, :), 'b.')