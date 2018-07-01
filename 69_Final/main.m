% IN4393-16: Final Assignment
% 
% Jesse Hagenaars & Michiel Mollema - 01.07.2018

clc; clear; close all

% run('C:/Users/jesse/Documents/MATLAB/vlfeat/toolbox/vl_setup')
% run('/home/michiel/Programs/MATLAB/vlfeat/toolbox/vl_setup')


%% Settings

% For RANSAC & matching
match_threshold = 1.5;  % lower = more matches
dist_threshold = 8;  % lower = less inliers
iter = 10000;

% For Harris
sigma = 1.2.^(-9:12);  % sigma scales
threshold_R = 1e-5;  % cornerness threshold
do_edges = 0;  % output of Canny as input

% Use Oxford SIFT as well
do_oxford = 1;

% Bundle adjustment
do_local_BA = 1;  % locally per set of 3-4 images
do_incremental_BA = 0;  % between set last stitched to main view and current set


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

 
%% Feature point detection & extraction of SIFT descriptors (2 pts)

% Harris-affine & Hessian-affine SIFT obtained using detectors from
%   http://www.robots.ox.ac.uk/~vgg/research/affine/detectors.html
% Obtained with a threshold of 10
harris_files = dir('features/*.png.haraff.sift');
hessian_files = dir('features/*.png.hesaff.sift');

% Only do if file doesn't exist already
if ~exist('features/sift_final.mat', 'file')
    
    sift = {};  

    for i = 1:size(images_gray, 3)
        
        fprintf('Finding corners with Harris & detecting SIFT, image %d...\n', i)
        
        % Detect edges first if do_edges
        if do_edges
            image = edge(images_gray(:, :, i), 'Sobel');
        else
            image = images_gray(:, :, i);
        end
        
        % Get SIFT features based on Harris corners
        % [features, descriptors]
        [sift{1, i}, sift{2, i}] = get_sift_harris(image, images_gray(:, :, i), sigma, threshold_R);
        
        % Load SIFT from Oxford if do_oxford
        if do_oxford
            
            sift_harris_oxford = dlmread([harris_files(i).folder '/' harris_files(i).name], ' ', 2, 0);
            sift_hessian_oxford = dlmread([hessian_files(i).folder '/' hessian_files(i).name], ' ', 2, 0);
            oxford_combined = cat(1, sift_harris_oxford, sift_hessian_oxford);
            
            % Concatenate x, y and descriptor
            sift{1, i} = [sift{1, i} oxford_combined(:, 1:2)'];
            sift{2, i} = [sift{2, i} oxford_combined(:, 6:end)'];      
            
        end
    end
    
    % Save
    save('features/sift_final', 'sift')
    
else
    
    disp('Loading features...')
    
    % Load already detected features
    load('features/sift_final', 'sift')
    
end


%%  Normalized 8-point RANSAC to find best matches (5 pts)

% Only do if file doesn't exist already
if ~exist('matches/matches_final.mat', 'file')
    
    % Cell array of matches per frame pair
    matches_8pt_RANSAC = {};  % matches after 8-point RANSAC
    matches = {};  % matches before RANSAC (within bounding box)
    
    % Loop over images
    for i = 1:size(sift, 2)

        fprintf('8-point RANSAC, image %d...\n', i)
        
        % Do 8-point RANSAC
        [~, inliers_1, inliers_2, matches_8pt_RANSAC{1, i}, matches{1, i}] = do_eightpoint(sift, match_threshold, dist_threshold, iter, i);
        
        % Plot matches for each pair
        if i ~= size(sift, 2)
            figure;
            showMatchedFeatures(images(:, :, :, i), images(:, :, :, i+1), inliers_1, inliers_2)
        else
            figure;
            showMatchedFeatures(images(:, :, :, i), images(:, :, :, 1), inliers_1, inliers_2)
        end
    end
    
    % Save
    save('matches/matches_bR_final', 'matches')
    save('matches/matches_final', 'matches_8pt_RANSAC')
    
else
    
    disp('Loading matches...')
    
    % Load already matched & RANSAC'ed features
    load('matches/matches_final', 'matches_8pt_RANSAC') 
    
end


%% Chaining (10 pts)

disp('Chaining...')

point_view_matrix = chaining(matches_8pt_RANSAC);


%% Stitching + eliminate affine ambiguity + bundle adjustment (13 + 5 + 4 pts)

disp('Stitching...')

% Construct sets of 3/4 frames if they don't already exist
if ~exist('models/triple_models.mat', 'file') || ~exist('models/quad_models.mat', 'file')

    % Cells to store 3D point set for each set of frames & set of points
    %   that are common between 2 consecutive sets
    S = {};
    M = {};
    affine_amb_solved = zeros(2, 19);  % indicate whether solved or not
    points_center = {};  % centered image coordinates
    pvm_dense = {};  % dense subblocks in point-view matrix
    colors = {};  % keep track of colors
    reprojection_error_before = {};  % keep track of reprojection errors before and after local BA
    reprojection_error_after = {};
    
    % Use n consecutive frames each time
    consec = [3 4];
    count = 0;
    
    % Loop over frames and combinations of 3 and 4
    for n = 1:length(consec)
        for f = 0:size(point_view_matrix, 1) - 1

            % Shift (cell) array circularly (current set of frames)
            pv_matrix_circ = circshift(point_view_matrix, -f, 1);
            sift_circ = circshift(sift, -f, 2);

            % Get x, y for each SIFT descriptor
            point = get_points(sift_circ(1, 1:consec(n)), pv_matrix_circ(1:consec(n), :));

            % Write dense subblock
            pvm_dense{n, f+1} = pv_matrix_circ(1:consec(n), :);
            pvm_dense{n, f+1}(:, ~all(pvm_dense{n, f+1}, 1)) = [];

            % Get color for later plotting
            color = [images(sub2ind(size(images), uint16(point(2, :)), uint16(point(1, :)), ones([1, size(point, 2)]), f+1 * ones([1, size(point, 2)]))); ...
                     images(sub2ind(size(images), uint16(point(2, :)), uint16(point(1, :)), 2 * ones([1, size(point, 2)]), f+1 * ones([1, size(point, 2)]))); ...
                     images(sub2ind(size(images), uint16(point(2, :)), uint16(point(1, :)), 3 * ones([1, size(point, 2)]), f+1 * ones([1, size(point, 2)])))];

            % Only do if there are at least 3 points
            if size(point, 2) > 2

                % Center points, get number of cameras and points
                N_cam = size(point, 1) / 2; N_pt = size(point, 2);
                point_center = point - repmat(sum(point, 2) / N_pt, 1, N_pt);

                % Perform structure-from-motion and solve for affine ambiguity
                [M{n, f+1}, S{n, f+1}, affine_amb_solved(n, f+1)] = sfm(point_center);
                
                % Save colors and centered image coordinates
                colors{n, f+1} = color;
                points_center{n, f+1} = point_center;

                % Do local bundle adjustment
                if do_local_BA
                    
                    % Reprojection error before
                    reprojection_error_before{n, f+1} = sum(sum(abs(point_center - M{n, f+1} * S{n, f+1})));
                    
                    % Reshape as 1 vector to allow concurrent optimization
                    %   of both M & S
                    MS_0 = [S{n, f+1}(:); M{n, f+1}(:)];
                    options = optimoptions(@fminunc, 'StepTolerance', 1e-16, 'OptimalityTolerance', 1e-16, 'FunctionTolerance', 1e-16, 'MaxFunctionEvaluations', 1e8, 'Display', 'iter', 'PlotFcn', ...
                                           {@optimplotx,@optimplotfval,@optimplotresnorm,@optimplotstepsize});
                    MS = fminunc(@(x)bundle_adjustment(point_center, x, N_cam, N_pt), MS_0, options);

                    % Reshape back
                    S_BA = reshape(MS(1:N_pt*3), [3 N_pt]);
                    M_BA = reshape(MS(end-N_cam*6+1:end), [2*N_cam 3]);
                    
                    % Reprojection error after
                    reprojection_error_after{n, f+1} = sum(sum(abs(point_center - M_BA * S_BA)));
                    
                    % Put in final array
                    S{n, f+1} = S_BA;
                    M{n, f+1} = M_BA;                    

                end
            end
        end

        % Increment count to write in next 2 rows
        count = count + 1;

    end

    % Construct models
    triple_models = {};
    quad_models = {};

    for v = 1:size(S, 2)

        triple_models{v, 1} = S{1, v};
        triple_models{v, 2} = pvm_dense{1, v};
        triple_models{v, 3} = colors{1, v}';
        triple_models{v, 4} = M{1, v};
        triple_models{v, 5} = points_center{1, v};
        
        quad_models{v, 1} = S{2, v};
        quad_models{v, 2} = pvm_dense{2, v};
        quad_models{v, 3} = colors{2, v}';
        quad_models{v, 4} = M{2, v};
        quad_models{v, 5} = points_center{2, v};
        
        if do_local_BA
            triple_models{v, 6} = reprojection_error_before{1, v};
            triple_models{v, 7} = reprojection_error_after{1, v};
            quad_models{v, 6} = reprojection_error_before{2, v};
            quad_models{v, 7} = reprojection_error_after{2, v};
        end

    end
    
    % Save
    save('models/triple_models', 'triple_models')
    save('models/quad_models', 'quad_models')
    
else
    
    % Load
    load('models/triple_models', 'triple_models')
    load('models/quad_models', 'quad_models')
    
end

% Perform the actual stitching of the sets
[complete_S, complete_colors, complete_M, complete_D] = stitching(triple_models, quad_models, do_incremental_BA);


%% Accuracy of reconstruction (1 pt)

% ??


%% 3D model plotting (4 pts)

disp('Plotting...')

% Rotation matrices to get castle upright
theta = pi/4;
phi = pi/8;
Rx = [1 0 0;...
    0 cos(theta) -sin(theta);...
    0 sin(theta) cos(theta)];
Ry = [cos(phi) 0 sin(phi);...
      0 1 0;...
     -sin(phi) 0 cos(phi)];
location = Ry * Rx * complete_S;

% Define a pointcloud with color
scene = pointCloud(location', 'Color', complete_colors);

% Reduce noise in point cloud
[scene, ~] = pcdenoise(scene, 'NumNeighbors', 200, 'Threshold', 0.5);  % 0.01 for less accurate but more complete surface plot

% Show pointcloud plot
figure;
pcshow(scene, 'MarkerSize', 50)

% Show surface plot
figure;
P = scene.Location;
x = P(:,1);
y = P(:,2);
z = P(:,3);

% Create boundaries for surfaces
[~, S] = alphavol(P, 30);  % 50 for less accurate but more complete surface plot
colors = double(scene.Color);
trisurf(S.bnd, x, y, z, (1:length(x)), 'LineStyle', 'none');
colormap((colors)./255)
