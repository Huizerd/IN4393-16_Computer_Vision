% IN4393-16: Final Assignment
% 
% Jesse Hagenaars & Michael Mollema - 04.06.2018

clc; clear; close all

if all(getenv('username') == 'jesse')
    run('C:/Users/jesse/Documents/MATLAB/vlfeat/toolbox/vl_setup')
else
    run('/home/michiel/Programs/MATLAB/vlfeat/toolbox/vl_setup')
end

%% Load data

% Images
image_files = dir('model_castle/*.jpg');

for i = 1:length(image_files)
    
    current_image = imread([image_files(i).folder '/' image_files(i).name]);
    
    if i == 1
        images = uint8(zeros([size(current_image) length(image_files)]));
    end
    
    images(:, :, :, i) = uint8(current_image);
   
end

% Convert to greyscale
images_grey = uint8(mean(images, 3));

% SIFT features, obtained using Harris-affine & Hessian-affine SIFT from
%   http://www.robots.ox.ac.uk/~vgg/research/affine/detectors.html
feature_files = dir('features/*.png.harhes.sift');
sift_oxford = {};

for i = 1:length(feature_files)
    
    current_features = dlmread([feature_files(i).folder '/' feature_files(i).name], ' ', 2, 0);
    
    sift_oxford{1, i} = current_features(:, 1:5)';
    sift_oxford{2, i} = current_features(:, 6:end)';
   
end
 
%% Feature point detection & extraction of SIFT descriptors (4 pts)

% Using VLFeat's SIFT
sift_vlfeat = {};

for i = 1:size(images, 4)
    
    [sift_vlfeat{1, i}, sift_vlfeat{2, i}] = vl_sift(single(images_grey(:, :, i)));
    
end

% Using Harris-affine & Hessian-affine SIFT from
%   http://www.robots.ox.ac.uk/~vgg/research/affine/detectors.html
%   --> loaded in above section


%%  Normalized 8-point RANSAC to find best matches (4 pts)


%% Chaining (8 pts)


%% Stitching (12 pts)


%% Bundle adjustment (4 pts)


%% Eliminate affine ambiguity (4 pts)


%% 3D model plotting (4 pts)