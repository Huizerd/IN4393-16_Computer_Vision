% IN4393-16: Final Assignment
% 
% Jesse Hagenaars & Michael Mollema - 04.06.2018

%% Load data

image_files = dir('model_castle/*.jpg');

for i = 1:length(image_files)
    
    current_image = imread([image_files(i).folder '/' image_files(i).name]);
    
    if i == 1
        images = zeros([size(current_image) length(image_files)]);
    end
    
    images(:, :, :, i) = current_image;
   
end

%% Feature point detection & extraction of SIFT descriptors (4 pts)


%%  Normalized 8-point RANSAC to find best matches (4 pts)


%% Chaining (8 pts)


%% Stitching (12 pts)


%% Bundle adjustment (4 pts)


%% Eliminate affine ambiguity (4 pts)


%% 3D model plotting (4 pts)