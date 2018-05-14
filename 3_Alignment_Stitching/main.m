% Assignment 3: Image Alignment and Stitching
% Jesse Hagenaars & Michiel Mollema - 19-03-2018

clear; clc

%% Load VLFeat
try
    vl_version
catch
    VLFEATROOT = 'C:\Users\jesse\Documents\MATLAB\vlfeat';
    run([VLFEATROOT '\toolbox\vl_setup'])
end

%% Load Images

imageDir = 'boat\';
images = dir([imageDir '\*.pgm']);

image1 = imread([imageDir images(1).name]);
image2 = imread([imageDir images(2).name]);

%% Image Alignment

% Get transform
[xBest, ~, transformedImage1, transformedImage2] = getTransform(image1, image2, 100, 10);

% Plot transforms
figure
subplot(2,2,1); imshow(image1); title('Image 1')
subplot(2,2,2); imshow(image2); title('Image 2')
subplot(2,2,3); imshow(transformedImage1); title('Image 2 Transformed')
subplot(2,2,4); imshow(transformedImage2); title('Image 1 Transformed')

%% Image Stitching

imLeft  = rgb2gray(imread('left.jpg'));
imRight = rgb2gray(imread('right.jpg'));
stitchedImage = getStitch(imLeft, imRight);

figure
imshow(stitchedImage)
title('Stitched Image')

%% Optional Keypoint Plotting

% % Create composed figure
% imtot = cat(2, image1, image2);
% figure
% imshow(imtot);
% title('Keypoint Matching Image 1 and Image 2')
% 
% % Plot the keypoints from image 1
% fa1 = frames1(:,matches(1,1:50));
% h1  = vl_plotframe(fa1);
% h2  = vl_plotframe(fa1);
% set(h1, 'color', 'k', 'linewidth', 3);
% set(h2, 'color', 'y', 'linewidth', 2);
% 
% % Plot the keypoints from image 2
% fb1 = frames2New(:,matches(2,1:50));
% fb1(1,:) = fb1(1,:) + length(image1);
% h1 = vl_plotframe(fb1);
% h2 = vl_plotframe(fb1);
% set(h1, 'color', 'k', 'linewidth', 3);
% set(h2, 'color', 'y', 'linewidth', 2);
% hold on;
% 
% for idx = 1:numel(fa1(1,:))
%     
%     X_lines = [fa1(1,idx),fb1(1,idx)];
%     Y_lines = [fa1(2,idx),fb1(2,idx)];
%     plot(X_lines,Y_lines)
%     
% end


