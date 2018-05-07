% Assignment 5: Epipolar Geometry
% Jesse Hagenaars & Michiel Mollema - 07-05-2018

clear; clc

% run('C:\Users\jesse\Documents\MATLAB\vlfeat\toolbox\vl_setup')

images = ["obj02_001.png", "obj02_002.png"];
descriptors = ["obj02_001.png.harhes.sift", "obj02_002.png.harhes.sift"];

for i = 2:length(images)
    
    % Image 1
    output1 = dlmread(descriptors(i-1), ' ', 2, 0);
    
    x1 = output1(:, 1);
    y1 = output1(:, 2);
    a1 = output1(:, 3);
    b1 = output1(:, 4);
    c1 = output1(:, 5);
    desc1 = output1(:, 6:end);
    
    % Image 2
    output2 = dlmread(descriptors(i), ' ', 2, 0);
    
    x2 = output2(:, 1);
    y2 = output2(:, 2);
    a2 = output2(:, 3);
    b2 = output2(:, 4);
    c2 = output2(:, 5);
    desc2 = output2(:, 6:end);
    
    % Matching
    [matches, scores] = vl_ubcmatch(desc1, desc2);
    
    
end