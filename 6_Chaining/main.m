%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main file for assignment 6: Matching
% Jesse Hagenaars & Michiel Mollema - 28-05-2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc

% run('C:\Users\jesse\Documents\MATLAB\vlfeat\toolbox\vl_setup')
run('/home/michiel/Programs/MATLAB/vlfeat/toolbox/vl_setup')

threshold = 0.1;
nMatches = 100;

%% Inputs
images = ["TeddyBearPNG/obj02_001.png", "TeddyBearPNG/obj02_002.png",...
    "TeddyBearPNG/obj02_003.png", "TeddyBearPNG/obj02_004.png",...
    "TeddyBearPNG/obj02_005.png", "TeddyBearPNG/obj02_006.png",...
    "TeddyBearPNG/obj02_007.png", "TeddyBearPNG/obj02_008.png",...
    "TeddyBearPNG/obj02_009.png", "TeddyBearPNG/obj02_010.png",...
    "TeddyBearPNG/obj02_011.png", "TeddyBearPNG/obj02_012.png",...
    "TeddyBearPNG/obj02_013.png", "TeddyBearPNG/obj02_014.png",...
    "TeddyBearPNG/obj02_015.png", "TeddyBearPNG/obj02_016.png"];
features = ["features/obj02_001.png.harhes.sift", "features/obj02_002.png.harhes.sift",...
    "features/obj02_003.png.harhes.sift", "features/obj02_004.png.harhes.sift",...
    "features/obj02_005.png.harhes.sift", "features/obj02_006.png.harhes.sift",...
    "features/obj02_007.png.harhes.sift", "features/obj02_008.png.harhes.sift",...
    "features/obj02_009.png.harhes.sift", "features/obj02_010.png.harhes.sift",...
    "features/obj02_011.png.harhes.sift", "features/obj02_012.png.harhes.sift",...
    "features/obj02_013.png.harhes.sift", "features/obj02_014.png.harhes.sift",...
    "features/obj02_015.png.harhes.sift", "features/obj02_016.png.harhes.sift"];


%% Match features and use Normalized 8-point algorithm with RANSAC
newMatches = NaN(3, nMatches, length(features));
for i = 1 : length(features)
    disp(i)
    if i == length(features)
        [matches, A] = matching(features(i), features(1), nMatches);
    else
        tic;
        [matches, A] = matching(features(i), features(i + 1), nMatches);
    end
        
    % RANSAC 8-point
    [F_RANSAC, inliers] = eightpoint(A, threshold);
    newMatches(:, 1:length(inliers), i)  = matches(:, inliers);
end