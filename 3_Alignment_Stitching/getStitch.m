function [stitchedImage] = getStitch(image1, image2)
%% Find Best Transform

% T is transform from image2 -> image1
[xBest, T, transformedImage1, transformedImage2] = getTransform(image1, image2, 100, 10);

%% Image Size

[xLim, yLim] = outputLimits(T, [1 size(image2, 2)], [1 size(image2, 1)]);

xMin = min([1 xLim]); yMin = min([1 yLim]);
xMax = max([size(image1,1), xLim]); yMax = max([size(image1, 2), yLim]);


stitchWidth  = round(xMax - xMin);
stitchHeight = round(yMax - yMin);

stitchedImage = zeros(stitchHeight, stitchWidth, 'like', image1);

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([stitchHeight stitchWidth], xLimits, yLimits);

%% Create the panorama

% Transform I into the panorama.
warpedImage = imwarp(image2, T, 'OutputView', panoramaView);

% Generate a binary mask.
mask = imwarp(true(size(image2,1),size(image2,2)), T, 'OutputView', panoramaView);

% Overlay the warpedImage onto the panorama.
stitchedImage = step(blender, stitchedImage, warpedImage, mask);

stitchedImage(1:size(image1, 1), 1:size(image1, 2)) = image1;
end

