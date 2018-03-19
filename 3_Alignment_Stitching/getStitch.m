function [stitchedImage] = getStitch(image1, image2)

%% Find Best Transform

[xBest, transformedImage1, transformedImage2] = getTransform(image1, image2, 100, 10);

