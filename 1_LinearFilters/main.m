% Assignment 1: Linear Filters: Gaussians and Derivatives
% Jesse Hagenaars and Michiel Mollema - 19-02-2018
clc; clear; close all

% Only first layer (all are equal)
image = imread('zebra.png');
image = image(:,:,1);

sigma = 1:6;

% Empty matrices for loops
magnitudeM   = zeros(size(image, 1), size(image, 2), length(sigma));
orientationM = magnitudeM;

figure('Name', 'Difference')

for i = 1:length(sigma)

    % Our blurring
    imOut  = uint8(round(gaussianConv(image, sigma(i), sigma(i))));

    % MATLAB's blurring
    imOut2 = imgaussfilt(image, sigma(i));
    
    % Difference
    subplot(2, 3, i)
    imshow(imOut2-imOut)
    title(['Difference for \sigma = ' num2str(sigma(i))])

    % Gradients
    [Ix, Iy] = gradMag(image, sigma(i));
    
    magnitudeM(:,:,i)   = sqrt(Ix.^2 + Iy.^2);
    orientationM(:,:,i) = atan2(Iy, Ix);

end

% Set threshold
threshold = 2.5:0.5:4;

figure('Name', 'Magnitude with Threshold')
for j = 1:length(threshold)
    
    binary = magnitudeM(:,:,end) > threshold(j);
    
    subplot(2, 2, j)
    imshow(binary)
    title(['Magnitude, threshold = ' num2str(threshold(j))])
    
end

sz = 31;
impulse = zeros(sz);
impulse(ceil(sz/2), ceil(sz/2)) = 255;

sigma = sigma(1);

figure('Name', 'Derivatives')
derivatives = ["x", "y", "xx", "yy", "xy", "yx"];

for k = 1:length(derivatives)
    
    derIm = imageDerivatives(impulse, sigma, derivatives(k));
    subplot(2, 3, k)
    imshow(derIm,'InitialMagnification',200)
    title(derivatives(k))
    
end
