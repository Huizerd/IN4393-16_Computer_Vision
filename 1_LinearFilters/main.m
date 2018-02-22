% Assignment 1: Linear Filters: Gaussians and Derivatives
% Jesse Hagenaars and Michiel Mollema - 19-02-2018
clc; clear; close all

image = imread('zebra.png');

sigma = 1:5:21;
magnitudeM   = zeros(size(image, 1), size(image, 2), length(sigma));
orientationM = magnitudeM;

for i = 1:length(sigma)

    % Our blurring
    imOut  = uint8(round(gaussianConv(image, sigma(i), sigma(i))));

    % MATLAB's blurring
    imOut2 = imgaussfilt(image, sigma(i));
    
    % Difference
%     figure('Name', 'Difference')
%     imshow(imOut2-imOut)

    % Gradients
    [Ix, Iy] = gradmag(image, sigma(i));
    
    magnitudeM(:,:,i)   = mean(sqrt(Ix.^2 + Iy.^2), 3);
    orientationM(:,:,i) = mean(atan2(Iy, Ix), 3);

end

% figure('Name', 'Orientation 1')
% imshow(orientationM(:,:,1), [-pi pi])
% colormap(hsv)
% colorbar
% 
% figure('Name', 'Orientation end')
% imshow(orientationM(:,:,end), [-pi pi])
% colormap(hsv)
% colorbar
% 
% figure('Name', 'Magnitude 1')
% imshow(magnitudeM(:,:,1))
% colorbar
% 
% figure('Name', 'Magnitude end')
% imshow(magnitudeM(:,:,end))
% colorbar

% Set threshold
threshold = 0:0.2:2;

% for j = 1:length(threshold)
%     
%     binary = magnitudeM(:,:,end) > threshold(j);
%     figure('Name', 'Threshold end')
%     imshow(binary)
%     colorbar
%     
% end

sz = 11;
impulse = zeros(sz);
impulse(ceil(sz/2), ceil(sz/2)) = 255;
impulse = cat(3, impulse, impulse, impulse);

sigma = sigma(1);

x = ImageDerivatives(impulse, sigma, 'x');
y = ImageDerivatives(impulse, sigma, 'y');
xx = ImageDerivatives(impulse, sigma, 'xx');
yy = ImageDerivatives(impulse, sigma, 'yy');
xy = ImageDerivatives(impulse, sigma, 'xy');
yx = ImageDerivatives(impulse, sigma, 'yx');