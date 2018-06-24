function [r, c] = harris(im, sigma, threshold_R)
% inputs: 
% im: double grayscale image
% sigma: integration-scale
% outputs:  The row and column of each point is returned in r and c
% This function finds Harris corners at integration-scale sigma.
% The derivative-scale is chosen automatically as gamma*sigma

gamma = 0.7; % The derivative-scale is gamma times the integration scale

% Calculate Gaussian Derivatives at derivative-scale
% Hint: use your previously implemented function in assignment 1 
[Ix, Iy] = gradMag(im, sigma*gamma);

% Allocate an 3-channel image to hold the 3 parameters for each pixel
M = zeros(size(Ix,1), size(Ix,2), 3);

% Calculate M for each pixel
% Window function of ones
M(:,:,1) = Ix.^2;
M(:,:,2) = Ix .* Iy;
M(:,:,3) = Iy.^2;

% Smooth M with a gaussian at the integration scale sigma.
M = imfilter(M, fspecial('gaussian', ceil(sigma*6+1), sigma), 'replicate', 'same');
M = M * (sigma*gamma)^2;  % multiply with square of sigma_D

% Compute the cornerness R
trace = M(:,:,1) + M(:,:,3);
det = M(:,:,1).*M(:,:,3) - M(:,:,2).^2;

k = 0.05;
R = det - k * trace.^2;

% Set the threshold
% threshold = 0.01 * max(max(R));
threshold = threshold_R * max(max(R));

% Find local maxima
% Dilation will alter every pixel except local maxima in a 3x3 square area.
% Also checks if R is above threshold
R = ((R>threshold) & ((imdilate(R, strel('square', 3))==R))) ; %.* sigma;

% Display corners
% figure
% imshow(R,[]);

% Return the coordinates
[r,c] = find(R);

end
