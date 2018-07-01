function [Ix, Iy] = grad_mag(image, sigma)
% GRAD_MAG Computs magnitudes of image gradients using Gaussian derivative
%   kernels.
%
% Inputs:
% - image: grayscale image
% - sigma: standard deviation of Gaussian to use
%
% Outputs:
% - Ix: gradient magnitude in x
% - Iy: gradient magnitude in y
%
% Jesse Hagenaars & Michiel Mollema - 01.07.2018

G = gaussian(sigma);

Ix = conv2(image, gaussian_der(G, sigma), 'same');
Iy = conv2(image, gaussian_der(G, sigma)', 'same');

end