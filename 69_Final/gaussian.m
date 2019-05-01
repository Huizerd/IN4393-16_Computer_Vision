function G = gaussian(sigma)
% GAUSSIAN Constructs a 1D Gaussian kernel.
%
% Inputs:
% - G: Gaussian kernel of same size
% - sigma: standard deviation of Gaussian to use
%
% Outputs:
% - Gd: 1D Gaussian kernel
%
% . & Michiel Mollema - 01.07.2018

% Range of kernel [-3*sigma, 3*sigma]
sz = floor(3*sigma + 0.5);
x = linspace(-3*sigma, 3*sigma, (2*sz+1));

% Construct kernel
G = 1 / (sigma * sqrt(2*pi)) * exp(-x.*x / (2*sigma*sigma));

% Normalize
G = G/sum(G);

end