function Gdd = gaussian_der_der(G, sigma)
% GAUSSIAN_DER_DER Constructs a 1D Gaussian double derivative kernel.
%
% Inputs:
% - G: Gaussian kernel of same size
% - sigma: standard deviation of Gaussian to use
%
% Outputs:
% - Gd: 1D Gaussian double derivative kernel
%
% Jesse Hagenaars & Michiel Mollema - 01.07.2018

% Range of kernel [-3*sigma, 3*sigma]
sz = floor(3*sigma + 0.5);
x = linspace(-3*sigma, 3*sigma, (2*sz+1));

% Double derivative
Gdd = (-(sigma*sigma) + (x.*x)) ./ (sigma*sigma*sigma*sigma) .* G;

end