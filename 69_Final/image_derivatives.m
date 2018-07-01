function F = image_derivatives(image, sigma, type)
% IMAGE_DERIVATIVES Computes derivatives of grayscale images.
%
% Inputs:
% - image: grayscale image
% - sigma: standard deviation of Gaussian to use
% - type: type of derivative (x, y, xx, yy, xy, yx)
%
% Outputs:
% - F: image convolved with Gaussian kernel
%
% Jesse Hagenaars & Michiel Mollema - 01.07.2018

% Get Gaussian
G = gaussian(sigma);

if type == 'x'
    F = conv2(image, gaussian_der(G, sigma), 'same');
    
elseif type == 'y'
    F = conv2(image, gaussian_der(G, sigma)', 'same');
    
elseif all(type == 'xx')
    F = conv2(image, gaussian_der_der(G, sigma), 'same');
    
elseif all(type == 'yy')
    F = conv2(image, gaussian_der_der(G, sigma)', 'same');
    
elseif all(type == 'xy')
    F = conv2(gaussian_der(G, sigma), gaussian_der(G, sigma), image, 'same');
    
elseif all(type == 'yx')
    F = conv2(gaussian_der(G, sigma), gaussian_der(G, sigma), image, 'same');
    
end
end