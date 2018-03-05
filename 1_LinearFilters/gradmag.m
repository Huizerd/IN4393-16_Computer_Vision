function [Ix, Iy] = gradMag(image, sigma)

% Ix = (image(2:end,:) - image(1:end-1,:)) / 2;
% Iy = (image(:,2:end) - image(:,1:end-1)) / 2;

G = gaussian(sigma);

Ix = conv2(image, gaussianDer(G, sigma), 'same');
Iy = conv2(image, gaussianDer(G, sigma)', 'same');

end