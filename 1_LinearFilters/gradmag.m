function [Ix, Iy] = gradmag(image, sigma)

% Ix = (image(2:end,:) - image(1:end-1,:)) / 2;
% Iy = (image(:,2:end) - image(:,1:end-1)) / 2;

G = gaussian(sigma);

Ix = normalConv(image, gaussianDer(G, sigma));
Iy = normalConv(image, gaussianDer(G, sigma)');

end