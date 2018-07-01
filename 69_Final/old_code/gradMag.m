function [Ix, Iy] = gradMag(image, sigma)

G = gaussian(sigma);

Ix = conv2(image, gaussianDer(G, sigma), 'same');
Iy = conv2(image, gaussianDer(G, sigma)', 'same');

end