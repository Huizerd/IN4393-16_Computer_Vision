function [Ix, Iy] = gradMag(image, sigma)

G = gaussian(sigma);

% 3 layer approach
% Ix = zeros(size(image));
% Iy = zeros(size(image));
% 
% for i = 1:size(image, 3)
%     Ix(:,:,i) = conv2(image(:,:,i), gaussianDer(G, sigma), 'same');
%     Iy(:,:,i) = conv2(image(:,:,i), gaussianDer(G, sigma)', 'same');
% end

Ix = conv2(image, gaussianDer(G, sigma), 'same');
Iy = conv2(image, gaussianDer(G, sigma)', 'same');

end